# SPDX-FileCopyrightText: 2026 XXX
# SPDX-License-Identifier: Apache-2.0 WITH SHL-2.1
#
# cocotb testbench for heichips26_RiMaX.
#
# RiMaX serialises the picorv32 memory interface onto the chip pins, so this
# testbench plays the part of the eFPGA on the other side of that link: it
# deserialises the requests, serves them out of a memory dictionary, and
# models an AXI UART Lite at 0x40600000.
#
# The program is embedded below, so there is no external firmware file.
# It exercises instruction fetch, a word store and load, the byte and half
# word strobes, and the FlotiMaX custom instruction, printing one character
# per passing test:
#
#     R i M a X   the core is fetching and executing
#     W           word store and load round trip
#     S           sb and sh carried the right MEM_WSTRB across the link
#     F           the FlotiMaX custom instruction returned 1.0 * 2.0

import os
import logging
from pathlib import Path

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles, Timer
from cocotb_tools.runner import get_runner

sim      = os.getenv("SIM", "icarus")
pdk_root = os.getenv("PDK_ROOT", Path("~/.ciel").expanduser())
pdk      = os.getenv("PDK", "ihp-sg13cmos5l")
scl      = os.getenv("SCL", "sg13cmos5l_stdcell")
gl       = os.getenv("GL", "0").strip().lower() in ("1", "true", "yes", "on")

hdl_toplevel = "heichips26_RiMaX"

CLK_FREQ_MHZ = 50
MEM_WORDS    = 1024
UART_BASE    = 0x4060_0000
REG_ADDR_HI  = 0x0000          # must match the RiMaX parameter

EXPECTED = b"RiMaX\r\nWSF\r\n"

# link encoding, matching pico_adapter_to_fpga
REG_RD, REG_W, MEM_RD, MEM_W = 0, 1, 2, 3
ADDR_L, ADDR_H, DATA_L, DATA_H = 0, 1, 2, 3

# ---------------------------------------------------------------------------
# embedded test program, one 32 bit word per entry, loaded at address 0
# ---------------------------------------------------------------------------
PROGRAM = [
    0x0180006f, 0x0082a303, 0x00837313, 0xfe031ce3, 0x00a2a223, 0x00008067,
    0x406002b7, 0x05200513, 0xfe5ff0ef, 0x06900513, 0xfddff0ef, 0x04d00513,
    0xfd5ff0ef, 0x06100513, 0xfcdff0ef, 0x05800513, 0xfc5ff0ef, 0x00d00513,
    0xfbdff0ef, 0x00a00513, 0xfb5ff0ef, 0x10000413, 0xdeadc3b7, 0xeef38393,
    0x00742023, 0x00042483, 0x00749663, 0x05700513, 0xf95ff0ef, 0x0aa00593,
    0x00b40023, 0x5cc00593, 0x00b41123, 0x00042483, 0x05ccc637, 0xeaa60613,
    0x00c49663, 0x05300513, 0xf6dff0ef, 0x3f800637, 0x400006b7, 0x06d61733,
    0x400007b7, 0x00f71663, 0x04600513, 0xf51ff0ef, 0x00d00513, 0xf49ff0ef,
    0x00a00513, 0xf41ff0ef, 0x0000006f,
]


def to_int(sig):
    """Read a signal, treating X and Z as 0.

    Before the core issues its first request, MEM_ADDR is undefined, so the
    TYPE field on uio_out[5:4] is X. stb is low at that point so nothing is
    sampled from it, but the value still has to be readable.
    """
    try:
        return int(sig.value)
    except ValueError:
        binstr = str(sig.value)
        cleaned = "".join("0" if c not in "01" else c for c in binstr)
        return int(cleaned, 2)


class LinkSlave:
    """The eFPGA side of the RiMaX link: deserialise, serve, respond."""

    F_COLLECT, F_SERVE, F_RSP_LO, F_RSP_HI, F_WACK = range(5)

    def __init__(self, dut, program):
        self.dut = dut
        self.log = logging.getLogger("rimax_link")
        self.mem = {i: w for i, w in enumerate(program)}
        self.uart = bytearray()

        self.state = self.F_COLLECT
        self.addr = 0
        self.wdata = 0
        self.wstrb = 0
        self.write = False
        self.rsp = 0

    # ---- pin access ------------------------------------------------------
    def _drive(self):
        """uio_in[0] FPGA_READY, [1] RD_STB, [2] WR_DONE; ui_in carries rdata."""
        ready = 1 if self.state == self.F_COLLECT else 0
        rdstb = 1 if self.state in (self.F_RSP_LO, self.F_RSP_HI) else 0
        wrdone = 1 if self.state == self.F_WACK else 0
        self.dut.uio_in.value = (wrdone << 2) | (rdstb << 1) | ready

        if self.state == self.F_RSP_LO:
            self.dut.ui_in.value = self.rsp & 0xFFFF
        elif self.state == self.F_RSP_HI:
            self.dut.ui_in.value = (self.rsp >> 16) & 0xFFFF
        else:
            self.dut.ui_in.value = 0

    def _sample(self):
        uio = to_int(self.dut.uio_out)
        oe = to_int(self.dut.uio_oe)
        # only the bits the chip actually drives are meaningful
        uio &= oe
        return {
            "type": (uio >> 4) & 0x3,
            "wstrb": (uio >> 8) & 0xF,
            "stb": (uio >> 12) & 0x1,
            "beat": (uio >> 13) & 0x3,
            "last": (uio >> 15) & 0x1,
            "payload": to_int(self.dut.uo_out),
        }

    # ---- request servicing ----------------------------------------------
    def _serve(self):
        word = self.addr >> 2
        if self.write:
            if (self.addr & 0xFFFF_0000) == UART_BASE:
                if (self.addr & 0xFFFF) == 0x0004:      # TX FIFO
                    self.uart.append(self.wdata & 0xFF)
            elif word < MEM_WORDS:
                cur = self.mem.get(word, 0)
                for b in range(4):
                    if self.wstrb & (1 << b):
                        mask = 0xFF << (8 * b)
                        cur = (cur & ~mask) | (self.wdata & mask)
                self.mem[word] = cur & 0xFFFF_FFFF
            self.state = self.F_WACK
        else:
            if (self.addr & 0xFFFF_0000) == UART_BASE:
                # status register at +8: TX empty, never full, no RX data
                self.rsp = 0x0000_0004 if (self.addr & 0xFFFF) == 0x0008 else 0
            else:
                self.rsp = self.mem.get(word, 0) if word < MEM_WORDS else 0xDEADBEEF
            self.state = self.F_RSP_LO

    async def run(self):
        """One step per clock edge, mirroring the Verilog model exactly."""
        self._drive()
        while True:
            await RisingEdge(self.dut.clk)
            if self.dut.rst_n.value == 0:
                self.state = self.F_COLLECT
                self._drive()
                continue

            s = self._sample()
            take = s["stb"] and self.state == self.F_COLLECT

            if self.state == self.F_COLLECT:
                if take:
                    p = s["payload"]
                    if s["beat"] == ADDR_L:
                        self.addr = (self.addr & 0xFFFF_0000) | p
                        if s["type"] in (REG_RD, REG_W):
                            self.addr = (REG_ADDR_HI << 16) | p
                    elif s["beat"] == ADDR_H:
                        self.addr = (p << 16) | (self.addr & 0xFFFF)
                    elif s["beat"] == DATA_L:
                        self.wdata = (self.wdata & 0xFFFF_0000) | p
                    elif s["beat"] == DATA_H:
                        self.wdata = (p << 16) | (self.wdata & 0xFFFF)
                    self.wstrb = s["wstrb"]
                    self.write = s["type"] in (REG_W, MEM_W)
                    if s["last"]:
                        self.state = self.F_SERVE
            elif self.state == self.F_SERVE:
                self._serve()
            elif self.state == self.F_RSP_LO:
                self.state = self.F_RSP_HI
            elif self.state == self.F_RSP_HI:
                self.state = self.F_COLLECT
            elif self.state == self.F_WACK:
                self.state = self.F_COLLECT

            self._drive()


async def start_up(dut):
    """Clock, reset, and the link slave running in the background."""
    period_ns = round(1000 / CLK_FREQ_MHZ, 4)
    cocotb.start_soon(Clock(dut.clk, period_ns, "ns").start())

    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0

    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 4)
    dut.rst_n.value = 1
    await RisingEdge(dut.clk)

    slave = LinkSlave(dut, PROGRAM)
    cocotb.start_soon(slave.run())
    return slave


async def run_until_output(dut, slave, nbytes, limit=400_000):
    for _ in range(limit):
        await RisingEdge(dut.clk)
        if len(slave.uart) >= nbytes:
            return True
    return False


@cocotb.test()
async def test_link_starts(dut):
    """After reset the core must fetch, which means a beat on the link."""
    logger = logging.getLogger("heichips26_RiMaX_tb")
    slave = await start_up(dut)

    for _ in range(200):
        await RisingEdge(dut.clk)
        await Timer(1, "ns")
        uio = to_int(dut.uio_out) & to_int(dut.uio_oe)
        if (uio >> 12) & 1:
            logger.info("first link beat seen")
            return

    assert False, "no beat appeared on the link within 200 cycles after reset"


@cocotb.test()
async def test_uio_oe_is_correct(dut):
    """uio[3:0] must be inputs and uio[15:4] outputs, so no pin floats."""
    await start_up(dut)
    await ClockCycles(dut.clk, 5)
    oe = to_int(dut.uio_oe)
    assert oe == 0xFFF0, f"uio_oe is 0x{oe:04x}, expected 0xfff0"


@cocotb.test()
async def test_program_output(dut):
    """Run the embedded program and check every byte it prints."""
    logger = logging.getLogger("heichips26_RiMaX_tb")
    slave = await start_up(dut)

    done = await run_until_output(dut, slave, len(EXPECTED))
    got = bytes(slave.uart)
    logger.info("UART output: %r", got)

    assert done, f"timeout, only {len(got)} of {len(EXPECTED)} bytes seen: {got!r}"
    assert got == EXPECTED, f"expected {EXPECTED!r}, got {got!r}"


@cocotb.test()
async def test_store_reached_memory(dut):
    """The word the program stores must actually land in the slave's memory."""
    slave = await start_up(dut)
    await run_until_output(dut, slave, len(EXPECTED))

    word = slave.mem.get(0x100 >> 2)
    assert word == 0x05CCBEAA, \
        f"[0x100] is {word:#010x} if written at all, expected 0x05ccbeaa"


def heichips26_RiMaX_runner():
    proj_path = Path(__file__).resolve().parent

    sources = []
    includes = [proj_path / "../../rtl/"]
    defines = {}

    if gl:
        sources.append(Path(pdk_root) / pdk / "libs.ref" / scl / "verilog" / f"{scl}.v")
        sources.append(Path(pdk_root) / pdk / "libs.ref" / scl / "verilog" / "sg13cmos5l_udp.v")
        sources.append(proj_path / f"../../final/nl/{hdl_toplevel}.nl.v")
    else:
        sources.append(proj_path / f"../../rtl/{hdl_toplevel}.v")

    build_args = []
    if sim == "icarus":
        build_args = ["-DSIM", "-gno-specify"]
    if sim == "verilator":
        build_args = ["--timing", "--trace", "--trace-fst", "--trace-structs"]

    runner = get_runner(sim)
    runner.build(
        sources=sources,
        hdl_toplevel=hdl_toplevel,
        defines=defines,
        always=True,
        includes=includes,
        build_args=build_args,
        waves=True,
        timescale=("1ns", "1fs"),
    )

    runner.test(
        hdl_toplevel=hdl_toplevel,
        test_module="heichips26_RiMaX_tb",
        plusargs=[],
        waves=True,
    )


if __name__ == "__main__":
    heichips26_RiMaX_runner()
