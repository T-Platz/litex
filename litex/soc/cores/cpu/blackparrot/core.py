# BlackParrot Chip core support for the LiteX SoC.
#
# Authors: Sadullah Canakci & Cansu Demirkiran  <{scanakci,cansu}@bu.edu>
# Copyright (c) 2019, Boston University
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#
#     * Redistributions in binary form must reproduce the above
#       copyright notice, this list of conditions and the following
#       disclaimer in the documentation and/or other materials provided
#       with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import os
from shutil import copyfile
from migen import *

from litex import get_data_mod
from litex.soc.interconnect import wishbone
from litex.soc.integration.soc import SoCRegion
from litex.soc.cores.cpu import CPU, CPU_GCC_TRIPLE_RISCV64

# Variants -----------------------------------------------------------------------------------------

CPU_VARIANTS = ["standard", "sim"]

# GCC Flags ----------------------------------------------------------------------------------------

GCC_FLAGS = {
    "standard": "-march=rv64imafd -mabi=lp64d ",
    "sim":      "-march=rv64imafd -mabi=lp64d ",
}

# BlackParrot --------------------------------------------------------------------------------------

class BlackParrot(CPU):
    family               = "riscv"
    name                 = "blackparrot"
    human_name           = "BlackParrotRV64[imafd]"
    variants             = CPU_VARIANTS
    data_width           = 64
    endianness           = "little"
    gcc_triple           = CPU_GCC_TRIPLE_RISCV64
    linker_output_format = "elf64-littleriscv"
    nop                  = "nop"
    io_regions           = {
        0x0000_0000: 0x7000_0000
    } # Origin, Length.

    # Memory Mapping.
    @property
    def mem_map(self):
        return {
            "clint"    : 0x0030_0000,
            "csr"      : 0x5800_0000,
            "rom"      : 0x7000_0000,
            "sram"     : 0x7100_0000,
            "main_ram" : 0x8000_0000,
        }

    # GCC Flags.
    @property
    def gcc_flags(self):
        flags =  "-mno-save-restore "
        flags += GCC_FLAGS[self.variant]
        flags += "-D__blackparrot__ "
        flags += "-mcmodel=medany"
        return flags

    def __init__(self, platform, variant="standard"):
        self.platform     = platform
        self.variant      = variant
        self.reset        = Signal()
        self.ibus         = ibus = wishbone.Interface(data_width=64, adr_width=37, bursting=True)
        self.dbus         = dbus = wishbone.Interface(data_width=64, adr_width=37, bursting=True)
        self.periph_buses = [ibus, dbus]
        self.memory_buses = []

        self.cpu_params = dict(
            # Clk / Rst
            i_clk_i   = ClockSignal("sys"),
            i_reset_i = ResetSignal("sys") | self.reset,

            # Wishbone ibus
            o_m00_adr_o = ibus.adr,
            o_m00_dat_o = ibus.dat_w,
            o_m00_cyc_o = ibus.cyc,
            o_m00_stb_o = ibus.stb,
            o_m00_sel_o = ibus.sel,
            o_m00_we_o  = ibus.we,
            o_m00_cti_o = ibus.cti,
            o_m00_bte_o = ibus.bte,
            i_m00_ack_i = ibus.ack,
            i_m00_err_i = ibus.err,
            i_m00_dat_i = ibus.dat_r,

            # Wishbone dbus
            o_m01_adr_o = dbus.adr,
            o_m01_dat_o = dbus.dat_w,
            o_m01_cyc_o = dbus.cyc,
            o_m01_stb_o = dbus.stb,
            o_m01_sel_o = dbus.sel,
            o_m01_we_o  = dbus.we,
            o_m01_cti_o = dbus.cti,
            o_m01_bte_o = dbus.bte,
            i_m01_ack_i = dbus.ack,
            i_m01_err_i = dbus.err,
            i_m01_dat_i = dbus.dat_r,
        )

        # Copy config loader to /tmp
        vdir = get_data_mod("cpu", "blackparrot").data_location
        blackparrot = os.path.join(vdir, "black-parrot")
        bp_litex = os.path.join(vdir, "bp_litex")
        copyfile(os.path.join(bp_litex, "cce_ucode.mem"), "/tmp/cce_ucode.mem")

        # Set environmental variables
        os.environ["BP"] = blackparrot
        os.environ["BP_LITEX_DIR"] = bp_litex

        os.environ["BP_COMMON_DIR"] = os.path.join(blackparrot, "bp_common")
        os.environ["BP_FE_DIR"] = os.path.join(blackparrot, "bp_fe")
        os.environ["BP_BE_DIR"] = os.path.join(blackparrot, "bp_be")
        os.environ["BP_ME_DIR"] = os.path.join(blackparrot, "bp_me")
        os.environ["BP_TOP_DIR"] = os.path.join(blackparrot, "bp_top")
        external = os.path.join(blackparrot, "external")
        os.environ["BP_EXTERNAL_DIR"] = external
        os.environ["BASEJUMP_STL_DIR"] = os.path.join(external, "basejump_stl")
        os.environ["HARDFLOAT_DIR"] = os.path.join(external, "HardFloat")
        os.environ["LITEX_FPGA_DIR"] = os.path.join(bp_litex, "fpga")
        os.environ["LITEX_SIMU_DIR"] = os.path.join(bp_litex, "simulation")

        self.add_sources(platform, variant)


    def set_reset_address(self, reset_address):
        assert not hasattr(self, "reset_address")
        self.reset_address = reset_address
        assert reset_address == 0x7000_0000, "cpu_reset_addr hardcoded to 0x7000_0000!"

    @staticmethod
    def add_sources(platform, variant="standard"):
        vdir = get_data_mod("cpu", "blackparrot").data_location
        bp_litex = os.path.join(vdir, "bp_litex")
        filename = os.path.join(bp_litex, {
            "standard": "flist.fpga",
            "sim"     : "flist.verilator"
        }[variant])
        with open(filename) as openfileobject:
            for line in openfileobject:
                temp = line
                if (temp[0] == '/' and temp[1] == '/'):
                    continue
                elif ("+incdir+" in temp) :
                    s1 = line.find('$')
                    vdir = os.path.expandvars(line[s1:]).strip()
                    platform.add_verilog_include_path(vdir)
                elif (temp[0]=='$') :
                    vdir = os.path.expandvars(line).strip()
                    platform.add_source(vdir, "systemverilog")
                elif (temp[0] == '/'):
                    assert("No support for absolute path for now")

    def add_soc_components(self, soc):
        self.clintbus = clintbus = wishbone.Interface(data_width=64, adr_width=37, bursting=False)
        self.cpu_params.update(
            i_c00_adr_i = clintbus.adr,
            i_c00_dat_i = clintbus.dat_w,
            i_c00_cyc_i = clintbus.cyc,
            i_c00_stb_i = clintbus.stb,
            i_c00_sel_i = clintbus.sel,
            i_c00_we_i  = clintbus.we,
            i_c00_cti_i = clintbus.cti,
            i_c00_bte_i = clintbus.bte,
            o_c00_ack_o = clintbus.ack,
            o_c00_err_o = clintbus.err,
            o_c00_dat_o = clintbus.dat_r,
        )
        soc.bus.add_slave("clint", clintbus, region=SoCRegion(origin=soc.mem_map.get("clint"), size=0x1_0000, cached=False))

    def do_finalize(self):
        assert hasattr(self, "reset_address")
        self.specials += Instance("ExampleBlackParrotSystem", **self.cpu_params)
