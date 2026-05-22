# SI5351.py
#
# This file is oriented towards flying a pico balloon with the Traquto Jet Pack
# See the Traquito web site for details about it.  It is enough code to send WSPR
# messages.  I had started with another file but it really needed a lot of
# changes so I just started over for the most part. 2026 PLT
#
# After a lot of testing and fiddling around I finally have figured out that the
# setting the invert frequency on CLK1 and then setting its frequency and then
# performing a "Soft Reset of the PLL" PLLA in our case, and then enabling the
# clock outputs is required to get the inverted output to stay locked in place
# in relation to the non-inverted clock's phase.  Wierd.  Later after a bunch of
# net surfing I found a paper by Hans Summers, GOUPL who basically says its even
# worse when trying to chanage frequencies whith a phase change.  Maybe just
# inverting the phase isn't as problematic.  His paper is here:
# https://qrp-labs.com/images/news/dayton2018/fdim2018.pdf  He also mentions that
# there are a number of errors in the datasheet and the AN619 application note so
# it is worth a read and I'm really glad he wrote it down as no-one else that I
# could find has and this took many days of testing to sort out my solution.
#
# Another article about how the SI5351 controls phase.  Not used here but may
# be useful later.
#
# BSD 3-Clause License
# 
# Copyright (c) 2026, Paul Taylor
# Copyright (c) 2025, Craig Ivey
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
# 
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
# 
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# 
from machine import I2C, Pin
import math
#
# The I2C address is hard coded in the chip but apparently there are some that
# have other addresses?
#
SI5351_I2C_ADDRESS = 0x60
#
# Registers
#
SI5351_REGISTER_0_DEVICE_STATUS                       = 0
SI5351_REGISTER_1_INTERRUPT_STATUS_STICKY             = 1
SI5351_REGISTER_2_INTERRUPT_STATUS_MASK               = 2
SI5351_REGISTER_3_OUTPUT_ENABLE_CONTROL               = 3
SI5351_REGISTER_9_OEB_PIN_ENABLE_CONTROL              = 9
SI5351_REGISTER_15_PLL_INPUT_SOURCE                   = 15

SI5351_REGISTER_16_CLK0_CONTROL                       = 16
SI5351_REGISTER_17_CLK1_CONTROL                       = 17
SI5351_REGISTER_18_CLK2_CONTROL                       = 18
SI5351_REGISTER_19_CLK3_CONTROL                       = 19
SI5351_REGISTER_20_CLK4_CONTROL                       = 20
SI5351_REGISTER_21_CLK5_CONTROL                       = 21
SI5351_REGISTER_22_CLK6_CONTROL                       = 22
SI5351_REGISTER_23_CLK7_CONTROL                       = 23

SI5351_REGISTER_24_CLK3_0_DISABLE_STATE               = 24
SI5351_REGISTER_25_CLK7_4_DISABLE_STATE               = 25

SI5351_REGISTER_26_MSNA_P3_15_8                       = 26
SI5351_REGISTER_27_MSNA_P3_7_0                        = 27
SI5351_REGISTER_28_MSNA_P1_17_16                      = 28
SI5351_REGISTER_29_MSNA_P1_15_8                       = 29
SI5351_REGISTER_30_MSNA_P1_7_0                        = 30
SI5351_REGISTER_31_MSNA_P3nP2_19_16                   = 31
SI5351_REGISTER_32_MSNA_P2_15_8                       = 32
SI5351_REGISTER_33_MSNA_P2_7_0                        = 33

SI5351_REGISTER_34_MSNB_P3_15_8                       = 34
SI5351_REGISTER_35_MSNB_P3_7_0                        = 35
SI5351_REGISTER_36_MSNB_P1_17_16                      = 36
SI5351_REGISTER_37_MSNB_P1_15_8                       = 37
SI5351_REGISTER_38_MSNB_P1_7_0                        = 38
SI5351_REGISTER_39_MSNB_P3nP2_19_16                   = 39
SI5351_REGISTER_40_MSNB_P2_15_8                       = 40
SI5351_REGISTER_41_MSNB_P2_7_0                        = 41

SI5351_REGISTER_42_MULTISYNTH0_PARAMETERS_1           = 42
SI5351_REGISTER_43_MULTISYNTH0_PARAMETERS_2           = 43
SI5351_REGISTER_44_MULTISYNTH0_PARAMETERS_3           = 44
SI5351_REGISTER_45_MULTISYNTH0_PARAMETERS_4           = 45
SI5351_REGISTER_46_MULTISYNTH0_PARAMETERS_5           = 46
SI5351_REGISTER_47_MULTISYNTH0_PARAMETERS_6           = 47
SI5351_REGISTER_48_MULTISYNTH0_PARAMETERS_7           = 48
SI5351_REGISTER_49_MULTISYNTH0_PARAMETERS_8           = 49

SI5351_REGISTER_50_MULTISYNTH1_PARAMETERS_1           = 50
SI5351_REGISTER_51_MULTISYNTH1_PARAMETERS_2           = 51
SI5351_REGISTER_52_MULTISYNTH1_PARAMETERS_3           = 52
SI5351_REGISTER_53_MULTISYNTH1_PARAMETERS_4           = 53
SI5351_REGISTER_54_MULTISYNTH1_PARAMETERS_5           = 54
SI5351_REGISTER_55_MULTISYNTH1_PARAMETERS_6           = 55
SI5351_REGISTER_56_MULTISYNTH1_PARAMETERS_7           = 56
SI5351_REGISTER_57_MULTISYNTH1_PARAMETERS_8           = 57

SI5351_REGISTER_58_MULTISYNTH2_PARAMETERS_1           = 58
SI5351_REGISTER_59_MULTISYNTH2_PARAMETERS_2           = 59
SI5351_REGISTER_60_MULTISYNTH2_PARAMETERS_3           = 60
SI5351_REGISTER_61_MULTISYNTH2_PARAMETERS_4           = 61
SI5351_REGISTER_62_MULTISYNTH2_PARAMETERS_5           = 62
SI5351_REGISTER_63_MULTISYNTH2_PARAMETERS_6           = 63
SI5351_REGISTER_64_MULTISYNTH2_PARAMETERS_7           = 64
SI5351_REGISTER_65_MULTISYNTH2_PARAMETERS_8           = 65

SI5351_REGISTER_66_MULTISYNTH3_PARAMETERS_1           = 66
SI5351_REGISTER_67_MULTISYNTH3_PARAMETERS_2           = 67
SI5351_REGISTER_68_MULTISYNTH3_PARAMETERS_3           = 68
SI5351_REGISTER_69_MULTISYNTH3_PARAMETERS_4           = 69
SI5351_REGISTER_70_MULTISYNTH3_PARAMETERS_5           = 70
SI5351_REGISTER_71_MULTISYNTH3_PARAMETERS_6           = 71
SI5351_REGISTER_72_MULTISYNTH3_PARAMETERS_7           = 72
SI5351_REGISTER_73_MULTISYNTH3_PARAMETERS_8           = 73

SI5351_REGISTER_74_MULTISYNTH4_PARAMETERS_1           = 74
SI5351_REGISTER_75_MULTISYNTH4_PARAMETERS_2           = 75
SI5351_REGISTER_76_MULTISYNTH4_PARAMETERS_3           = 76
SI5351_REGISTER_77_MULTISYNTH4_PARAMETERS_4           = 77
SI5351_REGISTER_78_MULTISYNTH4_PARAMETERS_5           = 78
SI5351_REGISTER_79_MULTISYNTH4_PARAMETERS_6           = 79
SI5351_REGISTER_80_MULTISYNTH4_PARAMETERS_7           = 80
SI5351_REGISTER_81_MULTISYNTH4_PARAMETERS_8           = 81

SI5351_REGISTER_82_MULTISYNTH5_PARAMETERS_1           = 82
SI5351_REGISTER_83_MULTISYNTH5_PARAMETERS_2           = 83
SI5351_REGISTER_84_MULTISYNTH5_PARAMETERS_3           = 84
SI5351_REGISTER_85_MULTISYNTH5_PARAMETERS_4           = 85
SI5351_REGISTER_86_MULTISYNTH5_PARAMETERS_5           = 86
SI5351_REGISTER_87_MULTISYNTH5_PARAMETERS_6           = 87
SI5351_REGISTER_88_MULTISYNTH5_PARAMETERS_7           = 88
SI5351_REGISTER_89_MULTISYNTH5_PARAMETERS_8           = 89

SI5351_REGISTER_90_MULTISYNTH6_PARAMETERS             = 90
SI5351_REGISTER_91_MULTISYNTH7_PARAMETERS             = 91
SI5351_REGISTER_092_CLOCK_6_7_OUTPUT_DIVIDER          = 92
SI5351_REGISTER_165_CLK0_INITIAL_PHASE_OFFSET         = 165
SI5351_REGISTER_166_CLK1_INITIAL_PHASE_OFFSET         = 166
SI5351_REGISTER_167_CLK2_INITIAL_PHASE_OFFSET         = 167
SI5351_REGISTER_168_CLK3_INITIAL_PHASE_OFFSET         = 168
SI5351_REGISTER_169_CLK4_INITIAL_PHASE_OFFSET         = 169
SI5351_REGISTER_170_CLK5_INITIAL_PHASE_OFFSET         = 170
SI5351_REGISTER_177_PLL_RESET                         = 177
SI5351_REGISTER_183_CRYSTAL_INTERNAL_LOAD_CAPACITANCE   = 183
SI5351_REGISTER_187_FANOUT                            = 187

SI5351_CRYSTAL_FREQ_25MHZ = 25000000
SI5351_CRYSTAL_FREQ_26MHZ = 26000000
SI5351_CRYSTAL_FREQ_27MHZ = 27000000
#
# From AN619.  Bits 0 to 5 should be written to 0b01 0010
#
SI5351_CRYSTAL_LOAD_6PF  = 0b01010010               
SI5351_CRYSTAL_LOAD_8PF  = 0b10010010               
SI5351_CRYSTAL_LOAD_10PF = 0b11010010        # = 0xD2    

SI5351_MULTISYNTH_DIV_4  = 4
SI5351_MULTISYNTH_DIV_6  = 6
SI5351_MULTISYNTH_DIV_8  = 8
#
# address = I2C address
# register = I2C register
# value = 8 bit value to write to the register
#
def write8(I2C, address, register, value):
    DataBuffer = bytearray(1)
    DataBuffer[0] = value & 0xff
    I2C.writeto_mem(address, register, DataBuffer)
    return

def read8(I2C, address, register):
    DataBuffer = bytearray(1)
    I2C.readfrom_mem_into(address, register, DataBuffer)
    return DataBuffer

def ReadInitStatus(I2C):
    status = bytearray(1)
    status = read8(I2C, SI5351_I2C_ADDRESS, SI5351_REGISTER_0_DEVICE_STATUS)
    status[0] &= 0b10000000       # Mask out the bits we don't care about.
    return status[0]
#
# Scan all devices from 0x08 to 0x77 for our address.
# 0 returned means it was found
#
def CheckI2CforSI5351(I2C):
    Error = 0
    
    DeviceList = I2C.scan()

    for Item in DeviceList:
        if Item == SI5351_I2C_ADDRESS:
            Error = 0
            break                # Found it, return.
        else:
            Error = 1
            
    return Error
#
# Configure the clocks per the Skyworks SI5351 datasheet flow chart.
# This configures the PLLs and the clocks as if we were going to use the 10m
# band but that can be changed later with other functions.
#
def InitializeClocks(I2C):
    i = 0
    Status = 0xFF
    #
    # Check to make sure the SI5351 initialized.  ToDo: just check bit 7
    #
    while Status != 0:
        Status = ReadInitStatus(I2C)    
        #print(f'Clock Initialization Status: {Status}')
    #
    #  Set the load capacitance for the XTAL (bits 0 to 5 have a specific pattern see contant)
    #
    write8(I2C, SI5351_I2C_ADDRESS, SI5351_REGISTER_183_CRYSTAL_INTERNAL_LOAD_CAPACITANCE, SI5351_CRYSTAL_LOAD_10PF)
    #
    # Disable all clock outputs
    #
    write8(I2C, SI5351_I2C_ADDRESS, SI5351_REGISTER_3_OUTPUT_ENABLE_CONTROL, 0xFF)
    #
    # Power down all output drivers
    #
    write8(I2C, SI5351_I2C_ADDRESS, SI5351_REGISTER_16_CLK0_CONTROL, 0x80)
    write8(I2C, SI5351_I2C_ADDRESS, SI5351_REGISTER_17_CLK1_CONTROL, 0x80)
    write8(I2C, SI5351_I2C_ADDRESS, SI5351_REGISTER_18_CLK2_CONTROL, 0x80)
    write8(I2C, SI5351_I2C_ADDRESS, SI5351_REGISTER_19_CLK3_CONTROL, 0x80)
    write8(I2C, SI5351_I2C_ADDRESS, SI5351_REGISTER_20_CLK4_CONTROL, 0x80)
    write8(I2C, SI5351_I2C_ADDRESS, SI5351_REGISTER_21_CLK5_CONTROL, 0x80)
    write8(I2C, SI5351_I2C_ADDRESS, SI5351_REGISTER_22_CLK6_CONTROL, 0x80)
    write8(I2C, SI5351_I2C_ADDRESS, SI5351_REGISTER_23_CLK7_CONTROL, 0x80)
    #
    # Now we should set SI5351_REGISTER_2_INTERRUPT_STATUS_MASK according to the flow chart but
    # it only applies to the SI5351C which we don't have.
    #
    # Register 15: PLL Input Source: CLKIN divider 1, PLLB_SRC XTAL input, PLLA_SRC XTAL input
    write8(I2C, SI5351_I2C_ADDRESS, SI5351_REGISTER_15_PLL_INPUT_SOURCE, 0x00)
    # Register 16: Clock 0 Control: powered up, MS0 operates in fractional mode, PLLA is the source for MS0, Clock is NOT inverted, MS0 is the source for CLK0, 8mA drive strength
    write8(I2C, SI5351_I2C_ADDRESS, SI5351_REGISTER_16_CLK0_CONTROL, 0x0F)
    # Register 17: Clock 1 Control: powered up, MS1 operates in fractional mode, PLLA is the source for MS1, Clock IS inverted, MS0 (not typo) is the source for CLK1, 8mA drive strength
    write8(I2C, SI5351_I2C_ADDRESS, SI5351_REGISTER_17_CLK1_CONTROL, 0x1F)
    # Register 18: Clock 2 Control: powered down, MS2 operates in fractional mode, PLLA is the source for MS2, Clock is NOT inverted, MS2 is the source for CLK2, 2mA drive
    write8(I2C, SI5351_I2C_ADDRESS, SI5351_REGISTER_18_CLK2_CONTROL, 0x8C)
    # Register 19-23: We don't have these clocks
    write8(I2C, SI5351_I2C_ADDRESS, SI5351_REGISTER_19_CLK3_CONTROL, 0x8C)
    write8(I2C, SI5351_I2C_ADDRESS, SI5351_REGISTER_20_CLK4_CONTROL, 0x8C)
    write8(I2C, SI5351_I2C_ADDRESS, SI5351_REGISTER_21_CLK5_CONTROL, 0x8C)
    write8(I2C, SI5351_I2C_ADDRESS, SI5351_REGISTER_22_CLK6_CONTROL, 0x4F)    # 0b01001111 = 0x4F Clk6 & 7 are integer only mode.
    write8(I2C, SI5351_I2C_ADDRESS, SI5351_REGISTER_23_CLK7_CONTROL, 0x4F)
    # Register 24 - 25: All Clock's disabled state = low state when disabled (not used by CBPro)
    write8(I2C, SI5351_I2C_ADDRESS, SI5351_REGISTER_24_CLK3_0_DISABLE_STATE, 0x00)
    write8(I2C, SI5351_I2C_ADDRESS, SI5351_REGISTER_25_CLK7_4_DISABLE_STATE, 0x00)
    #
    # We are using a multiplier of 30 numerator 0 and denominater 1.  The PLL should be 26 000 000 Hz * (30 + 0/1)
    # This gives us a Frequency of: 780 000 000 Hz and we set it as follows:
    # Register 26 MSNA_P3[15:8]   denom 0x00  26-33 Set PLLA which doesn't change even when changing bands.  P1: 3328 P2: 0 P3: 1
    write8(I2C, SI5351_I2C_ADDRESS, SI5351_REGISTER_26_MSNA_P3_15_8, 0x00)
    # Register 27 MSNA_P3[7:0]    denom 0x01
    write8(I2C, SI5351_I2C_ADDRESS, SI5351_REGISTER_27_MSNA_P3_7_0, 0x01)
    # Register 28 MSNA_P1[17:16]  int 0x00 R28,29,30 = 3328
    write8(I2C, SI5351_I2C_ADDRESS, SI5351_REGISTER_28_MSNA_P1_17_16, 0x00)
    # Register 29 MSNA_P1[15:8]   int 0x0D
    write8(I2C, SI5351_I2C_ADDRESS, SI5351_REGISTER_29_MSNA_P1_15_8, 0x0D)
    # Register 30 MSNA_P1[7:0]    int 0x00
    write8(I2C, SI5351_I2C_ADDRESS, SI5351_REGISTER_30_MSNA_P1_7_0, 0x00)
    # Register 31 MSNA_P3[19:16]  denom 0000 & MSNA_P2[19:16] num 0000  0x00
    write8(I2C, SI5351_I2C_ADDRESS, SI5351_REGISTER_31_MSNA_P3nP2_19_16, 0x00)
    # Register 32 MSNA_P2[15:18]  numer 0x00
    write8(I2C, SI5351_I2C_ADDRESS, SI5351_REGISTER_32_MSNA_P2_15_8, 0x00)
    # Register 33 MSNA_P2[7:0]    numer 0x00
    write8(I2C, SI5351_I2C_ADDRESS, SI5351_REGISTER_33_MSNA_P2_7_0, 0x00)
    # Register 34 - 41: Multisynth NB params (just set the same as NA, we don't use it.)
    write8(I2C, SI5351_I2C_ADDRESS, SI5351_REGISTER_34_MSNB_P3_15_8, 0x00)
    write8(I2C, SI5351_I2C_ADDRESS, SI5351_REGISTER_35_MSNB_P3_7_0, 0x01)
    write8(I2C, SI5351_I2C_ADDRESS, SI5351_REGISTER_36_MSNB_P1_17_16, 0x00)
    write8(I2C, SI5351_I2C_ADDRESS, SI5351_REGISTER_37_MSNB_P1_15_8, 0x0D)
    write8(I2C, SI5351_I2C_ADDRESS, SI5351_REGISTER_38_MSNB_P1_7_0, 0x00)
    write8(I2C, SI5351_I2C_ADDRESS, SI5351_REGISTER_39_MSNB_P3nP2_19_16, 0x00)
    write8(I2C, SI5351_I2C_ADDRESS, SI5351_REGISTER_40_MSNB_P2_15_8, 0x00)
    write8(I2C, SI5351_I2C_ADDRESS, SI5351_REGISTER_41_MSNB_P2_7_0, 0x00)
    # Register 42 MS0_P 3[15:8] 0x08  Multisynth0 parameters.  I just set for 10m symbol 3. P1: 3037 P2: 1559 P3: 2173 These will change with the different bands and lanes.
    write8(I2C, SI5351_I2C_ADDRESS, SI5351_REGISTER_42_MULTISYNTH0_PARAMETERS_1, 0x08)
    # Register 43 MS0_P3[7:0]  0x7D
    write8(I2C, SI5351_I2C_ADDRESS, SI5351_REGISTER_43_MULTISYNTH0_PARAMETERS_2, 0x7D)
    # Register 44 R0_DIV[2:0]   MS0_DIVBY4[1:0]   MS0_P1[17:16] 0x00
    write8(I2C, SI5351_I2C_ADDRESS, SI5351_REGISTER_44_MULTISYNTH0_PARAMETERS_3, 0x00)
    # Register 45 MS0_P1[15:8] 0x0B
    write8(I2C, SI5351_I2C_ADDRESS, SI5351_REGISTER_45_MULTISYNTH0_PARAMETERS_4, 0x0B)
    # Register 46 MS0_P1[7:0]  0xDD
    write8(I2C, SI5351_I2C_ADDRESS, SI5351_REGISTER_46_MULTISYNTH0_PARAMETERS_5, 0xDD)
    # Register 47 MS0_P3[19:16]  MS0_P2[19:16] 0x00
    write8(I2C, SI5351_I2C_ADDRESS, SI5351_REGISTER_47_MULTISYNTH0_PARAMETERS_6, 0x00)
    # Register 48 MS0_P2[15:8] 0x06
    write8(I2C, SI5351_I2C_ADDRESS, SI5351_REGISTER_48_MULTISYNTH0_PARAMETERS_7, 0x06)
    # Register 49 MS0_P2[7:0]  0x17
    write8(I2C, SI5351_I2C_ADDRESS, SI5351_REGISTER_49_MULTISYNTH0_PARAMETERS_8, 0x17)
    # Register 50 - 57: Multisynth 1 params  Set these the same as 42 to 49
    write8(I2C, SI5351_I2C_ADDRESS, SI5351_REGISTER_50_MULTISYNTH1_PARAMETERS_1, 0x00)
    write8(I2C, SI5351_I2C_ADDRESS, SI5351_REGISTER_51_MULTISYNTH1_PARAMETERS_2, 0x00)
    write8(I2C, SI5351_I2C_ADDRESS, SI5351_REGISTER_52_MULTISYNTH1_PARAMETERS_3, 0x00)
    write8(I2C, SI5351_I2C_ADDRESS, SI5351_REGISTER_53_MULTISYNTH1_PARAMETERS_4, 0x00)
    write8(I2C, SI5351_I2C_ADDRESS, SI5351_REGISTER_54_MULTISYNTH1_PARAMETERS_5, 0x00)
    write8(I2C, SI5351_I2C_ADDRESS, SI5351_REGISTER_55_MULTISYNTH1_PARAMETERS_6, 0x00)
    write8(I2C, SI5351_I2C_ADDRESS, SI5351_REGISTER_56_MULTISYNTH1_PARAMETERS_7, 0x00)
    write8(I2C, SI5351_I2C_ADDRESS, SI5351_REGISTER_57_MULTISYNTH1_PARAMETERS_8, 0x00)
    # Register 58 - 91: Multisynth 2 - 7 params  All 0x00
    for i in range(58,92):
        write8(I2C, SI5351_I2C_ADDRESS, i, 0x00)
    # Register 92: Clock 6 and 7 output divider
    write8(I2C, SI5351_I2C_ADDRESS, SI5351_REGISTER_092_CLOCK_6_7_OUTPUT_DIVIDER, 0x00)
    # Registers 149 to 170           
    for i in range(149, 171):
        write8(I2C, SI5351_I2C_ADDRESS, i, 0x00)
    #
    # Fanout is needed to allow CLK1 to use MS0.
    #
    Reg187 = 0x00
    Reg187 = read8(I2C, SI5351_I2C_ADDRESS, SI5351_REGISTER_187_FANOUT)
    Reg187[0] |= 0b00010000           # Set Bit 4 and don't bother the reserved bits
    write8(I2C, SI5351_I2C_ADDRESS, SI5351_REGISTER_187_FANOUT, Reg187[0])
    #
    # Power up the output drivers for clock 0 and 1.
    #
    Reg16 = 0x00
    Reg16 = read8(I2C, SI5351_I2C_ADDRESS, SI5351_REGISTER_16_CLK0_CONTROL)
    Reg16[0] &= 0b01111111           # Set Bit 7 to 0 power up clock 0.
    write8(I2C, SI5351_I2C_ADDRESS, SI5351_REGISTER_16_CLK0_CONTROL, Reg16[0])    
    
    Reg17 = 0x00
    Reg17 = read8(I2C, SI5351_I2C_ADDRESS, SI5351_REGISTER_17_CLK1_CONTROL)
    Reg17[0] &= 0b01111111           # Set Bit 7 to 0 power up clock 1.
    write8(I2C, SI5351_I2C_ADDRESS, SI5351_REGISTER_17_CLK1_CONTROL, Reg17[0])  
    #
    # Now apply PLLA and PLLB soft reset.
    # The register has reserved bits so read and then write.
    #
    Reg177 = 0x00
    Reg177 = read8(I2C, SI5351_I2C_ADDRESS, SI5351_REGISTER_177_PLL_RESET)
    Reg177[0] |= 0b10100000           # Reset PLL A and B.
    write8(I2C, SI5351_I2C_ADDRESS, SI5351_REGISTER_177_PLL_RESET, Reg177[0])
    #
    # Enabled desired outputs using register 3.  We only use CLK0 and CLK1. 0b0000 0011 or 0x03
    #  
    write8(I2C, SI5351_I2C_ADDRESS, SI5351_REGISTER_3_OUTPUT_ENABLE_CONTROL, 0x03)
    
    return
#
# AN619 says the PLL frequency should be between 600 and 900 MHz
# We have already configured the PLLs in the InitializeClocks() Function
#
def SetupPLL(I2C, mult, num, denom, pllsource = 'A'):
    Error = 0
    
    if ((mult < 15) or (mult > 90)):
        print('FATAL Error: mulitplier (a) out of range (15-90): ', mult)
        Error = 1
    elif (denom <= 0) or (denom > 0xfffff):
        print('FATAL Error: denominator (c) out of range (<=0,>1048575): ', denom)
        Error = 2
    elif (num < 0) or (num > 0xfffff): 
        print('FATAL Error: numerator (b) out of range (<0,>1048575): ', num)
        Error = 3
    else:
        # Calculate P1, P2, and P3 from equations in AN619
        # Integer config just leaves out the math as if num was 0
        #
        if num ==0:
            P1 = 128 * mult - 512
            P2 = num
            P3 = denom
        # Fractional Config
        else:
            P1 = 128*mult + math.floor( 128 * num/denom ) -512
            P2 = 128*num - denom * math.floor( 128 * num/denom)
            P3 = denom 
        #print(f'PLL: P1: {P1} P2: {P2} P3: {P3}')

        if pllsource == 'A':
            baseaddr = SI5351_REGISTER_26_MSNA_P3_15_8    # Register 26 is beginning of Multisynth NA Parameters
        else:
            baseaddr = SI5351_REGISTER_34_MSNB_P3_15_8    # Register 34 is beginning of Multisynth NB Parameters

        write8(I2C, SI5351_I2C_ADDRESS, baseaddr,   (P3 & 0x0000FF00) >> 8)          #26 or 34
        write8(I2C, SI5351_I2C_ADDRESS, baseaddr+1, (P3 & 0x000000FF))               #27 or 35
        write8(I2C, SI5351_I2C_ADDRESS, baseaddr+2, (P1 & 0x00030000) >> 16)         #28 or 36
        write8(I2C, SI5351_I2C_ADDRESS, baseaddr+3, (P1 & 0x0000FF00) >> 8)          #29 or 37
        write8(I2C, SI5351_I2C_ADDRESS, baseaddr+4, (P1 & 0x000000FF))               #30 or 38
        write8(I2C, SI5351_I2C_ADDRESS, baseaddr+5, ((P3 & 0x000F0000) >> 12) | ((P2 & 0x000F0000) >> 16) )   #32 or 39
        write8(I2C, SI5351_I2C_ADDRESS, baseaddr+6, (P2 & 0x0000FF00) >> 8)          #33 or 40
        write8(I2C, SI5351_I2C_ADDRESS, baseaddr+7, (P2 & 0x000000FF))               #34 or 41
        #
        # Now apply PLLA or PLLB soft reset.
        #
        if pllsource == 'A':
            PLLSoftReset(I2C, 'A')
        else:
            PLLSoftReset(I2C, 'B')
    
    PLLFreq = int(math.floor(SI5351_CRYSTAL_FREQ_26MHZ * (mult + num/denom)))
    
    return PLLFreq, Error
#
# Soft reset for PLL A or B
#
def PLLSoftReset(I2C, pllsource = 'A'):
    #
    # Apply PLLA or PLLB soft reset.
    # The register has reserved bits so read and then write.
    #
    if pllsource == 'A':
        Reg177 = 0x00
        Reg177 = read8(I2C, SI5351_I2C_ADDRESS, SI5351_REGISTER_177_PLL_RESET)
        Reg177[0] |= 0b00100000           # Reset PLL A
        write8(I2C, SI5351_I2C_ADDRESS, SI5351_REGISTER_177_PLL_RESET, Reg177[0])
    else:
        Reg177 = 0x00
        Reg177 = read8(I2C, SI5351_I2C_ADDRESS, SI5351_REGISTER_177_PLL_RESET)
        Reg177[0] |= 0b10000000           # Reset PLL B
        write8(I2C, SI5351_I2C_ADDRESS, SI5351_REGISTER_177_PLL_RESET, Reg177[0])

    return
#
# SetupMultisynth() is going to be called over and over every 0.6 seconds while send
# our WSPR Message.  The equations come from the AN619 application sheet for the
# SI5351.
# Clock is 0, 1, or 2 for CLK0,1 or 2 for the 10 pin package we are using.
# div seems to be a in the equation
# num is b.  This will assume integer mode if num = 0.
# denom is c
# note that if a phase delay is going to be used this function can't be used
# to set the CLK in integer mode.  Check AN619
#
def SetupMultisynth(I2C, Clock, div, num, denom):
    Error = 0
    
    if Clock not in [0,1,2]:
        print('FATAL Error: Clock out of range (0, 1, or 2): ', Clock)
        Error = 1
    elif div < 8:
        print('FATAL Error: div < 8: ', div)
        Error = 2
    elif (denom <= 0) or (denom > 0xfffff):
        print('FATAL Error: denominator (c) out of range (<=0,>1048575): ', denom)
        Error = 3
    elif (num < 0) or (num > 0xfffff): 
        print('FATAL Error: numerator (b) out of range (<0,>1048575): ', num)
        Error = 4
    else:
        # Output Multisynth Divider Equations from AN619:
        # a = div, b = num and c = denom
        # P1 register is an 18-bit value using following formula:
        # P1[17:0] = 128 * a + floor(128*(b/c)) - 512
        # P2 register is a 20-bit value using the following formula:
        # P2[19:0] = 128 * b - c * floor(128*(b/c))
        # P3 register is a 20-bit value using the following formula:
        # P3[19:0] = c

        if num==0:
            # integer mode 
            P1 = 128 * div - 512
            P2 = num
            P3 = denom
        else:
            # Fractional mode
            P1 = int( 128 * div + math.floor(128 * (num/denom)) - 512 )
            P2 = int( 128 * num - denom * math.floor(128 * (num/denom)))
            P3 = denom

        #print(f'MS[{output}]: P1: {P1} P2: {P2} P3: {P3}')

        baseaddrs = [42, 50, 58]        # Starting registers for the 3 clocks
        baseaddr = baseaddrs[Clock]     # Select the register set based on the clock passed in

        write8(I2C, SI5351_I2C_ADDRESS, baseaddr,  (P3 & 0x0000FF00) >> 8)          # 42|50|58 MSx_P 3[15:8]
        write8(I2C, SI5351_I2C_ADDRESS, baseaddr + 1, (P3 & 0x000000FF))            # 43|51|59 MSx_DIVBY4[1:0]   MSx_P1[17:16]
        write8(I2C, SI5351_I2C_ADDRESS, baseaddr + 2, (P1 & 0x00030000) >> 16)      # 44|52|60 MSx_P1[15:8]
        write8(I2C, SI5351_I2C_ADDRESS, baseaddr + 3, (P1 & 0x0000FF00) >> 8)       # 45|53|61 MSx_P1[15:8]
        write8(I2C, SI5351_I2C_ADDRESS, baseaddr + 4, (P1 & 0x000000FF))            # 46|54|62 MSx_P1[7:0]
        write8(I2C, SI5351_I2C_ADDRESS, baseaddr + 5, ((P3 & 0x000F0000) >> 12) | ((P2 & 0x000F0000) >> 16) )  # 47|55|63 MSx_P3[19:16]  MSx_P2[19:16]
        write8(I2C, SI5351_I2C_ADDRESS, baseaddr + 6, (P2 & 0x0000FF00) >> 8)       # 48|56|64 MSx_P2[15:8]
        write8(I2C, SI5351_I2C_ADDRESS, baseaddr + 7, (P2 & 0x000000FF))            # 49|57|65 MSx_P2[7:0]
    
    return Error
#
# Invert Clock Output
# Clock is the clock to invert 0, 1, 2.
# Invert is 0 to not invert or 1 to invert
#
def ClockSetInvertOutput(I2C, Clock, Invert):
    Register = 0x00
    #
    # Configure the clk control register Bit 4 is our interest here.
    # Bit 7: Clock power down 1=powered down 0=powered up
    # Bit 6: Multisynth Integer mode 1=int, 0=fractional
    # Bit 5: Source Select for ClockX 0=PLLA 1=PLLB (if allowed)
    # bit 4: Output clock invert 1=inverted
    # Bit 3-2: Output Clock X input source 00=XTAL, 01=CLKIN, 10=do not select, 11 Multisynth X
    # Bit 1-0: Drive strength: 00=2mA, 01=4mA, 10=6mA, 11=8mA
    #
    if Clock == 0:
        Register = read8(I2C, SI5351_I2C_ADDRESS, SI5351_REGISTER_16_CLK0_CONTROL)
        if Invert == 0:
            Register[0] &= 0b11101111           # Set bit 4 to 0
        else:
            Register[0] |= 0b00010000           # Set bit 4 to 1
        write8(I2C, SI5351_I2C_ADDRESS, SI5351_REGISTER_16_CLK0_CONTROL, Register[0])
        
    if Clock == 1:
        Register = read8(I2C, SI5351_I2C_ADDRESS, SI5351_REGISTER_17_CLK1_CONTROL)
        if Invert == 0:
            Register[0] &= 0b11101111           # Set bit 4 to 0
        else:
            Register[0] |= 0b00010000           # Set bit 4 to 1
        write8(I2C, SI5351_I2C_ADDRESS, SI5351_REGISTER_17_CLK1_CONTROL, Register[0])
        
    if Clock == 2:
        Register = read8(I2C, SI5351_I2C_ADDRESS, SI5351_REGISTER_18_CLK2_CONTROL)
        if Invert == 0:
            Register[0] &= 0b11101111           # Set bit 4 to 0
        else:
            Register[0] |= 0b00010000           # Set bit 4 to 1
        write8(I2C, SI5351_I2C_ADDRESS, SI5351_REGISTER_18_CLK2_CONTROL, Register[0])
        
    return
#
# Enable = 1 to enable clocks 0 and 1 and 0 to disable both clocks.
#
def EnableClocks0and1(I2C, Enable):
    
    if Enable == 0:    # Disable all clocks.
        write8(I2C, SI5351_I2C_ADDRESS, SI5351_REGISTER_3_OUTPUT_ENABLE_CONTROL, 0xFF)
    else:              # Enable Clock 0 and 1, disable the rest.
        write8(I2C, SI5351_I2C_ADDRESS, SI5351_REGISTER_3_OUTPUT_ENABLE_CONTROL, 0xFC)

    return

#1234567890123456789012345678901234567890123456789012345678901234567890123456789
# Test main
"""
import time

I2C = I2C(0, scl=Pin(5), sda=Pin(4), freq=100000)

CheckI2CforSI5351(I2C)

InitializeClocks(I2C)
#
# The PLL is setup in the init function but could be used to change it so here it is
print('PLL Freq, Error: ', SetupPLL(I2C, 30, 0, 1, 'A'))
#
# The following is required to get the inverted output on CLK1 to work and stay working
# once this is done then it seems to hold even though we're changing frequencies.  All
# the steps are needed or the inverted phase on CLK1 drifts comparted to CLK0
#
ClockSetInvertOutput(I2C, 1, 1)
#
# 16 + 21552 / 1837  F = 780000000 / (16 + 21552 / 1837)
SetupMultisynth(I2C, 0, 16, 21552, 1837)
SetupMultisynth(I2C, 1, 16, 21552, 1837)
print(f'Preliminary Frequency 0 Setting: 28 126 177.8')

PLLSoftReset(I2C, 'A')

EnableClocks0and1(I2C, 1)
# End of required for inverted output on CLK1

while True:
    # We'll just run through the four symbols for the 10m band
    # 16 + 21552 / 1837  F = 780000000 / (16 + 21552 / 1837)
    SetupMultisynth(I2C, 0, 16, 21552, 1837)
    SetupMultisynth(I2C, 1, 16, 21552, 1837)
    print(f'Frequency 0: 28 126 177.8')    # the pico can't calculate this, used a calculator.
    time.sleep(5)    
    # 16 + 30269 / 2580
    SetupMultisynth(I2C, 0, 16, 30269, 2580)
    SetupMultisynth(I2C, 1, 16, 30269, 2580)  
    print(f'Frequency 1: 28 126 179.3')
    time.sleep(5)
    # 16 + 47703 / 4066
    SetupMultisynth(I2C, 0, 16, 47703, 4066)
    SetupMultisynth(I2C, 1, 16, 47703, 4066)    
    print(f'Frequency 2: 28 126 180.6')
    time.sleep(5)
    # 16 + 25494 / 2173
    SetupMultisynth(I2C, 0, 16, 25494, 2173)
    SetupMultisynth(I2C, 1, 16, 25494, 2173)   
    print(f'Frequency 3: 28 126 182.3')
    time.sleep(5)
    EnableClocks0and1(I2C, 0)
    time.sleep(5)
"""

#1234567890123456789012345678901234567890123456789012345678901234567890123456789
# End of File: SI5351.py