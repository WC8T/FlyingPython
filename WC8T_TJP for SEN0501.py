#
# Flying Python for the Traquito version 2026-03-01 Paul Taylor, WC8T
# This version hosts the DFRobot SEN0501 v2 multi-sensor I2C board
#
# Yes, you need an amateur radio license, general class in the U.S.A., to use this software to transmit.
#
# First save the Python runtime to the pico it will be something like: RPI_PICO-date-version.uf2.  Hold the pico
# button and plug into USB then copy it over.  Then load Thonny python environment and save: SEN0501.py, 
# SI5351.py, ATGM336H.py, MorseLED.py  to the pico using Thonny.  Next save this file on the Pico as main.py.  If
# in Thonny environment press CTRL-D from the Shell window to restart the pico and run the code.
#
# References for WSPR and pico ballon telemetry:
# Document on WSPR by Joe Taylor K1JT
# Document by Andy Talbot G4JNT: http://www.g4jnt.com/Coding/WSPR_Coding_Process.pdf 
# This is the paper that mentions the convolutions and where the two numbers come from (It is 36$ so I didn't buy it):
# J. Layland and W. Lushbaugh, "A Flexible High-Speed Sequential Decoder for Deep Space Channels," in
# IEEE Transactions on Communication Technology, vol. 19, no. 5, pp. 813-820, October 1971, doi: 10.1109/TCOM.1971.1090732.
# A project called Encoding WSPRs with some helpful notes: hackaday.io/project/166875-careless-wspr/log/167301-encoding-wsprs
# the code for it is here: https://github.com/ziggurat29/CarelessWSPR.git
# The Traquito web site and all its info and functionality.  The Traquito is what got us started flying balloons.  If you haven't
# flown use the code there.  It made it easy for us to get going (you know the 10 flights you need to start being successfull).
# More references I saw in the Traquito code: JTEncode Library https://github.com/etherkit/JTEncode and
# K6HX (Mark VandeWettering) https://github.com/brainwagon/genwspr
#
# This basis for this code came from Craig Ivey: https://github.com/IveyWorks/OpenTJP.git  I did a lot of re-arranging and changed
# the way the timing is done and added a ton of coments along the way to help me figure out how it all worked.  The motivation
# was learning how the code worked and using the SEN0501 on a flight.  The code was much smaller and easier to
# follow than the Traquito code so this is how I started my first Python project.  I'm currently going through the SI5351 code
# to see how that works, comment it more, etc.
#
# The sensor code that sets the values to be sent out via WSPR goes in this function: WSPR_ExtTel1EncodeMeasurements() and
# WSPR_ExtTel2EncodeMeasurements()
# Then (in the SEN0501 case) the data is transmitted in the third time slot - slot2.
#
#1234567890123456789012345678901234567890123456789012345678901234567890123456789
#
# BSD 3-Clause License
# 
# Copyright (c) 2025, Craig Ivey,
# Copyright (c) 2026, Paul Taylor, WC8T,
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

import ATGM336H
import SI5351
import SEN0501
import MorseLED

import machine
import math
from machine import ADC, Pin, I2C, UART, WDT
import time

# These are set by the pico ballon pilot
MY_CHANNEL = 155      #  0-599  See the traquito web site to pick a channel.  #KI7KDB - 298 20m, W0MNE - 134, KF7CEP - 157 WC8T - 155,245 on 10m
MY_CALLSIGN = 'KI7KDB'  
MY_BAND = '10m'       # ['10m', '12m', '15m', '17m', '20m', '30m', '40m', '80m']  remember to include the 'm' character SI5351 may not do 80m

# Constants
GPS_ON = 1
GPS_OFF = 0
MY_DEBUG_PRINT_ENABLE = True		# Print debug statement to serial output
TX_ON = 1		# Turn on the TCXO (temp controlled crystal oscillator) and the SI5351
TX_OFF = 0		# Turn them off.
LED_ON = 1
LED_OFF = 0

MY_POWER = 13		  # this is in dBm.  values from 0 to 60 but only those ending in 0,3,or7 will work
MY_FREQ_OFFSET = 0    # Shift output freq by X Hertz. Valid: -20, -10, 0, +10, +20  I'm not sure why this is in the code?
                      # For some reason WSPR doesn't pick up the output if it this shift is used?

#
# Synchronization Bits.  I have not discovered the "why" this is used.
# 
SYNC = [1,1,0,0,0,0,0,0,1,0,0,0,1,1,1,0,0,0,1,0,0,1,0,1,1,1,1,0,0,0,0,0,0,0,1,0,0,1,0,1,0,0, 
    0,0,0,0,1,0,1,1,0,0,1,1,0,1,0,0,0,1,1,0,1,0,0,0,0,1,1,0,1,0,1,0,1,0,1,0,0,1,0,0,1,0, 
    1,1,0,0,0,1,1,0,1,0,1,0,0,0,1,0,0,0,0,0,1,0,0,1,0,0,1,1,1,0,1,1,0,0,1,1,0,1,0,0,0,1,
    1,1,0,0,0,0,0,1,0,1,0,0,1,1,0,0,0,0,0,0,0,1,1,0,1,0,1,1,0,0,0,1,1,0,0,0]

# This table is used in conjunction with the MY_FREQ_OFFSET and the band and lane to figure out what values to use
# to set the frequency for a particular symbol.  Each band has the values for -20, -10, 0, 10, 20 hz offset and each
# set takes 32 values.  In the 32 values are 8 values for each "lane" used by the pico ballon reporting software.
# It is generated by "find_freq_parameters.py" and it takes a long time to run.
# This table 10m is generated for 780 000 000 MHz PLL frequency. (multiplier of 30 divider of 16).
# Base Frequency + Lane1: 20Hz, Lane2: 60Hz, Lane3: 140Hz, Lane4: 180Hz
#
SIGGENPARAMS = { # 28.126000 – 28.126200 MHz
'10m': [ 29413, 2507, 7145, 609, 38048, 3243, 13457, 1147, 31337, 2671, 36640, 3123, 51059, 4352, 37297, 3179,    #0-15 -20Hz Lanes 1-4 (8 values per lane)  
         47539, 4052, 12049, 1027, 31853, 2715, 37461, 3193, 17129, 1460, 39690, 3383, 13668, 1165, 37543, 3200,  #16-31 
         18103, 1543, 44008, 3751, 48654, 4147, 19593, 1670, 33308, 2839, 12096, 1031, 59647, 5084, 15076, 1285,  #32-47 -10Hz Lanes 1-4   
         13363, 1139, 21775, 1856, 52619, 4485, 65149, 5553, 48231, 4111, 22385, 1908, 29131, 2483, 60233, 5134,  #48-63  
         17927, 1528, 35021, 2985, 31865, 2716, 26386, 2249, 26996, 2301, 17223, 1468, 23183, 1976, 35103, 2992,  #64-79  0Hz Lanes 1-4 
         24403, 2080, 59189, 5045, 38247, 3260, 32639, 2782, 21552, 1837, 30269, 2580, 47703, 4066, 25494, 2173,  #80-95
         17751, 1513, 36992, 3153, 19241, 1640, 23054, 1965, 30457, 2596, 20027, 1707, 15557, 1326, 12577, 1072,  #96-111  +10Hz Lanes 1-4
         21423, 1826, 36581, 3118, 33777, 2879, 19276, 1643, 32721, 2789, 23347, 1990, 38634, 3293, 13973, 1191,  #112-127
         31337, 2671, 36640, 3123, 51059, 4352, 37297, 3179, 21341, 1819, 44829, 3821, 11744, 1001, 32252, 2749,  #128-143  +20Hz Lanes 1-4
         17129, 1460, 39690, 3383, 13668, 1165, 37543, 3200, 22514, 1919, 26456, 2255, 31712, 2703, 40253, 3431]  #144-159
,                # 24.926000 – 24.926200 MHz
'12m': [ 34439, 2252, 66997, 4381, 25921, 1695, 31304, 2047, 28643, 1873, 105687, 6911, 44899, 2936, 20385, 1333, # updated   
         45541, 2978, 30692, 2007, 19712, 1289, 65620, 4291, 5857, 383, 21807, 1426, 10093, 660, 14329, 937,
         16776, 1097, 43691, 2857, 42437, 2775, 15522, 1015, 92780, 6067, 22006, 1439, 43385, 2837, 21379, 1398,   
         22327, 1460, 39164, 2561, 47269, 3091, 38537, 2520, 34882, 2281, 11087, 725, 31640, 2069, 43721, 2859,
         61201, 4002, 18397, 1203, 22526, 1473, 39669, 2594, 135752, 8877, 51123, 3343, 25248, 1651, 46994, 3073,    
         271334, 17743, 86907, 5683, 41519, 2715, 26930, 1761, 17311, 1132, 25156, 1645, 51306, 3355, 220654, 14429,
         40663, 2659, 35907, 2348, 11760, 769, 42284, 2765, 47361, 3097, 23367, 1528, 5123, 335, 27863, 1822,    
         35402, 2315, 30539, 1997, 9099, 595, 40632, 2657, 36977, 2418, 29132, 1905, 32741, 2141, 21287, 1392,
         28643, 1873, 105687, 6911, 44899, 2936, 20385, 1333, 36228, 2369, 21486, 1405, 40097, 2622, 34974, 2287,    
         5857, 383, 21807, 1426, 10093, 660, 14329, 937, 13442, 879, 20660, 1351, 45923, 3003, 91219, 5965]
,                # 21.096000 – 21.096200 MHz
'15m': [ 82574, 3937, 32866, 1567, 40878, 1949, 26448, 1261, 26385, 1258, 61558, 2935, 35173, 1677, 51155, 2439,  # updated       
         38969, 1858, 27832, 1327, 35781, 1706, 34187, 1630, 82489, 3933, 34103, 1626, 45995, 2193, 24581, 1172,
         56881, 2712, 24833, 1184, 46457, 2215, 34439, 1642, 16779, 800, 69507, 3314, 27161, 1295, 56713, 2704,    
         22253, 1061, 115229, 5494, 27811, 1326, 73890, 3523, 42010, 2003, 56272, 2683, 73701, 3514, 25357, 1209,
         51239, 2443, 76051, 3626, 53630, 2557, 44821, 2137, 34334, 1637, 72653, 3464, 12773, 609, 13570, 647,    
         42094, 2007, 46855, 2234, 34145, 1628, 5558, 265, 34061, 1624, 43562, 2077, 19799, 944, 29300, 1397,
         32803, 1564, 5600, 267, 28797, 1373, 29594, 1411, 35907, 1712, 39095, 1864, 21540, 1027, 23931, 1141,    
         55559, 2649, 24602, 1173, 57929, 2762, 70618, 3367, 27706, 1321, 18205, 868, 26909, 1283, 32446, 1547,
         26385, 1258, 61558, 2935, 35173, 1677, 51155, 2439, 40668, 1939, 50232, 2395, 65375, 3117, 94067, 4485,    
         82489, 3933, 34103, 1626, 45995, 2193, 24581, 1172, 30852, 1471, 5537, 264, 30055, 1433, 45869, 2187]
,                # 18.106000 – 18.106200 MHz
'17m': [ 144497, 5336, 97243, 3591, 42163, 1557, 26863, 992, 43571, 1609, 86465, 3193, 2383, 88, 75579, 2791,     # updated    
         120449, 4448, 60739, 2243, 41973, 1550, 31737, 1172, 82673, 3053, 33822, 1249, 57056, 2107, 27675, 1022,
         31629, 1168, 37072, 1369, 56461, 2085, 36395, 1344, 18387, 679, 29625, 1094, 38480, 1421, 11238, 415,    
         47443, 1752, 42325, 1563, 50178, 1853, 69296, 2559, 38615, 1426, 17087, 631, 66642, 2461, 32468, 1199,
         56813, 2098, 24155, 892, 45250, 1671, 21095, 779, 52453, 1937, 34743, 1283, 6472, 239, 29977, 1107,    
         25265, 933, 127354, 4703, 29706, 1097, 14000, 517, 23586, 871, 34526, 1275, 61876, 2285, 258796, 9557,
         109916, 4059, 22801, 842, 52751, 1948, 62283, 2300, 33389, 1233, 49745, 1837, 47362, 1749, 4089, 151,    
         51911, 1917, 44058, 1627, 99733, 3683, 55675, 2056, 43083, 1591, 49582, 1831, 41377, 1528, 69079, 2551,
         43571, 1609, 86465, 3193, 2383, 88, 75579, 2791, 79749, 2945, 43625, 1611, 35447, 1309, 52832, 1951,
         82673, 3053, 33822, 1249, 57056, 2107, 27675, 1022, 22232, 821, 67725, 2501, 49257, 1819, 40023, 1478]
,                # 14.097000 – 14.097200 MHz
'20m': [ 54552, 1387, 27217, 692, 86882, 2209, 37915, 964, 71700, 1823, 122594, 3117, 40786, 1037, 45781, 1164,   # updated 
         100332, 2551, 18210, 463, 27256, 693, 45348, 1153, 82161, 2089, 17266, 439, 47393, 1205, 38701, 984,
         32212, 819, 91051, 2315, 85466, 2173, 31976, 813, 60530, 1539, 65407, 1663, 45191, 1149, 40078, 1019,    
         22497, 572, 49399, 1256, 76183, 1937, 53686, 1365, 29891, 760, 59664, 1517, 29773, 757, 72171, 1835,
         36971, 940, 15811, 402, 47315, 1203, 41966, 1067, 44719, 1137, 79330, 2017, 24739, 629, 9872, 251,    
         44404, 1129, 39881, 1014, 57501, 1462, 52978, 1347, 21081, 536, 12625, 321, 63007, 1602, 20963, 533,
         20806, 529, 57069, 1451, 56951, 1448, 46489, 1182, 63637, 1618, 63519, 1615, 170577, 4337, 43775, 1113,    
         21907, 557, 48101, 1223, 13097, 333, 52270, 1329, 79093, 2011, 37403, 951, 45623, 1160, 70401, 1790,
         71700, 1823, 122594, 3117, 40786, 1037, 45781, 1164, 43421, 1104, 72211, 1836, 62457, 1588, 47944, 1219,   
         82161, 2089, 17266, 439, 47393, 1205, 38701, 984, 94589, 2405, 45151, 1148, 73744, 1875, 12271, 312]
,                # 10.140100 – 10.140300 MHz
'30m': [ 101192, 1661, 80783, 1326, 67441, 1107, 42341, 695, 67197, 1103, 97658, 1603, 35152, 577, 69512, 1141,   # updated
         90712, 1489, 31009, 509, 50382, 827, 47275, 776, 69511, 1141, 54829, 900, 68719, 1128, 44777, 735,
         74447, 1222, 50931, 836, 44656, 733, 148833, 2443, 58546, 961, 41366, 679, 16388, 269, 20287, 333,    
         5422, 89, 61957, 1017, 64272, 1055, 42584, 699, 193668, 3179, 82548, 1355, 52453, 861, 38563, 633,
         52454, 861, 41488, 681, 56353, 925, 55561, 912, 160651, 2637, 131774, 2163, 53794, 883, 34299, 563,    
         36370, 597, 152425, 2502, 48737, 800, 68841, 1130, 57814, 949, 50869, 835, 105576, 1733, 41609, 683,
         53185, 873, 43011, 706, 94612, 1553, 163393, 2682, 40513, 665, 58424, 959, 50626, 831, 82549, 1355,    
         77309, 1269, 71887, 1180, 39416, 647, 85777, 1408, 70851, 1163, 46970, 771, 53123, 872, 107769, 1769,
         67197, 1103, 97658, 1603, 35152, 577, 69512, 1141, 45143, 741, 24125, 396, 78589, 1290, 47458, 779,    
         69511, 1141, 54829, 900, 68719, 1128, 44777, 735, 47701, 783, 39233, 644, 131528, 2159, 71521, 1174]
,                # 7.040000 – 7.040200 MHz  
'40m': [ 58868, 621, 158972, 1677, 158024, 1667, 62091, 655, 169588, 1789, 66072, 697, 40193, 424, 29102, 307,   # updated  
         59246, 625, 20665, 218, 79437, 838, 38107, 402, 24267, 256, 83323, 879, 72327, 763, 114889, 1212,
         52801, 557, 44459, 469, 64366, 679, 56024, 591, 14314, 151, 89107, 940, 56782, 599, 70622, 745,
         28912, 305, 51852, 547, 77541, 818, 48629, 513, 44837, 473, 10522, 111, 68156, 719, 18295, 193,
         137453, 1450, 78206, 825, 66167, 698, 30998, 327, 52137, 550, 6920, 73, 58583, 618, 85789, 905,    
         53653, 566, 61900, 653, 47681, 503, 58677, 619, 43889, 463, 48913, 516, 79531, 839, 94603, 998,
         49483, 522, 60574, 639, 49009, 517, 63797, 673, 124939, 1318, 33652, 355, 50241, 530, 70053, 739,   
         121430, 1281, 2749, 29, 139725, 1474, 71000, 749, 37917, 400, 73085, 771, 5024, 53, 72611, 766,
         169588, 1789, 66072, 697, 40193, 424, 29102, 307, 49293, 520, 62185, 656, 42373, 447, 61711, 651,    
         24267, 256, 83323, 879, 72327, 763, 114889, 1212, 64364, 679, 66639, 703, 56591, 597, 19622, 207]
,                # 3.570000 – 3.570200 MHz
'80m': [ 73098, 361, 40295, 199, 31993, 158, 63581, 314, 27133, 134, 40497, 200, 100635, 497, 139917, 691,   
         86864, 429, 81397, 402, 5062, 25, 90711, 448, 50012, 247, 72487, 358, 49607, 245, 22475, 111,
         38675, 191, 115215, 569, 84032, 415, 68238, 337, 32600, 161, 12959, 64, 77349, 382, 121693, 601,    
         94153, 465, 78967, 390, 54062, 267, 58719, 290, 30979, 153, 57301, 283, 101036, 499, 43735, 216,
         29563, 146, 58721, 290, 124124, 613, 36245, 179, 288337, 1424, 62365, 308, 68237, 337, 67832, 335,    
         62566, 309, 14376, 71, 33409, 165, 61756, 305, 30169, 149, 42925, 212, 64185, 317, 153477, 758,
         198031, 978, 49204, 243, 27943, 138, 97193, 480, 42319, 209, 138296, 683, 89700, 443, 65402, 323,    
         93545, 462, 4657, 23, 92735, 458, 50822, 251, 33611, 166, 25107, 124, 20855, 103, 87267, 431,
         27133, 134, 40497, 200, 100635, 497, 139917, 691, 52443, 259, 63782, 315, 23083, 114, 126349, 624,    
         50012, 247, 72487, 358, 49607, 245, 22475, 111, 53251, 263, 48999, 242, 44747, 221, 174331, 861]               
}
#
# Debug printing.  Turn it off for flying using the global constant MY_DEBUG_PRINT_ENABLE
#
def eprint(*args):
    if MY_DEBUG_PRINT_ENABLE:
        for val in args:
            print(val, end=' ')
        print('')
#
# Callsign has to have a max of 6 characters including A-Z, 0-9, and space
# The third character has to be a number and the callsign must be six
# characters long.  So for short call signs W7XX would front padded with a
# space ' W7XX ' and one space after. The first character could be a number,
# letter or space, The second character can be a letter or number, the  third
# must be a number, the last three can only be letters or a space
#
# Can handle callsigns: XXNXXX, XXNXX, XXNX, XNXX
#
def WSPR_FormatCallsign(inStr):
    Error = 0
    outStr = ''
    
    inStr = inStr.strip()  # remove leading/trailing whitespace
    inStr = inStr.upper()  # uppercase
    
    len_inStr = len(inStr)
    # check for spaces in the middle
    testArr = inStr.split(' ')
    if len(testArr) > 1:
        eprint('ERROR: Space in middle of callsign {inStr} not allowed.')
        Error = 1
        
    elif len_inStr > 6 or len_inStr < 4:
        eprint(f'ERROR: Callsign {inStr} is >6 or <4.')
        Error = 2
    else:
        # Look for the last digit in the callsign
        last_number_index = -1
        for i,c in enumerate(inStr):
            if c.isdigit():
                last_number_index = i
                
        if len_inStr == 6:
            # AB#CDE
            if last_number_index == 2:
                outStr = inStr
            else:
                eprint('ERROR: 6 char callsign with number not in position 3')
                Error = 3
        
        elif len_inStr == 5:
            # A#CDE - add a space at beginning
            if last_number_index == 1:
                outStr = " " + inStr
            elif last_number_index == 2:
                # AB#DE - add a space at end
                outStr = inStr + " "
            else:
                eprint('ERROR: 5 char callsign with number not in position 2 or 3')
                Error = 4
            
        elif len_inStr == 4:
            # A#AA - add a space at the beginning and the end
            if last_number_index == 1:
                outStr = " " + inStr + " "
            # AA#A - add two spaces at the end
            elif last_number_index == 2:
                outStr = inStr + "  "
            else:
                eprint('ERROR: 4 char callsign with number not in position 2 or 3')
                Error = 5
    
    return Error, outStr
#
# Part of the WSPR endocing process.  First call WSPR_FormatCallsign() to get the
# callsign into the proper format
#
def WSPR_ConvertCallsign(FormattedCallsignString):
    # Derived from: "The WSPR Coding Process", PDF file, Andy Talbot, G4JNT, June 2009.
    # 
    N_array = [0, 0, 0, 0, 0, 0]
    #
    # 37 characters allowed as integers 0-36: 0-9 will be 0-9, A-Z will be 10-35, and [space] will be 36
    #
    for i,c in enumerate(FormattedCallsignString):
        N = 0
        if c == ' ':								# if the character is a space
            N = 36									# 36
        elif c.isdigit():							# if the character is a number
            N = ord(c) - ord('0')					# 0 to 9
        else:										# character must be a letter
            N = ord(c.upper()) - ord('A') + 10		# 10 to 35
        
        N_array[i] = N								# set our array value
    #
    # Now convert the callsign to a ~28 bit long number from the formula in the paper:
    # N1 = [Ch 1] The first character can take on any of the 37 values including [sp],
    # N2 = N1 * 36 + [Ch 2] but the second character cannot then be a space so can have 36 values
    # N3 = N2 * 10 + [Ch 3] The third character must always be a number, so only 10 values are possible.
    # N4 = 27 * N3 + [Ch 4] – 10]
    # N5 = 27 * N4 + [Ch 5] – 10] Characters at the end cannot be numbers,
    # N6 = 27 * N5 + [Ch 6] – 10] so only 27 values are possible.
    # (In practice N will just be one 32 bit integer variable that can be built up in stages)
    #
    N1 = N_array[0]
    N2 = N1 * 36 + N_array[1]
    N3 = N2 * 10 + N_array[2]
    N4 = N3 * 27 + N_array[3] - 10
    N5 = N4 * 27 + N_array[4] - 10
    N6 = N5 * 27 + N_array[5] - 10
    
    return N6
#
# Part of the process of endocing the WSPR message.  Essentially it turns the Maidenhead
# Grid String into a number that will be encoded into the WSPR message.
#
def WSPR_ConvertLocator(MDNHGridString):
    #
    # Derived from: "The WSPR Coding Process", PDF file, Andy Talbot, G4JNT, June 2009.
    #
    Loc_array = [0, 0, 0, 0]
    #
    # Make A-R into a number from 0 to 17
    #
    Loc_array[0] = ord(MDNHGridString[0].upper()) - ord('A')
    Loc_array[1] = ord(MDNHGridString[1].upper()) - ord('A')
    #
    # Make the string numbers into integers from 0 to 17
    #
    Loc_array[2] = ord(MDNHGridString[2]) - ord('0')
    Loc_array[3] = ord(MDNHGridString[3]) - ord('0')
    #
    # From the paper above, calculate M1
    # M1 = (179 - 10 * [Loc 1] - [Loc 3] ) * 180 + 10 * [Loc 2] + [Loc 4]
    #
    M1 = (179 - 10 * Loc_array[0] - Loc_array[2] ) * 180 + 10 * Loc_array[1] + Loc_array[3]
    
    return M1
#
# This function is called to pack the Type 1 Message data before being encoded for transmission.
# it is only called by WSPR_Create4FSKSymbols() which is called by any of the other encoding functions
# FormattedCallsignString is generated by the WSPR_FormatCallsign() function.  It is important that
# the callsign be in the proper format before this function is called.  It isn't formatted here because
# we use this function to pack messages that don't contain a nomal callsign.
#
def WSPR_PackType1Msg(FormattedCallsignString, MDNHGridString, OutputPower):
    # Derived from: "The WSPR Coding Process", PDF file, Andy Talbot, G4JNT, June 2009.
    # This is the Bit Packing Step
    # M = M1 * 128 + [Pwr] + 64
    # Power can be a value from 0 to 60 but only those ending in 0,3,or7 will work
    # The Maidenhead Grid String will only use the first 4 positions of the locater.
    #
    M = WSPR_ConvertLocator(MDNHGridString) * 128 + OutputPower + 64
    
    N = WSPR_ConvertCallsign(FormattedCallsignString)

    M_binary = "{:022b}".format(M)		# Make M into a 22 bit binary number
    N_binary = "{:028b}".format(N)		# Make N into a 28 bit binary number

    C = N_binary + M_binary + '0'*31  # pad 0 at the end to make 81 digits

    return C

#
# Just count the number of 1 bits.  If even then output 0 if odd output 1.
#
def WSPR_CalculateParity(REG):
    p = 0
        
    ones_count = bin(REG).count('1')   
    p = ones_count % 2
    
    #p will be 0 if there are an even number of '1' bits and 1 if there are an odd number    
    return p
#
# C is a string containing 1s and 0s from the bit packing step (81 characters long)
#
def WSPR_ConvolutionalCoding(C):
    # Derived from: "The WSPR Coding Process", PDF file, Andy Talbot, G4JNT, June 2009.
    # See YouTube by 3Blue1Brown on convolutional coding for general explanation.
    # C is the 81 bit string of 1s and 0s from the Bit Packing Step
    #
    CW = '0'*31 + C   # pre-pad 0 at the beginning, effectively "clears" the REGs
    
    # Convert the Hex string into a base 16 integer
    # J. Layland and W. Lushbaugh, "A Flexible High-Speed Sequential Decoder for Deep Space Channels,"
    #
    MAGIC0 = int('F2D05351', 16) # Magic numbers from the paper mentioned above.
    MAGIC1 = int('E4613C47', 16)

    S = []
    # Shift each bit left one at a time for the next 81 bits.
    for i in range(0, 81):
        
        REG0_binary_string = REG1_binary_string = CW[i:i+32]		# Get 32 bits of CW and put it in the registers.  This increments each time through the loop
                                                                    # So loop 1 is 1 to 33, loop 2 is 2 to 34, etc.
        
        REG0 = int(REG0_binary_string, 2) & MAGIC0					# Bitwise AND with the magic number
        REG1 = int(REG1_binary_string, 2) & MAGIC1
        
        S.append(WSPR_CalculateParity(REG0))
        S.append(WSPR_CalculateParity(REG1))
        
    # Return array of 162 bits    
    return S
#
# S is the 162 bit array of 1s and 0s from the Convolution step.
#
def WSPR_Interleave(S):
    # Derived from: "The WSPR Coding Process", PDF file, Andy Talbot, G4JNT, June 2009.
    # For an explanation on a different error correction process see YouTube 3Blue1Brown
    # on the Hamming code - not exactly related but it helps if you're not familiar
    #
    # From the paper above:
    # Initialise a counter, P to zero
    # Take each 8-bit address from 0 to 255, referred to here as I
    # Bit-reverse I to give a value J.
    # For example, I = 1 gives J = 128, I = 13 J = 176 etc.
    # If the resulting bit-reversed J yields a value less than 162 then :
    # Set Destination bit D[J] = source bit S[P]
    # Increment P
    # Stop when P = 162

    D = [None]*162
    P = 0
    for I in range(0,256):
        J = '{:08b}'.format(I)		# Make the value I into an 8 bit binary string
        J = "".join(reversed(J))	# reverse string
        J = int(J, 2)  				# convert string of binary digits to a base 10 integer
        if J < 162:
            D[J] = S[P]
            P += 1
        if P >= 162: break
        
    # return 162 value array of 1s and 0s    
    return D
 #
 # D is the 162 value array from the Interleave step.  This generates an array of 162
 # values from 0 to 3.  These will dictate the four frequencies output.
 #
def WSPR_MergeWithSyncVector(D):
    # Derived from: "The WSPR Coding Process", PDF file, Andy Talbot, G4JNT, June 2009.
    # From the paper: Each symbol represents a frequency shift of 12000 / 8192, or approximately 1.46Hz, per
    # Symbol Value giving four-level Multi-FSK modulation. The transmitted symbol length is
    # the reciprocal of the tone spacing, or approximately 0.683 seconds, so the complete
    # message of 162 symbols takes around 110.6 seconds to send and occupies a bandwidth
    # of approximately 6Hz
    #
    global SYNC							# Sync Vector Constant from the paper.
    
    Symbol = [None]*162
    
    for i, Data in enumerate(D):
        Symbol[i] = SYNC[i] + 2 * Data
    #
    # Return an array with each index being a value from 0 to 3.
    return Symbol
#
# Create the symbols we'll send (0,1,2,3) That is: Modulation is continuous phase 4 FSK,
# with 1.4648 Hz tone separation. (from Wikipedia).  This function is called after the Basic
# Telemetry and the Extended Telemetry Encoding before transmitting the symbols.
# WSPR_Create4FSKSymbols
# WSPR_FormatCallsign() should be used to format the callsign.
# grid4Str is a 4 Character Maidenhead Grid (if it is 6 the pack step reduces it to the first 4
# powerDBmNum is the output power of the radio but has special notation values from 0 to 60 but
#   only those ending in 0,3,or7 will work
#
def WSPR_Create4FSKSymbols(formattedCallsignSr, grid4Str, powerDBmNum):
    
    C = WSPR_PackType1Msg(formattedCallsignSr, grid4Str, powerDBmNum)		#BitPack
    S = WSPR_ConvolutionalCoding(C)											#Convolve
    D = WSPR_Interleave(S)													#Interleave
    WSPR_Symbols = WSPR_MergeWithSyncVector(D)											# Merge with the Sync Vector
    
    for i,c in enumerate(WSPR_Symbols):			# Convert the integer array to a character array.
        WSPR_Symbols[i] = str(c)
    
    return WSPR_Symbols

#
# This function encodes "Basic Telemetry" (traquito site name for this).  The Basic Telemetry is the
# second message sent and is usually fixed to the parameters here although it looks like that isn't an
# absolute.  
# Once this is called then WSPR_Create4FSKSymbols() must be called to create symbols for the Transmission.
# Channel is from the Traquito web site Channel Map Page.
# Locater56 is the 5th and 6th letters of the maidenhead grid position "Subsquare"
# Altitude in meters 0 - 21340
# Millivolts 3000 to 4950 from the Pico voltage sense typically
# Temperature -50 to 39 C from the Pico temp sensor (not particularly accurate) or a add on board
# Speed in knots from the GPS 0 - 82 knots
# GPSValid - not used in this code as it only sends if the GPS Valid is true so just set to True
#
def WSPR_EncodeBasicTelemetry(Channel, Locator56, Altitude, Temperature, Millivolts, Speed, GPSValid):
    #
    # The Basic telemetry is encoded into the Callsign, Locater, and Power of the Slot 0 (first) message.  Once
    # encoded here it is fed to the original function to prepare it for sending by encoding it in WSPR format.
    # Derived from spreadsheet 308d.xls (http://www.qrp-labs.com/images/ultimate3builder/ve3kcl/s4/308d.xls)
    # There is more explanation here: https://qrp-labs.com/ultimate3/ve3kcl-balloons/ve3kcl-s4.html#protocol
    # Alternatively it looks like the Slot 2 Extended Telemetry format could be used as well but I haven't
    # explored that.  The encoding is done in a specific way so the pico balloon website(s) can read and
    # decode the data.
    #
    # Force the values to be in an expected range.
    #
    Channel = int(Channel)
    if Channel < 0:
        Channel = 0
    elif Channel > 599:
        Channel = 599

    Temperature = int(Temperature)
    if Temperature < -50:
        Temperature = -50
    elif Temperature > 39:
        Temperature = 39
    
    Millivolts = int(Millivolts)
    if Millivolts < 3000:
        Millivolts = 3000
    elif Millivolts > 4950:
        Millivolts = 4950
        
    Speed = int(Speed)
    if Speed < 0:
        Speed = 0
    elif Speed > 82:
        Speed = 82

    Altitude = int(Altitude)
    if Altitude < 0:
        Altitude = 0
    elif Altitude > 21340:
        Altitude = 21340
    #
    # Note this isn't really needed for us since we simply don't send if the GPS is
    # not valid.
    #
    if GPSValid:
        GPSValid = 1
    else:
        GPSValid = 0
    #
    # pack the Channel id13 (top row of table) into the callsign 00 to 19 and Q0 to Q9
    # Callsign positions 0 and 2  For example Channel 298 is 14
    # Callsign 0 is allowed 37 values
    # Select 0, 1, or Q based on the callsign range.  This is the id13 accross the top of the
    # channel map table on the Traquito web site.
    #
    CALLSIGN = [None]*6

    CALLSIGN[0] = ''
    if 0 <= Channel and Channel <= 199:
        CALLSIGN[0] = '0'
    elif 200 <= Channel and Channel <= 399:
        CALLSIGN[0] = '1'
    elif 400 <= Channel and Channel <= 599:
        CALLSIGN[0] = 'Q'
    else:
        # This error should not happen as we checked it before now.
        eprint('ERROR: Channel not in range(0-599): ', Channel)
        return None
    #
    # Next calculate the second index digit of id13 which will be 0 to 9 making 00 to 19 and Q0 to Q9
    # Callsign 2 is allowed 10 (has to be a number)
    CALLSIGN[2] = chr(  int(Channel / 20) % 10 + 48  )
    #
    # Next Calculate what fits in Callsign positions 1, 3, 4, 5 the grid subsquare and altitude
    # Callsign position 1 is allowed 36 values (A-Z,0-9), 3, 4, and 5 are allowed 27 values (A-Z,' ')
    # So 36 * 26 * 26 * 26 = 632 736 posibilities.
    # Get the last two digits of the 6 digit maidenhead - The subsquare
    # check if the character is from A to X X is 88
    #
    if 65 <= ord(Locator56[0]) and ord(Locator56[0]) <= 88:
        # good
        pass
    else:
        eprint('ERROR: Bad Maidenhead locator position 5: ', Locator56[0])
        return None
    # check if the second character is from A to X    
    if 65 <= ord(Locator56[1]) and ord(Locator56[1]) <= 88:
        # goood
        pass
    else:
        eprint('ERROR: Bad Maidenhead locator position 6', Locator56[1])
        return None
    #
    # get the integer for the character - 'A' (65) so that A to X is 0 to 23
    # Then 24 * 23 + 23 = 575
    Locator5_ord = ord(Locator56[0]) - 65  #  24 values A to X
    Locator6_ord = ord(Locator56[1]) - 65  #  24 values A to X
    # Move locator5 to values 24 to 575, then locater6 will be 0 to 23 to make a total from 0 to 575
    # So 0-23 always mean the subgrid starts with A.  AA is 0 and 575 would be XX
    LocatorBits_ord = 24 * Locator5_ord + Locator6_ord   # 576 posible values
    #
    # Now the altitude.  It can be 0 to 21340 meters in 20 meter steps or 1067 20 meter steps.
    # so the index will be 0 to 1067 
    Alt_index = round((Altitude) / 20)

    # Move the locatorbits from 1068 to 615 167.  Altitude will be from 0 to 1067
    MSW = 1068 * LocatorBits_ord + Alt_index
    
    # MSW_char is callsign 1 and can have 36 values A-Z and 0-9.
    MSW_char_index = int( MSW / (26*26*26) )
    
    if MSW_char_index < 10:								# 0 to 9
        MSW_char = chr( MSW_char_index + 48 )			# 48 is '0'
    else:												# A to Z
        MSW_char = chr( MSW_char_index - 10 + 65 )		# 65 is 'A'

    # CSW is callsign 3 and can have 27 values A to Z (we don't use the space though)
    CSW = MSW - (MSW_char_index * (26*26*26))	# subtract off what was encoded already
    CSW_char_index = int( CSW / (26*26) )
    CSW_char = chr( CSW_char_index + 65 )

    # CSW can have 27 values A to Z (we don't use the space though)
    LSW = CSW - (CSW_char_index * (26*26))
    LSW_char_index = int( LSW / (26) )
    LSW_char = chr( LSW_char_index + 65 )

    # CSW can have 27 values A to Z (we don't use the space though)
    DSW = LSW - (LSW_char_index * (26))
    DSW_char_index = DSW
    DSW_char = chr( DSW_char_index + 65 )

    CALLSIGN[1] = MSW_char
    CALLSIGN[3] = CSW_char
    CALLSIGN[4] = LSW_char
    CALLSIGN[5] = DSW_char
    #
    # Done with the CALLSIGN portion of the message
    #
    # Now on to figuring out what the locator and power portion will be.
    # Originally locator would hold A-R, A-R, 0-9, 0-9.  Not all combinations are used
    # AA00 = 32220 to RR99 = 179
    # The locator can have 18 * 18 * 10 * 10 = 32,400
    # The power can hold 19 
    #
    POWERS = [0,3,7,10,13,17,20,23,27,30,33,37,40,43,47,50,53,57,60]
   
    Temp_int = int(Temperature + 50)  # Temp has 90 values (0-89) representing -50 to 39.
    #
    # Battery is 40 values (0-39) representing 3000 to 4950 50mV each.
    # 1950 / 50 = 39 values and 0=3000.
    Bat_int = (int(((Millivolts) - 3000) / 50) + 20) % 40
    #
    # Speed is 42 values: 0 and 2 to 82 knots (rounded to nearest integer)
    Speed_int = round((Speed) / 2.0) 
    #
    # The following is just how the U4 message format says to encode it into the power
    # fields: 
    #
    Result1 = Temp_int * 40 + Bat_int			# 90 temp values * 40 Bat values
    Result2 = Result1 * 42 + Speed_int			# above * 42 speed values
    Result3 = Result2 * 2 + GPSValid			# above * 2 GPS
    Result = Result3 * 2 + 1    				# values set to 1 = 8 satellites tracking

    A = Result
    A_char_index = int(A / 34200)
    A_char = chr(  A_char_index + 65 )

    B = A - A_char_index*34200
    B_char_index = int( B / 1900 )
    B_char = chr(  B_char_index + 65 )

    C = B - B_char_index*1900
    C_char_index = int( C / 190 )
    C_char = chr(  C_char_index + 48 )

    D = C - C_char_index*190
    D_char_index = int( D / 19 )
    D_char = chr(  D_char_index + 48 )

    E = D - D_char_index*19
    E_char_index = E
    E_val = POWERS[E_char_index]


    out = [CALLSIGN[0] + CALLSIGN[1] + CALLSIGN[2] + CALLSIGN[3] + CALLSIGN[4] + CALLSIGN[5], \
        A_char + B_char + C_char + D_char, \
        E_val]
    
    # output in the form of Callsign (string), Locater (4 digit) (string), and power (int)
    # (mimic slot 1 so it can then be packed by the slot 1 pack routine)
    #
    return out[0], out[1], out[2]
#
# This function encodes the Extended Telemetry after the Big Number has
# been packed with our data then the header data. This is used to send data
# in slot 2 (third message) and slot 3 (fourth message) if used.  I've heard
# a fifth is not recomended but I haven't looked into it.
# Channel is the channel set in the constant at the top.
# SlotNum is the slot we're transmitting 0,1,2,3
# Big Number is the number from encoding the extended telemetry. See the
# function WSPR_ExtTel1EncodeMeasurements() & WSPR_ExtTel2EncodeMeasurements() for an example.
#
def WSPR_EncodeExtendedTelemetry(Channel, SlotNum, BigNumber):
    #
    # This is similar to the WSPR_EncodeBasicTelemetry() function but has been
    # genericized - if that is a word - to work easily with extra telemetry data
    # without having to write a custom packing function for each sensor.  It is
    # sort of discussed on the Traquito web site.  Again the encoding just has
    # to match the decoding done by the web site(s) that display the data.
    #
    # Add the Extended Telemetry Header to the message
    # Use default values.  
    BigNum = WSPR_EncodeExtTelemHeader(0, 0, 0, SlotNum, BigNumber)

    Channel = int(Channel)
    if Channel < 0:
        Channel = 0
    elif Channel > 599:
        Channel = 599
    #
    # pack the Channel id13 (top row of table) into the callsign 00 to 19 and Q0 to Q9
    # Callsign positions 0 and 2  For example Channel 298 is 14
    # Callsign 0 is allowed 37 values
    # Select 0, 1, or Q based on the callsign range.
    #
    CALLSIGN = [None]*6

    CALLSIGN[0] = ''
    if 0 <= Channel and Channel <= 199:
        CALLSIGN[0] = '0'
    elif 200 <= Channel and Channel <= 399:
        CALLSIGN[0] = '1'
    elif 400 <= Channel and Channel <= 599:
        CALLSIGN[0] = 'Q'
    else:
        #error
        eprint('ERROR: Channel not in range(0-599): ', Channel)
        return None  
    #
    # Next calculate the second index digit of id13 which will be 0 to 9 making 00 to 19 and Q0 to Q9
    # Callsign 2 is allowed 10 (has to be a number)
    #
    CALLSIGN[2] = chr(  int(Channel / 20) % 10 + 48  )
    #
    # The above portion is the same as the slot 1 (2nd slot).

    #36
    C_1 = BigNum // 1900
    C_1 = C_1 // (18*18 * 26*26*26)
    MSW_char_index = int(C_1) % 36
    if MSW_char_index < 10:
        MSW_char = chr( MSW_char_index + 48 )
    else:
        MSW_char = chr( MSW_char_index - 10 + 65 )
    CALLSIGN[1] = MSW_char
    
    #26
    C_3 = BigNum // 1900
    C_3 = C_3 // (18*18 * 26*26)
    CSW_char_index = int(C_3) % 26
    CSW_char = chr( CSW_char_index + 65 )
    CALLSIGN[3] = CSW_char
    
    #26
    C_4 = BigNum // 1900
    C_4 = C_4 // (18*18 * 26)
    LSW_char_index = int(C_4) % 26
    LSW_char = chr( LSW_char_index + 65 )
    CALLSIGN[4] = LSW_char
    
    
    #26
    C_5 = BigNum // 1900
    C_5 = C_5 // (18*18)
    DSW_char_index = int(C_5) % 26
    DSW_char = chr( DSW_char_index + 65 )
    CALLSIGN[5] = DSW_char
    
    #18
    A = BigNum // 1900
    A = A // (18)
    #A_char_index = int(A / 34200) % 18
    A_char_index = int(A) % 18
    A_char = chr(  A_char_index + 65 )
    
    
    #18
    B = BigNum // 1900   # integer division
    B_char_index = B % 18
    B_char = chr(  B_char_index + 65 )
    
    
    
    #10
    C = BigNum % 1900
    #eprint('Ca',C)
    C = C / (10 * 19)
    #eprint('Cb:',C)
    C_char_index = int(C) % 10
    C_char = chr( C_char_index + 48 )
    
    
    
    #10
    D = BigNum % 1900
    #eprint('Da:',D)
    D = D / (19)
    #eprint('Db:',D)
    D_char_index = int(D) % 10
    D_char = chr( D_char_index + 48 )
    
    #19
    E = BigNum % 19
    E_char_index = E
    POWERS = [0,3,7,10,13,17,20,23,27,30,33,37,40,43,47,50,53,57,60]
    E_val = POWERS[E_char_index]


    out = [CALLSIGN[0] + \
        CALLSIGN[1] + \
        CALLSIGN[2] + \
        CALLSIGN[3] + \
        CALLSIGN[4] + \
        CALLSIGN[5], \
        A_char + B_char + C_char + D_char, \
        E_val]
    #
    # output in the format of formatted callsign, locater, power
    #
    return out[0], out[1], out[2]
# 
# This function encodes the Extended Telemetry Header Fields. Which is done after
# the user defined fields are packed into the Big Number
#
def WSPR_EncodeExtTelemHeader(HdrTelemetryType, HdrRESERVED, HdrType, HdrSlot, BigNumber):

    #It appears that this comes from the original traquito code in WsprMessageTelemetryExtendedCommon.h
    #HdrTelemetryType is lowValue=0, HighValue=1, StepSize=1, NumValues=2,  NumBits=1,       Value=0 meaning extended telemetry?
    #HdrRESERVED         lowValue=0, HighValue=3, StepSize=1, NumValues=4,  NumBits=2,       Value=0
    #HdrType             lowValue=0, HighValue=15,StepSize=1, NumValues=16, NumBits=4,       Value=0 user defined
    #HdrSlot             lowValue=0, HighValue=4 ,StepSize=1, NumValues=5 , NumBits=log2(5), Value=0 
     
    # Infinite thanks to Doug.
    out = 0
    #
    out = BigNumber
    
    # HdrSlot
    out = out * 5
    out = out + HdrSlot
    
    #HdrType
    out = out * 16
    out = out + HdrType   # 0 - User Defined
    
    #HdrRESERVED
    out = out * 4
    out = out + HdrRESERVED   # 0 - Reserved for future use.
    
    #HdrTelemetryType
    out = out * 2
    out = out + HdrTelemetryType  #  0 - User Defined
    
    return out

#
# Encode our Sensor Measurements into a Big Number to transmit in the Extended Telemetry.
#
# This is the .json file used with the website to decode the extended telemetry:
#// Definition for the SEN0501 for Traquito Web site
#
#{ "name": "Humidity",     "unit": "Percent",  "lowValue":   0,    "highValue": 100,    "stepSize": 1},
#{ "name": "UV",           "unit": "mwpcm2",   "lowValue":   0,    "highValue":    16,    "stepSize":  0.5   },
#{ "name": "Luminosity",   "unit": "klx",      "lowValue":   0,    "highValue":   120,    "stepSize":  2   },
#{ "name": "Pressure",     "unit": "mBar",     "lowValue":   0,  "highValue":     1100,  "stepSize":  0.5 },
#
def WSPR_ExtTel1EncodeMeasurements(Humidity, UV, Luminosity, Pressure):
    # The info below is from the Traquito Pro Tools "CODEC Extended Telemetry"  This has to match the message
    # definition exactly or the decode won't work.
    # Humidity 0-100%, 101 Values, StepSize = 1
    # UV 0-16mw/cm^2, 33 Values, Step Size = 0.5
    # Luminosity 0-120klx, 61 Values, StepSize 2
    # Pressure 0-1100mBar, 2201 Values, StepSize = 0.5

    # This is from the Traquito web site example which starts as 0 so the first term with 120 is of no
    # value but here anyway for clarity perhaps.
    # packing values into a big number:  https://traquito.github.io/pro/telemetry/#conversion-to-big-number-stage
    #
    # Note the example says to encode these in reverse order [from the extended telemetry definition on the web site]
    # Ok, what the traquito site doesn't say is we should send the index value.  That is if my pressure is 500 I would
    # send 1000 because the step size is 1/2.  This makes sense because we have to have integer values to send.
    
    out = 0
    
    x = int(Pressure * 2)		# because Pressure has 0.5mBar Steps
    if x < 0:
        x = 0
    elif x > 2200:				# adjusted for multiplication of value above.
        x = 2200			
    out = out * 2201 + x		# Yep, it will just be x.  see the coment above
    
    x = int(Luminosity / 2)		# because Luminosity has 2 klux steps
    if x < 0:
        x = 0
    elif x > 60:
        x = 60				
    out = out * 61 + x
    
    x = int(UV * 2)				# because UV has 0.5 mw/cm^2 steps
    if x < 0:
        x = 0
    elif x > 32:
        x = 32
    out = out * 33 + x
    
    x = int(Humidity)
    if x < 0:
        x = 0
    elif x > 100:
        x = 100
    out = out * 101 + x
    
    return out

#
# Encode our Sensor Measurements into a Big Number to transmit in the Extended Telemetry.
#
# This is the .json file used with the website to decode the extended telemetry:
# // Speed and Pressure High Resolution.
# { "name": "HighSpeed",    "unit": "MPH",      "lowValue":   0,    "highValue": 300,  "stepSize": 1},
#
def WSPR_ExtTel2EncodeMeasurements(Speed):
    # The info below is from the Traquito Pro Tools "CODEC Extended Telemetry"  This has to match the message
    # definition exactly or the decode won't work.
    # HighSpeed, MPH, 301 Values, Step Size 1
    # 

    # This is from the Traquito web site example which starts as 0 so the first term with 120 is of no
    # value but here anyway for clarity perhaps.
    # packing values into a big number:  https://traquito.github.io/pro/telemetry/#conversion-to-big-number-stage
    #
    # Note the example says to encode these in reverse order [from the extended telemetry definition on the web site]
    # Ok, what the traquito site doesn't say is we should send the index value.  That is if my pressure is 500 I would
    # send 1000 because the step size is 1/2.  This makes sense because we have to have integer values to send.
    
    out = 0
    
    Speed = Speed * 1.1508         # Speed in Knots from the GPS converted to MPH
    
    x = int(Speed)                 # Extended Speed from the GPS
    if x < 0:
        x = 0
    elif x > 300:
        x = 300
    out = out * 301 + x
    
    return out

#
# Just check the channel so an error will be created immediatly instead of waiting
# until transmit time.  Use at the start of the main program.
#
def TRAQUITO_CheckChannel(Channel):
    Error = 0
    
    if (Channel < 0) or (Channel > 599):
        Error = 1
        
    return Error

def TRAQUITO_CheckBandString(TransmitBand):
    Error = 0
    
    if not isinstance(TransmitBand, str):
        eprint('ERROR: Band String is not a string.')
        Error = 1
    else:    
        TransmitBand = TransmitBand.lower()
        
        if TransmitBand == '10m' or \
            TransmitBand == '12m' or \
            TransmitBand == '15m' or \
            TransmitBand == '17m' or \
            TransmitBand == '20m' or \
            TransmitBand == '22m' or \
            TransmitBand == '30m' or \
            TransmitBand == '40m' or \
            TransmitBand == '80m' or \
            TransmitBand == '160m':
            Error = 0
        else:
            TransmitBand = ' '
            Error = 2
        
    return Error, TransmitBand
#
# This is part of the channel map on the traquito web site.
# See the right hand column.  When a channel is selected
# the tracker will start its transmission on a specific minute
# This takes into account the band.  It is used to help sort
# out the extended telemetry as belonging to your call sign.
#
def TRAQUITO_GetChannelTime(Channel):
    global MY_BAND  # set by the user
    # band    10  12  15  17  20  30  40  80  160
    # time     4,  0,  6,  2,  8,  4,  0,  2,  8
    # offset   3   0   2   4   1   3   0   4   1
    
    Error = 0
    MyChannelStartTime = 0
    offset = 1
    if not (0 <= Channel <= 599):
        eprint('ERROR: Channel out of range: ', Channel)
        Error = 1
    elif MY_BAND == '10m': offset = 3
    elif MY_BAND == '12m': offset = 0
    elif MY_BAND == '15m': offset = 2
    elif MY_BAND == '17m': offset = 4
    elif MY_BAND == '20m': offset = 1
    elif MY_BAND == '30m': offset = 3
    elif MY_BAND == '40m': offset = 0
    elif MY_BAND == '80m': offset = 4
    elif MY_BAND == '160m': offset = 1
    else:
        eprint('ERROR: Offset not selected by band: ', MY_BAND)
        Error = 2
        
    if Error == 0:
        # Figure out the channel start time.
        MyChannelStartTime = ((Channel - offset) % 5) * 2
    
        if MyChannelStartTime < 0 or MyChannelStartTime > 8 or (MyChannelStartTime % 2) != 0:
            eprint('ERROR: Channel Start Time is odd or out of range 1-7: ', MyChannelStartTime)
            Error = 3
    
    return Error, MyChannelStartTime
#
# This function calculates lane you would see on the Traquito website under
# Channel Map.  From channel and lane we figure out the numbers we'll use
# to set the SI5351 clock for the four frequencies we'll output.
#
def	TRAQUITO_CalculateChannelParams(channel):
    Error = 0
    Lane = 0
    # check if channel out of range
    channel = int(channel)
    if channel < 0 or channel > 599:
        Error = 1
        eprint('Error: Channel out of range (0-599): ', channel)

    # Calculate the Lane.  Has to be between 1 and 4
    Lane = int((int(channel/5) % 4) + 1)
    if Lane < 1 or Lane > 4:
        Error = 2
        eprint('Error: Lane out of range (1 - 4): ', Lane)

    # MY_FREQ_OFFSET Shift output freq by X Hertz,   valid: -20, -10, 0, +10, +20
    # Not sure what this was for but for the traquito it probably should always be 0
    #
    if abs(MY_FREQ_OFFSET) != 20 and abs(MY_FREQ_OFFSET) != 10 and MY_FREQ_OFFSET != 0:
        Error = 3
        eprint('Error: FREQ_OFFSET out of range (-20,-10,0,10,20): ', MY_FREQ_OFFSET)
    else:
        offset_index = (MY_FREQ_OFFSET // 10) + 2  # floor division, so -2, -1, 0, 1, 2 are possible.
    #
    # Example of how the parameters are found for 10m band, channel 298, lane 4
    #
    # for offset 0 (offset index = 2) [10m][64:96]  then [24:32] from that array. (python doesn't get the 32nd)
    # 64 to 96 is:
    # 17927, 3056, 28797, 4909, 31865, 5432, 13193, 2249, 13498, 2301, 17223, 2936, 23183, 3952, 35103, 5984,   		#64-79, 0-15 
    # 24403, 4160, 32727, 5579, 27119, 4623, 27776, 4735,  10776, 1837, 18836, 3211, 47703, 8132, 12747, 2173			#80-95, 16-31
    # from that: 24 - 32
    # 10776, 1837, 18836, 3211, 47703, 8132, 12747, 2173  These will be used to calculate the SI5351's output frquency for each symbol.
    #
    #eprint(f'Band: {MY_BAND} Channel: {channel} Lane: {Lane} OffsetIndex: {offset_index}')
    #eprint('param list: ', SIGGENPARAMS[MY_BAND][(32*offset_index):(32*(offset_index+1))])
    
    ChannelSigGenParams = SIGGENPARAMS[MY_BAND][(32*offset_index):(32*(offset_index+1))][(8*(Lane-1)):(8*(Lane))]
    #eprint('Siggenparms Array (8 items exactly): ', ChannelSigGenParams)
    if len(ChannelSigGenParams) != 8:
        eprint('ERROR: Signal generation array is not 8 integers: ', ChannelSigGenParams)

    return Error, ChannelSigGenParams
#
# Turn the SI5351 and its TCXO on or off.
#
def TX_Power(OnOff):
    #
    # TX_ON = 1, TX_OFF = 0 even though the pin is set the opposite way
    #
    # Pico Pin 34, GP28
    
    Pico34 = Pin(28, Pin.OUT) # Osc, low = ON  On jetpack diagram: WSPR_LSWITCH (both transmitter and crystal)
    
    if (OnOff <= 0):
        Pico34.value(1)   # 1 = Off
    else :
        Pico34.value(0)   # 0 = On
   
    return
#
# Get ready to use the SI5351.  This will need to be called every time the SI5351 is powered
# off.
#
def TX_Init(TxI2C, ChannelSigGenParams):
    Error = 0
    PLLFreq = 0
    
    SI5351.InitializeClocks(TxI2C)
    #
    # The PLL is setup in the init function but could be used to change it so here it is
    # For traquito: 26MHz * (m + n/d) so 30 = 780MHz
    PLLFreq, Error = SI5351.SetupPLL(TxI2C, 30, 0, 1, 'A')
    #
    # The following is required to get the inverted output on CLK1 to work and stay working
    # once this is done then it seems to hold even though we're changing frequencies.  All
    # the steps are needed or the inverted phase on CLK1 drifts comparted to CLK0
    #
    SI5351.ClockSetInvertOutput(TxI2C, 1, 1)
    #
    # Get the values for the band we are on and then set this accordingly as when we don't
    # the phase issue comes back.
    SymbolNum = 2   
    DIV  = 16
    NUM  = ChannelSigGenParams[2 * SymbolNum]     
    DENOM= ChannelSigGenParams[2 * SymbolNum + 1]
    #
    # Example of 16 + 21552 / 1837  F = 780000000 / (16 + 21552 / 1837) = 28 126 177.8 Hz 10m
    #
    SI5351.SetupMultisynth(TxI2C, 0, DIV, NUM, DENOM)
    SI5351.SetupMultisynth(TxI2C, 1, DIV, NUM, DENOM)
    
    SI5351.PLLSoftReset(TxI2C, 'A')
    
    return PLLFreq, Error
#
# Send our WSPR message.  This function is BLOCKING it does not return until
# the message is sent which takes about 1 minute 50 seconds.  The sending loop
# is time critical so if you change anything in that loop adjust the time.
# Essentially it is looking at the symbol array which is an array of numbers
# 0,1,2,3 and it sets the frequency to one of the four frequencies for the chosen
# band and sends that frequency for about 0.683 seconds then goes to the next one
# until the entire symbol array or 162 symbols in our case, has been sent.
#
# SymbolArray is an array of symbols to send see above.
# ChannelSigGenParams comes from TRAQUITO_CalculateChannelParams().  I'm not sure why
# the author did it this way in particular but it seems fine.  
#
def TX_WSPRSymbols(TxI2C, SymbolArray, ChannelSigGenParams):
    global PLED
    
    for i,c in enumerate(SymbolArray):
        #
        # Example of what is being set by the ChannelSigGenParams:
        # 10m Channel 298, Lane 4 We should have 28 126 180
        # Each "tone" would be 12000/8192 Hz appart or 1.46 Hz 
        # SIGGENPARAMS are: 10776, 1837, 18836, 3211, 47703, 8132, 12747, 2173
        # Since we are doing 4FSK we have 0,1,2,3 as symbols being output.  We output the particular freq. for f0.683 seconds.
        # For symbol 0: 390 / (8 + 10776/1837) = 28 126 177.7638
        # For symbol 1: 390 / (8 + 18836/3211) = 28 126 179.1393 diference: 1.376
        # For symbol 2: 390 / (8 + 47703/8132) = 28 126 180.6153 diference: 1.476
        # For symbol 3: 390 / (8 + 12747/2173) = 28 126 182.3371 diference: 1.722
        # From what I've seen the frequency seen by WSPR is not these frequencies but the transmission
        # is decoded successfully.  Maybe because all the frequencies are shifted together the same amount.
        #
        PLED.toggle()
        
        SymbolNum = int(c)
        
        DIV  = 16										# was 8 now 16 
        NUM  = ChannelSigGenParams[2 * SymbolNum]     
        DENOM= ChannelSigGenParams[2 * SymbolNum + 1]
        #
        #eprint(f'Symbol: {SymbolNum} Multisynth D: {DIV} N: {NUM} D: {DENOM}')
        #eprint('APPROXIMATE Clock A Frequency(Hz): ', (780000000 / (DIV + NUM / DENOM)))  # no floating point co-processor so not accurate)
        #
        # PLL A is at 780 MHz.  See setupPLL in the init function.
        # The output current is set to 8mA. (3.3 volts peak to peak documented? but 1.7 volts on the scope)
        # so 1.13 volts RMS I guess and that leads to 9 mW ~ 9.5 dBm
        #
        #print('symbol: ', SymbolNum)
        SI5351.SetupMultisynth(TxI2C, 0, DIV, NUM, DENOM)
        SI5351.SetupMultisynth(TxI2C, 1, DIV, NUM, DENOM)
        #
        # If this is the first time through the loop we turn on the clock outputs after we have the frequency of the first
        # symbol set.  We leave it on until the entire set of symbols is transmitted.
        if i == 0:
            TxTimeStart = time.ticks_ms()		# So we can print the total transmit time at the end.
            SI5351.EnableClocks0and1(TxI2C, 1)	# we should probably set this for iteration 0 in the for loop so the frequency
                                                # for the first symbol is set before we turn on the outputs.
            RTCTime = rtc.datetime()			# Get the current time so we can print it.
            eprint(f'TX ON @ RTCTime (mm:ss): {RTCTime[5]}:{RTCTime[6]}  Transmit Start Time')
        
        #
        # I'm figuring the deadline by transmitting the full set of symbols and adjusting it to get 110,600 ms total.
        #
        deadline = time.ticks_add(time.ticks_us(), 671000)  # now 671 000 ~ 110,606 ms total.
        #
        # wait for the transmit time to elapse 0.683 seconds per symbol equivalent to 1.46 Hz.
        #
        while time.ticks_diff(deadline, time.ticks_us()) > 0:
            time.sleep_us(100)    #100 microseconds 0.000 100 s
        
    SI5351.EnableClocks0and1(TxI2C, 0)			# Transmitter Off

    TxTimeEnd = time.ticks_ms()				# Total transmit time.

    eprint(f'TX OFF Symbols Sent: {len(SymbolArray)} in {time.ticks_diff(TxTimeEnd, TxTimeStart)} ms')
           
    return
#
# Print the approximate frequencies for symbols 0,1,2, and 3.  They are not very accurate say at the 1s and hundreths
# but it serves as a gut check and the values for calculating it on a calculator are printed as well if more
# accurate results are desired.
#
def TX_PrintTargetFrequencies(PLLFreq, ChannelSigGenParams):

    Fraction = 0.0
    Frequency = 0.0
    
    for i in range( 0, 4, 1):
        DIV  = 16 
        NUM  = ChannelSigGenParams[2 * i]     
        DENOM= ChannelSigGenParams[2 * i + 1]
        # no floating point co-processor so not accurate but can be a gut check. (780000000 / (DIV + (NUM / DENOM))
        Fraction = (NUM/DENOM)
        Fraction = Fraction + DIV
        Frequency = PLLFreq/Fraction
        eprint(f'Clock A Frequency for symbol {i} = {PLLFreq} / ({DIV} + {NUM} / {DENOM}) ~ {Frequency} Hz (not real accurate)')

    return
#
# It seemed like this might have been needed but I haven't found a use for it yet.
# 
def TX_DeInit():
    #
    # This has to be done to shut the transmitter off
    # Haven't discovered what to put here.
    
    return

# End Of code for transmitting WSPR ***************************************
#
# This gets the voltage we'll send in the WSPR message in slot 1 (second msg)
#
def PicoReadVSYS():
    # ToDo: check this for proper conversion.  What is the max the
    # ADC can convert.  This code is based on 3.3 volts but seems to be ok
    # when plugged into USB its reading 4.86
     
    # Conversion factor for ADC (12-bit resolution scaled to 16-bit)
    conversion_factor = 3.3 * 3 / 65535

    # Configure GPIO25 as output and set it high
    Pin(25, mode=Pin.OUT).value(1)

    # Configure GPIO29 as input with no pull resistors
    Pin(29, mode=Pin.IN)

    # Read ADC value from GPIO29
    vsys_adc = ADC(29)
    vsys_voltage = vsys_adc.read_u16() * conversion_factor

    return round(vsys_voltage, 2)
#
# Read the on-board temp sensor.  According to the documentation this is
# particularly sensitive to the voltage so it may not be very accurate.
# probably not an issue for our use with the pico balloon but if another
# temp sensor is available it should be used in place of this.
#
def PicoReadTemp():
    # Create ADC object for internal temperature sensor (ADC4)
    sensor_temp = ADC(4)

    # Conversion factor for 12-bit ADC to voltage
    CONVERSION_FACTOR = 3.3 / 65535

    # Read raw ADC value
    reading = sensor_temp.read_u16()
        
    # Convert ADC reading to voltage
    voltage = reading * CONVERSION_FACTOR
        
    # Convert voltage to temperature in Celsius
    # Formula from Raspberry Pi Pico datasheet:
    # Temp(C) = 27 - (V - 0.706)/0.001721
    temperature_c = 27 - (voltage - 0.706) / 0.001721
        
    # Print temperature
    #print("Temperature: {:.2f} °C".format(temperature_c))
    
    return round(temperature_c, 2)

#
# Use LED_ON, or LED_OFF
def PicoLED(OnOff):
    LED = Pin("LED", Pin.OUT)
    
    if (OnOff <= 0):
        LED.value(0)  # LED ON
    else :
        LED.value(1)  # LED ON
    return LED

#
#*******************************************************************************
#
# main
#0123456789012345678901234567890123456789012345678901234567890123456789012345678
#
Error = 0
PLLFreq = 0
HaveGPSData = False				# True if have received a parsable message from the GPS
TxMtrWarmUp = False				# True when the SI5351 and TCXO have been turned on the warm up before transmitting
HaveGGA = False                 # GPS NMEA Message GGA: Time,Position, and fix related data.  True when valid
HaveRMC = False					# GPS NMEA Message RMC: Position, velocity, time.  True when valid.
SlotZeroComplete = False		# Slot 0 First Message has been Sent = True
SlotOneComplete = False			# Slot 1 Second Message has been Sent = True
SlotTwoComplete = False			# Slot 2 Third Message has been Sent = True
SlotThreeComplete = False		# Slot 3 Fourth Message has been Sent = True
GPSState = GPS_OFF				# GPS power is on or off
NMEADataBuffer = bytearray()	# place to put the GPS NMEA messages.
FatalError = 0					# used to accumulate errors to show the pico ballon pilot something
                                #   needs to be corrected BEFORE flight.  Could be contants or hardware
                                #   higher numbers just mean more failures.  Once flying it has no use.

machine.freq(48000000)   # CPU clock speed = 48 MHz
eprint('CPU clock speed = ', machine.freq())

eprint('voltage: ', PicoReadVSYS())
eprint('temp (°C): ', PicoReadTemp())

# The clock is used to time the sending of the WSPR messages after the GPS is
# turned off.  The GPS is used to set the clock.
rtc = machine.RTC()

PLED = PicoLED(LED_ON)
#
# Initialize our sensor
Error, SEN0501_I2C = SEN0501.SEN0501_Init()
if (Error != 0):
    FatalError += 1  # Fatal errors just add up. The run loop is skipped if above 0
    print('FATAL Error SEN0501 not found')
#
# Power Cycle the transmitter (in case we are rebooting)
#
TX_Power(TX_OFF)
time.sleep(0.5)    # short pause to let everthing settle.
#
# Turn the power on so we can check if the transmitter responds.
TX_Power(TX_ON)
time.sleep(0.5)    # short pause to let everthing settle.
#
# The SI5351 is on I2C0
#
TxI2C = I2C(0, scl=Pin(5), sda=Pin(4), freq=100000)

Error = SI5351.CheckI2CforSI5351(TxI2C)
if (Error != 0):
    FatalError += 1
    eprint('FATAL ERROR: SI5351 not found')
#
# Format our callsign properly for WSPR message for Slot0
Error, FormattedCallsign = WSPR_FormatCallsign(MY_CALLSIGN)
if (Error != 0):
    FatalError += 1
    eprint('FATAL ERROR: Callsign can not be formatted properly')
#    
# check if global MY_BAND is formatted properly
Error, MY_BAND = TRAQUITO_CheckBandString(MY_BAND)
if (Error != 0):
    FatalError += 1
    eprint(f'FATAL ERROR ({Error}): Invalid band: {MY_BAND}')
#
# Check that the channel is valid
Error = TRAQUITO_CheckChannel(MY_CHANNEL)
if (Error != 0):
    FatalError += 1
    eprint('FATAL ERROR: Channel not in range (0-599): ', MY_CHANNEL)
#
# Get our start time minute.  You can see what it should be on the channel map on the
# Traquito web site.
Error, ChannelStartTime = TRAQUITO_GetChannelTime(MY_CHANNEL)    
if (Error != 0):
    FatalError += 1
    eprint('FATAL ERROR: Can not calculate channel start time for channel: ', MY_CHANNEL)
#
# This determines what the four frequenies will be during transmit based on
# Channel and lane.
Error, ChannelSigGenParams = TRAQUITO_CalculateChannelParams(MY_CHANNEL)
if (Error != 0):
    FatalError += 1
    eprint('FATAL ERROR: Can not calculate signal generation parameters')

eprint(f'Band {MY_BAND} Channel {MY_CHANNEL} Start Time {ChannelStartTime}')

PLLFreq, Error = TX_Init(TxI2C, ChannelSigGenParams)
if (Error != 0):
    FatalError += 1
    eprint('FATAL ERROR: SI5351 Initialize failed')
    
TX_PrintTargetFrequencies(PLLFreq, ChannelSigGenParams)  # Print the frequencies for the symbols - its approximate.

TX_Power(TX_OFF) # Turn the transmitter back off
#
# Done with Transmitter and WSPR Stuff.
#
uart1 = ATGM336H.ATGM336H_GPSInit()	# Get the GPS ready to go.
GPSState = GPS_ON
#
# if there was some fatal error above running the code won't work so we blink the LED in
# morse code to indicate the number of problems (see the else case) in the event no
# terminal is hooked up or debug messages are off.
#
if FatalError == 0:
    #
    # The basics of the main event loop are:
    # get a valid gps message so we have position and time of day
    # when we are 30  seconds from transmit time turn off the GPS and turn on the transmitter to let it warm up
    #     and prepare teh data for the first messge
    # at the specific minute + 1 second transmit the first message
    # at completion prepare the next message data
    # at the specific minute + 2 + 1 second transmit the second message
    # at completion prepare the next message data
    # at the specific minute + 4 + 1 second transmit the third message.
    # at completion of the third message turn the transmitter off and the GPS on.  Reset all the state flags
    #     and repeat the process.
    #
    while True:                   # loop forever
        if GPSState is GPS_ON:
            HaveMessage = False
            PLED.toggle()
            # Get a valid GPS reading.  We need this for time of day so we can know when to
            # transmit so nothing can really be done until this completes.
            NMEADataBuffer, HaveMessage = ATGM336H.ATGM336H_GPSGetUARTData(uart1, NMEADataBuffer)
            #print(f'NMEADataBuffer: {NMEADataBuffer} HaveMessage: {HaveMessage}')
        
            #TODO: If there is no fix for a long time perhaps reboot the pico ?  or power cycle the GPS or both ?
            # The GPS sends about 1 message per second.
        
            if HaveMessage is True:
            
                #print('GPS Message: ', NMEADataBuffer.decode('ASCII'))
                # parse messsage
                Type, ValidFix, Quality, TimeHH, TimeMM, TimeSSsss, MDNHGrid, Altitude, Speed, DateDay, DateMonth, DateYear = ATGM336H.ATGM336H_GPSGetNMEASentence(NMEADataBuffer)
            
            
                # ToDo: Only GGA Gets altitude so it is written over by RMC as 0.  Need to store based on type so we don't zero it out.
                
                if (Quality > 0) or (ValidFix == True): 
                    eprint(f'Type: {Type} ValidFix: {ValidFix} Quality: {Quality} Time:{TimeHH}:{TimeMM}:{TimeSSsss} MDNHGrid: {MDNHGrid} Altitude: {Altitude} Speed: {Speed} Date: {DateDay}-{DateMonth}-{DateYear}')
                    if 'GGA' in Type:
                        GGAAltitude = Altitude  # Save the altitude for sending since the RMC will overwrite it with 0.
                        HaveGGA = True
                    if 'RMC' in Type:
                        RMCSpeed = Speed    # Save the speed because GGA doesn't have speed and will overwrite with 0.
                        HaveRMC = True
                    if HaveRMC is True and HaveGGA is True:
                        # check to see if we are in a restricted area.  If so just keep on watching the GPS until we are out
                        if ATGM336H.inGeofence(MDNHGrid) is False:
                            # Since the GPS will be off we'll set the Pico's clock so we can time our transmissions.
                            WeekDay = time.localtime(time.mktime((DateYear, DateMonth, DateDay, 0,0,0,0,0)))[6] # Monday = 0, Sunday = 6
                            DateTime = (2000 + DateYear, DateMonth, DateDay, WeekDay, TimeHH, TimeMM, int(TimeSSsss), 0)
                            rtc.datetime(DateTime)    # Set the Pico's clock
                            HaveGPSData = True
                #
                # clear the buffer
                NMEADataBuffer[:] = b""
        
        
        # End of if GPSState is GPS_ON:
        #
        # Once we have all the GPS data we need from both message types then we need to look for a time to send a message.
        # The WSPR message should be sent on a specific even minute + 1 second (mm:ss 02:01). 30 seconds before the time to send
        # we will shut off the GPS power (the hotstart power GPS VBAT will still be on) and turn on the Transmitter.
        #
        if (HaveGPSData is True) and (TxMtrWarmUp is False):
            #
            #Check the time of day for an odd minute and between 28 and 30 seconds.  Then shut down the GPS and turn on the
            # transmitter to warm it up.
            #  Odd minute (ChannelStartTime - 1)
            if  (TimeMM % 10 == (ChannelStartTime - 1)) and (28 <= TimeSSsss <= 30) :
                eprint(f'GPSTime (mm:ss): {TimeMM}:{TimeSSsss}  GPS OFF  Transmitter ON (warmup ~ 30s)')
                
                ATGM336H.ATGM336H_GPSPower(GPS_OFF)
                GPSState = GPS_OFF
                
                # Power up the transmitter and initialize it
                TX_Power(TX_ON)
                PLLFreq, Error = TX_Init(TxI2C, ChannelSigGenParams)
                eprint('TX ON: WARMUP')
                #
                # This will create the 4FSK symbols for the Type 1 Message (slot 0)
                # which will be transmitted when our time comes up (next state)
                #
                WSPR_Symbols = WSPR_Create4FSKSymbols(FormattedCallsign, MDNHGrid, MY_POWER)
                eprint(f'Slot 0: {FormattedCallsign} {MDNHGrid} {MY_POWER}')
                
                HaveGPSData = False							# set to false since the GPS is now off and we won't collect until after the transmit cycle.
                TxMtrWarmUp = True           
        #
        # Now the GPS is no longer fetching time data so we use the RTC that we set using the GPS Time
        # Wait until appointed time after the transmitter is warmed up and send the message.
        #
        if (TxMtrWarmUp is True) and (SlotZeroComplete is False):
            # Wait until the even minute + 1 second then transmit the first message
            # if (RTCTime[5] % 10 == ChannelStartTime) and RTCTime[6] >= 1:
            RTCTime = rtc.datetime()
            if (RTCTime[5] % 10 == ChannelStartTime) and RTCTime[6] >= 1:
                RTCTime = rtc.datetime()
                eprint(f'RTCTime (mm:ss): {RTCTime[5]}:{RTCTime[6]}  Transmitting Message Slot 0')
                
                # Transmit Slot 0
                # The transmit function is blocking until the transmission completes
                TX_WSPRSymbols(TxI2C, WSPR_Symbols, ChannelSigGenParams)
                
                eprint("Transmit Slot 0 Complete")
                #
                # Prepare for next slot: This Will Encode the Basic Telemetry Message (Slot 1)
                # MY_CHANNEL, MY_GRID[4:6], MY_ALT, MY_TEMPERATURE, MY_VOLTAGE*1000, MY_SPEED, True
                # -50 to 39 C, 3000 to 4950 mV,  0-82 knots, 0-21340 meters
                PicoMilliVolts = int(PicoReadVSYS() * 1000)
                Temp = SEN0501.SEN0501_ReadTemp(SEN0501_I2C)
                eprint(f'Slot 1 Data: Channel: {MY_CHANNEL} Grid56: {MDNHGrid[4:6]} Alt(m): {GGAAltitude} Temp(C): {Temp} MilliV: {PicoMilliVolts} Speed(knots): {RMCSpeed}')
                Slot1Callsign,  MDNHGridString, Power = WSPR_EncodeBasicTelemetry(MY_CHANNEL, MDNHGrid[4:6], GGAAltitude, Temp, PicoMilliVolts, RMCSpeed, True)
                eprint(f'Slot 1: {Slot1Callsign} {MDNHGridString} {Power}')
                WSPR_Symbols = WSPR_Create4FSKSymbols(Slot1Callsign, MDNHGridString, Power)
                
                SlotZeroComplete = True
        
        #
        # Once slot 0 is complete we will wait until the time to send Slot 1
        #
        if (SlotZeroComplete is True) and (SlotOneComplete is False) :
            RTCTime = rtc.datetime()
            if (RTCTime[5] % 10 == ((ChannelStartTime + 2) % 10)) and RTCTime[6] >= 1:
                RTCTime = rtc.datetime()
                eprint(f'RTCTime (mm:ss): {RTCTime[5]}:{RTCTime[6]}  Transmitting Message Slot 1')
                
                # Transmit Basic Telemetry (Slot 1)
                # The transmit function is blocking until the transmission completes
                TX_WSPRSymbols(TxI2C, WSPR_Symbols, ChannelSigGenParams)
                
                eprint("Transmit Slot 1 Complete")
                # Done with Slot 1
                # Get Slot 2 Data Ready
                Humidity = SEN0501.SEN0501_ReadHumidity(SEN0501_I2C)
                #now try ultraviolet with the LTR390-UV-01 sensor (V2 of the board)
                UV = SEN0501.SEN0501_ReadUV(SEN0501_I2C)
                #now try luminosity VEML7700
                Luminosity = SEN0501.SEN0501_ReadLuminosity(SEN0501_I2C)
                #now try atmospheric pressure BMP280
                Pressure = SEN0501.SEN0501_ReadAtmosphericPres(SEN0501_I2C)
                
                eprint(f'Slot 2 Data: Humid(%) {Humidity} UV(mw/cm^2) {UV} Luminosity(klx) {Luminosity} Pressure(mBar) {Pressure}')
                # Get our sensor measurements and encode them to the BigNumber, then Encode that in the Extended
                # Telemetry message, then create symbols to transmit.
                BigNumber = WSPR_ExtTel1EncodeMeasurements(Humidity, UV, Luminosity, Pressure)
                Slot2Callsign, MDNHGridString, Power = WSPR_EncodeExtendedTelemetry(MY_CHANNEL, 2, BigNumber)
                print(f'Slot 2: {Slot2Callsign} {MDNHGridString} {Power}')
                WSPR_Symbols = WSPR_Create4FSKSymbols(Slot2Callsign, MDNHGridString, Power)
                # Ready to transmit Slot 2
                
                SlotOneComplete = True
        #
        # Slot 1 is complete we will wait until its time to send Slot 2
        #
        if (SlotTwoComplete is False) and (SlotOneComplete is True):
            RTCTime = rtc.datetime()
            if (RTCTime[5] % 10 == ((ChannelStartTime + 4) % 10)) and RTCTime[6] >= 1:
                RTCTime = rtc.datetime()
                eprint(f'RTCTime (mm:ss): {RTCTime[5]}:{RTCTime[6]}  Transmitting Message Slot 2')   
        
                # Transmit Slot 2
                # The transmit function is blocking until the transmission completes
                TX_WSPRSymbols(TxI2C, WSPR_Symbols, ChannelSigGenParams)
                
                eprint("Transmit Slot 2 Complete")
                # Get our data ready for Slot 3
                # Get our sensor measurements and encode them to the BigNumber, then Encode that in the Extended
                # Telemetry message, then create symbols to transmit.
                #
                eprint(f'Slot 3 Data: Extended Speed (MPH): {Speed}')
                BigNumber = WSPR_ExtTel2EncodeMeasurements(Speed)   # Expects speed in knots converts to MPH
                Slot3Callsign, MDNHGridString, Power = WSPR_EncodeExtendedTelemetry(MY_CHANNEL, 3, BigNumber)
                print(f'Slot 3: {Slot3Callsign} {MDNHGridString} {Power}')
                WSPR_Symbols = WSPR_Create4FSKSymbols(Slot3Callsign, MDNHGridString, Power)
                # Ready to transmit Slot 3
        
                SlotTwoComplete = True     
        #
        # Slot 2 is complete we will wait until its time to send Slot 3
        #
        if (SlotThreeComplete is False) and (SlotTwoComplete is True):
            RTCTime = rtc.datetime()
            if (RTCTime[5] % 10 == ((ChannelStartTime + 6) % 10)) and RTCTime[6] >= 1:
                RTCTime = rtc.datetime()
                eprint(f'RTCTime (mm:ss): {RTCTime[5]}:{RTCTime[6]}  Transmitting Message Slot 3')   
        
                # Transmit Slot 3
                # The transmit function is blocking until the transmission completes
                TX_WSPRSymbols(TxI2C, WSPR_Symbols, ChannelSigGenParams)       
        
                eprint("Transmit Slot 3 Complete")
                SlotThreeComplete = True        
                #
                #***********************************************************************************    
                # Reset the State Machine and Turn the Transmitter Off and the GPS On
                #
                eprint("All Transmissions Completed.  Resetting-Waiting for next TX time.")
                TX_Power(TX_OFF)			# Turn off the Transmitter and TCXO
                
                RTCTime = rtc.datetime()
                eprint(f'RTCTime (mm:ss): {RTCTime[5]}:{RTCTime[6]}  Transmitter OFF  GPS ON')
                HaveGPSData = False
                TxMtrWarmUp = False
                HaveGGA = False
                HaveRMC = False
                SlotZeroComplete = False
                SlotOneComplete = False
                SlotTwoComplete = False
                SlotThreeComplete = False
                
                ATGM336H.ATGM336H_GPSPower(GPS_ON)
                time.sleep(0.25) 		 # Wait a quarter second
                ATGM336H.ATGM336H_GPSNReset()     # Maybe not necessary but suggested by dmalnati
                time.sleep(1)			 # Wait a second for it to have reset
                GPSState = GPS_ON
            #
            # All done, repeat the process
            #
    
    # End of While True ******************************************************** 
else:
    # Fatal Error of some sort
    #
    eprint(f'FATAL ERROR(S): {FatalError}.  Must be corrected in order to run.')
    while True:
        # Blink the LED to indicate an error.  The following function blocks until
        # it is done sending the message so if something more need to run here
        # it may need to be removed.
        MorseLED.send_morse(f'ER {FatalError}')
        time.sleep_ms(100)


#0123456789012345678901234567890123456789012345678901234567890123456789012345678
# End Of File: WC8T_TJP.py