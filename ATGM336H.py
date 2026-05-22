# ATGM336H.py
# functions for operating the GPS
# Paul Taylor 2026-01-22
#0123456789012345678901234567890123456789012345678901234567890123456789012345678
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

import machine
import math
from machine import ADC, Pin, I2C, UART
import time
global uart1

MY_DEBUG_PRINT_ENABLE = True

def eprint(*args):
    if MY_DEBUG_PRINT_ENABLE:
        for val in args:
            print(val, end=' ')
        print('')
 
GPS_ON = 1
GPS_OFF = 0

GEOFENCE_EXCL = ['PM27','PM28','PM29','PM37','PM38','PM39','PM48','PM49','PN20','PN30','PN31',
    'PN40','PN41','PN42','PN52','LK12','LK13','LK14','LK15','LK16','LK17','LK22','LK23','LK24','LK25',
    'LK26','LK27','LK33','LK34','LK35','LK36','LK37','LK43','LK44','LK45','LK46','LK47','LK48',
    'LK54','LK55','LK56','LK57','LK58','LK65','LK66','LK67','LK68','FM08','IO54','IO64','IO65','IO66',
    'IO67','IO68','IO70','IO71','IO72','IO73','IO74','IO75','IO76','IO77','IO78','IO80','IO81','IO82',
    'IO83','IO84','IO85','IO86','IO87','IO88','IO89','IO90','IO91','IO92','IO93','IO94','IO95','IO96',
    'IO97','JO00','JO01','JO02','JO03','JO04']
    # N.Korea PM27, 28, 29, 37, 38, 39, 48, 49   PN20, 30, 31, 40, 41, 42 52
    # Yemen LK12, 13, 14, 15, 16,  17, 22, 23, 24, 25, 26, 27, 33, 34, 35, 36, 37, 43, 44, 45,
    #    46, 47, 48, 54, 55, 56, 57, 58, 65, 66, 67, 68
    # NRQZ - National Radio Quiet Zone - Probably not since we are way above it but - FM08
    # UK IO54, 64, 65, 66,67,68, 70, 71, 72, 73, 74, 75, 76,77,78, 80, 81, 82, 83, 84, 85, 86,
    #    87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97,  JO00, 01, 02, 03, 04
         
def inGeofence(MdnhdGrid):
    #
    # check if the Grid List is a string and it is greater than 4 characters
    # just a safety in case something goofy happens to the grid string
    #
    if isinstance(MdnhdGrid, str) and len(MdnhdGrid) >= 4:
        # Get the first four characters of the Grid since that is all we
        # want to test and loop through the excluded grid squares to see
        # if there is a match
        #
        if MdnhdGrid[0:4] in GEOFENCE_EXCL:
            return True
            
            
    return False

#
# GPS init function to set up the GPS after a reboot or cold start of the
# Pico.  It appears that we can power off the GPS with GPS_VBAT on and
# we don't have the re-initialize it.
#
def ATGM336H_GPSInit():
    #
    # initialize the serial port to comunicate with the GPS
    # pico pin 11, 12.  GP8, GP9
    #
    uart1 = UART(1, baudrate=9600, tx=Pin(8), rx=Pin(9))
    #
    # Pico Pin 5, GP3
    # This allows the module to hot start (backs up SRAM and RTC on the GPS) 
    Pico5 = Pin(3, Pin.OUT)  # GPS VBAT, high = on On jetpack: GPS_V_BCKP
    Pico5.value(0)			 # Shut off VBAT in case there were issues we want complete restart
    time.sleep(0.5)			 # Wait a half second.
    Pico5.value(1)           # Turn it on.
    
    ATGM336H_GPSPower(GPS_ON)
    time.sleep(0.25) 		 # Wait a quarter second
    ATGM336H_GPSNReset()     # Maybe not necessary but suggested by dmalnati
    time.sleep(1)			 # Wait a second for it to have reset
    #
    # Set the GPS to Airborne mode with less than 1g : Max height 50,000m
    # max speed 100 m/s, max vertical speed 100 m/s, Max position error Large
    #
    uart1.write(bytearray(ATGM336H_GPSCreateNMEAString('$PCAS11,5'), 'utf-8'))
    #
    # Tell the GPS we only want RMC and GGA messages
    #
    uart1.write(bytearray(ATGM336H_GPSCreateNMEAString('$PCAS03,1,0,0,0,1,0,0,0'), 'utf-8'))
    return uart1
#
# Adding the checksum to the NMEA sentence
#
def ATGM336H_GPSCreateNMEAString(sentence):
    NMEA_Sentence = sentence + '*' + ATGM336H_CalcNMEAChecksum(sentence) + '\r\n'
    return NMEA_Sentence

def ATGM336H_CalcNMEAChecksum(sentence: str) -> str:
    """
    Calculate the NMEA-0183 checksum for a given sentence.
    Args:
        sentence (str): The NMEA sentence (with or without starting '$'/'!' and without checksum).
    Returns:
        str: Two-character hexadecimal checksum (uppercase).
    """
    if not sentence:
        raise ValueError("Empty sentence provided.")

    # Remove starting '$' or '!' if present
    if sentence[0] in ('$','!'):
        sentence = sentence[1:]

    # If sentence already contains '*', strip everything after it
    if '*' in sentence:
        sentence = sentence.split('*')[0]

    checksum = 0
    for char in sentence:
        checksum ^= ord(char)  # XOR each character's ASCII value

    return f"{checksum:02X}"  # Return as two-digit uppercase hex



def ATGM336H_GPSPower(OnOff):
    #
    # 1 is on and 0 is off or GPS_ON, GPS_OFF
    #
    # Pico Pin 4, GP2
    Pico4 = Pin(2, Pin.OUT)  # GPS, low = ON On jetpack diagram: GPS_LSWITCH
    
    if (OnOff <= 0):
        Pico4.value(1)   # 1 = Off
    else :
        Pico4.value(0)   # 0 = On
    return
 
def ATGM336H_GPSNReset():
    # Pico Pin 9, GP6
    Pico9 = Pin(6, Pin.OUT)  # GPS NRESET, low = reset On jetpack: GPS_RESET
    
    Pico9.value(0)   # 0 is reset
    time.sleep(0.25) # Wait a quarter second
    Pico9.value(1)   # 1 is not reset.
    return

def ATGM336H_GPSGetUARTData(uart1, sentence):
    HaveMessage = False
    DollarCount = 0
    byte = ''
    Error = 0
    
    OutSentence = bytearray()
    buffer = bytearray()
    #
    # check for bytes to read
    # if there are then read the count so as not to block
    # execution
    # read a character, check if its a /n,  if it is then complete a message if not store character in buffer
    #
    # The ATGM336H appears to send occasional sentences with out \r\n.  The GPTXT type.  So that leads to parsing
    # problems.  So check if we have a $ in the message and if we do count it.
    #
    if ('$' in sentence):
        DollarCount +=1
    
    count = uart1.any()
    
    if count > 0:
        buffer = uart1.read(1)
        try:
            byte = buffer.decode('ASCII', 'strict')
        except:
            eprint(' ')
            eprint('Error UART byte decode failed, deleting buffer and starting over')
            OutSentence[:] = b""
            Error = 1
        #
        # if there is another $ in the message we have two messages in the buffer so dump the buffer out and start
        # over.
        if ('$' in byte) and (DollarCount >= 1):
            # We have two messages in the buffer now so delete it and start over
            OutSentence[:] = b""
            eprint(' ')
            eprint('ERROR two $ found in message, deleting buffer and starting over')
            Error = 1
        
        if(Error == 0):
            if(('\n' in byte) or ('\r' in byte)):
                if('\n' in byte):
                    # for some reason messages arrive with a \n but have not body
                    if len(sentence) < 5 :
                        eprint('\n ERROR: String has less than 5 characters with linefeed')
                        print('UART Buffer has: ', count)
                    elif '*' not in sentence :
                        eprint('\n ERROR: No * in message (no checksum) but with linefeed')
                        print('UART Buffer has: ', count)
                    elif '$' not in sentence :
                        eprint('\n ERROR: No $ found in message (no start of message) with linefeed')
                        print('UART Buffer has: ', count)
                    else:
                        # completed message
                        HaveMessage = True
                        OutSentence = sentence
                else:
                    OutSentence = sentence
            else:
                OutSentence = sentence + buffer
        
    return OutSentence, HaveMessage


def ATGM336H_GPSGetNMEASentence(NMEADataBuffer):
    ValidFix = False
    Quality = 0
    TimeHH = -1
    TimeMM = -1
    TimeSS = -1
    TimeSSsss = -1.0
    Latitude = 0.0
    Longitude = 0.0
    Altitude = 0.0
    Speed = 0.0
    DateDay = 0
    DateMonth = 0
    DateYear = 0
    MDNHGrid = 'XX00XX'
    Type = 'xxx'
    # we're expecting a complete message from $ to checksum without \r\n
    #decode into character data
    NMEADataString = NMEADataBuffer.decode('ASCII')
    #
    #eprint('NMEADataString: ', NMEADataString)
    
    #
    # check the checksum of our message which is after the *
    # split the data string into two with before and after the *
    ChkSplitString = NMEADataString.split('*')
    
    #print(f'ChkSplitString[0]: {ChkSplitString[0]} ChkSplitString[1]: {ChkSplitString[1]}')
    
    if len(ChkSplitString[1]) != 2 :
            eprint('Error: Checksum not 2 digits: ', ChkSplitString[1])
            ChkSplitString[0] = ''  # Blank the string to prevent the rest of the code from executing
    else :
            ChkSum = ATGM336H_CalcNMEAChecksum(ChkSplitString[0])
            if ChkSum != ChkSplitString[1]:
                eprint(f'Error: Checksum does not match: {ChkSum} Calculated: {ChkSplitString[1]}')
                ChkSplitString[0] = ''  # Blank the string to prevent the rest of the code from executing
    
    # ToDo: decide what to do with invalid checksums.  Original author hints there is an issue with
    # them but his checksum calculator had an issue.
    #
    # Next process RMS and GGA messages.  We need both to get all the data we want: Time, Position
    # Speed, and Altitude.  Even though we only asked for these two we still get GPTXT and possibly
    # others so we need to check for those specific messages.
    # There are two characters between the $ and the RMC or GGA.  They tell the satelite network being used
    # so they can change.
    
    # Process RMC Message
    if 'RMC' in ChkSplitString[0]:
        # break the message appart into its comma seperated fields.
        # 
        # GNRMC,005857.000,A,4337.82028,N,11559.09366,W,1.08,0.00,170126,,,E    
        #   0       1      2     3      4      5      6   7    8     9   10 11 12 
        RMCMsgParts = ChkSplitString[0].split(',')
        
        if RMCMsgParts[2] == 'A':		# No use parsing if the data isn't valid = A
            ValidFix = True
            
            Type = 'RMC'
            TimeHH = int(RMCMsgParts[1][0:2])
            TimeMM = int(RMCMsgParts[1][2:4])
            TimeSSsss = float(RMCMsgParts[1][4:])
            TimeSS = int(RMCMsgParts[1][4:6])
            
            # Latitude is in the form ddmm.mm so seperate the dd from the mm.mm then
            # since mm.mm is in 60ths of a degree we divide by 60 to get decimal degrees
            # and put it back together as a float.
            Latitude = int(RMCMsgParts[3][0:2]) + float(RMCMsgParts[3][2:])/60
            
            if RMCMsgParts[4] == 'S':
                Latitude = -Latitude		# our equations work with +/- 
            # Longitude is in the form dddmm.mm so sperate the ddd from the mm.mm then
            # since mm.mm is in 60ths of a degree divide by 60 to get hundreths of a degree
            # (decimal degrees) and then put it back together as a float
            Longitude = int(RMCMsgParts[5][0:3]) + float(RMCMsgParts[5][3:])/60
            if RMCMsgParts[6] == 'W':
                Longitude = -Longitude      # our equations work with +/-

            Speed = float(RMCMsgParts[7])     # Speed knots
            
            DateDay = int(RMCMsgParts[9][0:2])
            DateMonth = int(RMCMsgParts[9][2:4])
            DateYear = int(RMCMsgParts[9][4:])
        
    # Process GGA Message
    if 'GGA' in ChkSplitString[0]:
        # break the message appart into its comma seperated fields.
        # 
        # $GNGGA,005857.000,4337.82028,N,11559.09366,W,6,04,4.7,1004.3,M,0.0,M,,
        #    0        1         2      3       4     5 6  7  8     9   10 11 12 13 14
        GGAMsgParts = ChkSplitString[0].split(',')
        
        Quality = int(GGAMsgParts[6])
        
        if(Quality > 0):							# No use parsing if Q is bad =0
        
            Type = 'GGA'
            TimeHH = int(GGAMsgParts[1][0:2])
            TimeMM = int(GGAMsgParts[1][2:4])
            TimeSSsss = float(GGAMsgParts[1][4:])
        
            # Latitude is in the form ddmm.mm so seperate the dd from the mm.mm then
            # since mm.mm is in 60ths of a degree we divide by 60 to get decimal degrees
            # and put it back together as a float.
            Latitude = int(GGAMsgParts[2][0:2]) + float(GGAMsgParts[2][2:])/60
            if GGAMsgParts[3] == 'S':
                Latitude = -Latitude				# our equations work with +/-
                
            # Longitude is in the form dddmm.mm so sperate the ddd from the mm.mm then
            # since mm.mm is in 60ths of a degree divide by 60 to get hundreths of a degree
            # (decimal degrees) and then put it back together as a float   
            Longitude = int(GGAMsgParts[4][0:3]) + float(GGAMsgParts[4][3:])/60
            if GGAMsgParts[5] == 'W':
                Longitude = -Longitude				# our equations work with +/-
            
            Altitude = float(GGAMsgParts[9])     # Altitude in meters above Mean Sea Level
        
    if(ValidFix == True) or (Quality > 0):
        #eprint(f'Latitude {Latitude} Longitude {Longitude}')
        MDNHGrid = convertLatLonToGridSquare(Latitude, Longitude)
        
    return Type, ValidFix, Quality, TimeHH, TimeMM, TimeSSsss, MDNHGrid, Altitude, Speed, DateDay, DateMonth, DateYear
#
# Convert Lat and Lon in +/- float format to Maidenhead Grid Square format
#
def convertLatLonToGridSquare(Lat, Lon):
    #
    # Check the wikipedia articale on maidenhead grid for a very good
    # explanation of the system.
    # There is a QST article: January of 1989 pp: 29-30, 43 that talk
    # about converting.
    # -90 <= Lat < 90 and -180 <= Lon < 180  I don't know if the GPS
    # would eveer produce exactly 180 or 90.
    if (180.0 <= Lon):
        Lon = 179.999
    if (90.0 <= Lat):
        Lat = 89.999
    #
    # Reference our Lon and Lat to the starting point of the grid AA so
    # we can calculate our position from there.
    #
    LonStart = Lon + 180.0
    LatStart = Lat + 90.0
    #
    # Get the Field Longitude is first.  There are 18 zones of 20 degrees
    # each.   Letters A through R.((-180 + 180)/18 = 0, 0=A. (0 + 180)/20 = 9, 0=I.
    # (180 + 180)/20 = 18, 18 = R
    # ord() just gets the number of character A so we add to that to find our char.
    #
    GridLonField = chr(int(LonStart/20) + ord('A'))
    #
    # Next figure out the first number (3rd position in the locater)
    # These are numbers from 0 to 9 which is 2 degrees of longitude
    #
    GridLonSquare = math.floor((LonStart % 20) / 2)
    #
    # Last figure out the first letter of the subsquare (5th position
    # in locater.  A through X which is 5 minutes of longitude
    #
    GridLonSubSquare = chr(math.floor(((LonStart % 20) % 2) * 12) + ord('A'))
    #
    # Get the Lattitude Field next.  It is the second letter.  Again there are
    # 18 zones but of 10 degrees each.  Letters A through R.
    #
    GridLatField = chr(int(LatStart/10) + ord('A'))
    #
    # Next figure out the second number (4th position in the locater)
    # These are numbers from 0 to 9 which is 1 degree of latitude
    #
    GridLatSquare = math.floor((LatStart % 10))
    #
    # Next figure out the next character (6th position in the locater)
    # These are letters from A to X which are 2.5 minutes of lattitude
    #
    GridLatSubSquare = chr(math.floor(((LatStart % 10) - GridLatSquare) * 24) + ord('A'))

    #eprint('MyGrid: ', GridLonField + GridLatField + str(GridLonSquare) + str(GridLatSquare) + GridLonSubSquare + GridLatSubSquare)
    
    return GridLonField + GridLatField + str(GridLonSquare) + str(GridLatSquare) + GridLonSubSquare + GridLatSubSquare


# End Of File: ATGM336H.py