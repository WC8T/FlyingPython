# FlyingPython
Python for the Traquito Jet Pack &amp; Raspberry Pi Pico Ballon Transmitter and Additional Sensors

See the Traquito site https://traquito.github.io/ This site is how we (local Ham club) got started pico
ballooning.  We used the software from the site and eventually after a lot of launches had some
success.  Then we wanted to run some alternate sensors and this project started.

After looking for code I stumbled into Craig Ivey's code which was the easiest for me to read even 
though I'm primarily a C programmer.  It is here: https://github.com/IveyWorks/OpenTJP.git  So I used it as a
starting point and began programming in Python.

So this first version of the code is to read the DFRobot SEN0501 Version 2

I changed a lot of stuff around amd seperated out some things to make it easier for me.  My wish is
to be able to re-code for whatever sensor I might want to add to the system and for others to do that too.
I also tried to add lots of comments to show where I found information and what the code is doing.  It needs
more still.  Summer is here though so I won't do much until winter is here and I'm back indoors.


POWER DRAW NOTES:

The idea is to see if the flexible MPT3.6-150 solar panels will work ok.  They work fine with the C code 
using diodes with just the traquito and RP Pico.  Rated output is 3.6 volts and 100mA although in the Idaho sun I often see 4.8 volts.
Our configuration has three solar panels in a sort of triangular arangement so hopefully one
catches the sun.  I still need to test without diodes to see if the draw from the shaded panels is a problem.  
There is a MPT4.8-150 - 4.8 volts at 100mA

The SEN0501 sensor draws very little of the power.

Test done on the bench with DC power supply.  RP Pico with Traquito and SEN0501 sensor using multimeter.

3.6 volts
GPS on, Transmitter off: 56mA (I'm using a powered GPS antenna so this is a little higher)

GPS off, Transmitter warmup: 79mA

GPS off, Transmitter transmitting: 111mA

4.8 volts
GPS on, Transmitter off: 46mA (I'm using a powered GPS antenna so this is a little higher)

GPS on, Transmitter warmup: 63mA

GPS on, Transmitter transmitting: 83mA

(The powered GPS antenna is drawing from a pin on the GPS as shown in the GPS datasheet.  This
allows for testing indoors on the bench with the antenna in a window.)

End of: README.md
