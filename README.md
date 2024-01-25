
Nixie clock code

1/28/21

The current version of this clock software is 0.9.6

The clock can display hour and minutes, minutes and seconds, temperature,
and barometric pressure corrected for altitude. The digits can also be 
switched off. These five display modes are controlled with a rotary encoder.
The rotary encoder can also be used to alter the digit intensity, the tube
LED color and intensity. To change those values, press the rotary encoder
briefly; a decimal point in the left-most tube will light indicating the clock
is in the intensity/color set mode. The time can be set using a long press
of the rotary encoder. This mode is indicated by two decimal points bracketing
the tubes (hours or minutes) that are going to be set. In all of the 'set' 
modes, rotate the encoder to change the value.

The time is kept using a DS3231 clock chip with a battery backup. See the code
for ways to set the time when the code is built.

Temperature and barometric pressure are measured using a XXX sensor.

The clock uses an Arduino Pro Mini and the display tubes use exixe modules
from Tindie (which use an STM32 processor that reads commands from the SPI
bus).

The clock uses the Arduino EEPROM to store values for the intensity and
color values so they will be saved even if power is removed.

Version 0.9.6: Fixed the time set modes so that they don't use the digit fades.
That was making the display tediously slow while trying to set the time. Also 
removed the compiled in time of 1638; now used the time of the clock which can
be maintained by the battery.

12/18/21
