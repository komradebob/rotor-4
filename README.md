# rotor-4
Arduino Nano based GS-232 rotor controller

Control a G-{800,1000,2800}DXA rotor via an RS232 port. Initially based on the Lemington G-2800 code, but most of it is long gone, outside of the calibration routines. 

Hook up your rotor to the output of a 2n2222 connected to the output pins of the Nano, one pin for speed control, and off you go. 

Much work remains to be done.

  Remaining todo's:
  
  1) Convert the last of the if(debug) statments to #ifdefs
  2) Add elevaton control and El ADC to control Az/El rotors as well
  3) Clean up and use return variables rather than globals
  4) Modify to use all 450 degrees of Az rotation
  5) Move volts to degree conversion and read_adc to a single function
  6) Make motor speed selection persistant when changed
  7) Make tolerant of 2 digit heading input
  8) DONE - Clean up display routine to add leading zero
  9) Clean up the relay code to fully remove the L/R relays from the K commands
  10) Re-enable calibration switch - Bypassed it in SW to avoid opening the box again
  11) Review BRAKE code to control rotors with brakes
  12) Write OS X Client
  13) Write TCP/IP client
