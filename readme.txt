Freescale 9S12XDP512 Serial Monitor

Date: 12-January-2018

Version 2.03a as updated by Technological Arts (http://technologicalarts.ca) and my own change to allow the use of the FTDI USB UART.

Technological Arts corrected the problem with not being able to erase all of the flash memory.
I have commented out the line in the .asm file where it looked for the SCI0 RX line to be held HI to allow the user code to run.
Originally if the SCI0 RX line was held LO, then it would go into serial monitor mode instead of running the user code.

