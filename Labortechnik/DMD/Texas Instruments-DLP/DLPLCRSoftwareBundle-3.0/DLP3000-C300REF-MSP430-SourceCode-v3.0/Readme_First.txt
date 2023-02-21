-----------------------------------------------------------------------------------------------------------------
-----------------------------------------------------------------------------------------------------------------
Copyright © 2013-2014 Texas Instruments Incorporated - http://www.ti.com/        
-----------------------------------------------------------------------------------------------------------------
-----------------------------------------------------------------------------------------------------------------
Product name and version number: DLP LightCrafter MSP430 Software v3.0
Company name: Texas Instruments

-----------------------------------------------------------------------------------------------------------------    
BUG FIXES
-----------------------------------------------------------------------------------------------------------------
Fixed the bug which caused the LED Overvoltage shutdown protection mode to occur when the device was powered down
Fixed the bug which would cause shutdown protection mode because of power supply transients

-----------------------------------------------------------------------------------------------------------------
FEATURES AND OPERATION SEQUENCE
-----------------------------------------------------------------------------------------------------------------
* Power-up and Power-down (with mirror park) sequence necessary for DLPC300 

	Upon power-up, the following sequence should occur:
	1) PWREN_CORE drives high
	2) 120msec later, PWREN_INTF drives high
	3) 100msec later, SYSRESETZ is released. Then, RESETZ is released.
	    Assert POWERGOOD with LEDs disabled
	4) The MSP430 waits for INIT_DONE_DPP to go high for a max of 500msec
	5) The MSP430 waits for INIT_DONE_DPP to go low for a max of 500msec
	6) MSP430 ensures to releases DM365 reset (SYSRESETZ) and then drives 
	    SYS_PWRONZ low to turn on the D507 LED
	7) I2C bus is initialized for 100KHz clock rate, issuing the following commands to DLPC300:
	    -unpark the DMD
	    -set revision register to MSP SW revision
	    -enable HW test point: GPIO[4:0]_TSTPT[4:0], IRW_SYNC_TSTPT_7, IRW_EN_TSTPT_6, TXD_TSTPT_0
	    -set input data to RGB565
	    -set resolution to QWVGA landscape 
	8)  MSP430 drives LED_ENABLEZ low
	9)  MSP430 issues an I2C command to DLPC300 with SHORT_FLIP and LONG_FLIP
	10) MSP430 loads and leaves first splash screen for 5 seconds
	11) MSP430 loads and leaves second splash screen for 5 seconds
	12) MSP430 loads and leaves third splash screen for 5 seconds
	13) MSP430 loads and leaves fourth splash screen for 5 seconds
	14) Measure Vref2 15 back-to-back times, compute average
	15) Measure DMD Voltage 15 back-to-back times, compute average

	The shutdown sequence is:
	1) MSP430 issues I2C command to shutdown LEDs
	2) MSP430 issues I2C command to park DMD
	3) MSP430 drives SYS_PWRONZ high to turn off the D507 LED
	4) MSP430 drives LED_ENABLEZ high and POWERGOOD low
	5) 500usec later, MSP430 holds DM365 in reset. 
	6) MSP430 sets all I/Os connected to DM365 to input. 
	7) MSP430 turns off DM365 3.3V supply and drives RESETZ low
	6) 100msec later, MSP430 drives PWREN_INTF and PWREN_CORE low

* Timer expires every 65 msec
* DMD thermistor is checked every 65msec. If DMD temperature is above 70C,
   system shutdowns and system fault LED is set to RED
* Button press turns on system. A second button press turns off system. 
   When system turns on, the LED driver sense voltage is checked to be 
   under 1.46V. Typical value is 0.96V
* When HDMI input is selected and no HDMI Sync signal is detected, LEDs 
   will be turned off after 2.24 sec
* When PROG_EN_DM is high, DM365 is requesting a firmware upgrade.
       1) MSP430 drives TX high
       2) MSP430 places DLPC300 in reset
       3) MSP430 ensures that BSL_RX_MSP is an input and sets BSL_TX_MSP and SPIPROGZ as output
       4) MSP430 drives SPIPROGZ low to connect SPI from DM365 to DLPC300
       5) Wait until PROG_EN_DM is lowered
       6) MSP430 releases DLPC300 from reset and stops driving SPIPROGZ
       7) IF BSL_RX_MSP caused and interrupt, MSP430 jumps into the Boostrap loader, otherwise MSP430 sets BSL_TX_MSP as input
* To activate FAN(J8 on driver board), set FAN_ON = 1 and build the project.

-----------------------------------------------------------------------------------------------------------------    
How To BUILD
-----------------------------------------------------------------------------------------------------------------

This MSP430 code needs to be merged with the Boostrap Loader code. To accomplish this 
follow these steps:
1) Download Code Composer Studio(CCS) v4.0 from the link, http://processors.wiki.ti.com/index.php/Download_CCS
2) Open the CCS and Import the project by navigating to, Project--> Import Existing CCS Eclipse Project
3) Browse for the downloaded MSP430 source files and click 'Finish'
4) Compile this by buiding the project in Release/ Debug mode. The build will create "MSP430.hex" file
5) Copy the MSP430.hex file from the Debug/ Release folder and Paste it in MSP430 directory
5) Open the MSDOS command terminal and navigate to this location
6) Execute the MSP430BIN.EXE to merge the Boostrap loader with this code as follows,
   MSP430BIN.EXE -b MSP430_BOOT.hex -m MSP430.hex - o lcrmsp430v3.hex

   Where, lcrmsp430v3.hex is the output file name
7) This command will create two files in the same directory:
   lcrmsp430v3.hex
   lcrmsp430v3.txt
8) Using the GUI, load lcrmsp430v3.txt file in the Upgrade tab with MSP430 SW Package. The LightCrafter GUI only accepts .txt file for the upgrade.
9) Power cycle the LightCrafter


