/**************************************************************************************************************************************************
*                                 Copyright © 2012 Texas Instruments Incorporated - http://www.ti.com/                                            *
***************************************************************************************************************************************************
*  Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met: *
*                                                                                                                                                 *
*    Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.                 *
*                                                                                                                                                 *
*    Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the        *
*    documentation and/or other materials provided with the distribution.                                                                         *
*                                                                                                                                                 *
*    Neither the name of Texas Instruments Incorporated nor the names of its contributors may be used to endorse or promote products derived      *
*    from this software without specific prior written permission.                                                                                *
*                                                                                                                                                 *
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT          *
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT     *
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT         *
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY    *
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE      *
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                                                                           *
***************************************************************************************************************************************************/

#include "common.h"
#include "msp430f2232.h"
#include "dlpc300_LightCrafter_pins.h"
#include "dlpc300.h"
#include "i2c_master.h"
#include "dlpc300_LightCrafter.h"

#if 0	// ONLY FOR DEBUG
#pragma DATA_SECTION (BOOT, ".boot");
#pragma DATA_ALIGN   (BOOT, 2);
const unsigned char   BOOT[0x200] = {
#include "boot.h"
};
#endif

typedef void (*voidFcn)(void);

#define LED_MAX_ON_TIME		35	// (2.24sec @ 65ms timer tick)

#define THERMISTOR_SENSING	1	// LightEngine overtemperature sensing
#define FAN_ON				0
#define LED_DRIVER_SENSING	0	// If LED_DRIVER_SENDING is enabled (1) and R139 on driver board is not installed, LightCrafter will enter shutdown mode after booting.
#define LED_ENABLE_SENSING	1	// LED_ANODE overvoltage sensing using LED_ENABLE
#define SYS_VOLTAGE_SENSING 1	// Check that system power is within operating voltages
#define SYS_VOLTAGE_AVE_NUM	3	// Number of system voltage measurements to average (3*65msec=195msec)
#define BLINKING_LED        0
#define EVT					0

#define BOOT_BSL_ENTRY		0xC00
 
// global variables
uint08 PowerIsOn;
uint08 FirstPowerUp = 0;
uint08 EnableSafetyChecks = TRUE;
uint16 VRef1 = 0;
uint16 VRef2 = 0;
uint16 DMDVoltage = 0;
uint16 led_driver_sense = 0;   // LEDDRV_SENSE = 0.12 * LED Current
uint16 sys_voltage = 0;
uint16 sys_voltage_cnt = 0;
uint16 sys_voltage_sum = 0;
uint16 sys_voltage_avg = 0;
uint16 thermistor = 0;
uint08 led_timeout = 0;
uint16 adc_vals[15] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
display_mode_t display_mode = disp_mode_splash;
adjust_mode_t adjust_mode = adjust_mode_disp;


/*----------------------------------------------------------------------------+
| Main Routine                                                                |
+----------------------------------------------------------------------------*/
void main( void )
{
  	// Stop watchdog timer to prevent time out reset
  	WDTCTL = WDTPW + WDTHOLD + WDTNMI;
        
  	// Initiallize IO ports
  	Init_Ports();
  	

  	FirstPowerUp = 1;
  	PowerIsOn = 0;
  	__bis_SR_register(GIE);
  	P1IFG = SW_POWER;
  	
  	Init_TimerA3();
    
    while (1) 
    {
    	// after the power on interrupt, go to power mode 0
    	// Enter LPM0 w/ interrupt
    	__bis_SR_register(LPM0_bits + GIE);


    	// Enter LPM4 w/ interrupt
    	__bis_SR_register(LPM4_bits + GIE);
  	}
}

/*----------------------------------------------------------------------------+
| System Initialization Routines.
+----------------------------------------------------------------------------*/
void Init_TimerA3(void)
{
  TACTL = TASSEL_2 + MC_1 + TACLR;  // SMCLK, continuous mode, clear TAR
  TACCR0 = 65535;					// setup for Timer A - 65 msec timeout rate
  TACCTL0 = CCIE;                   // CCR0 interrupt enabled
}
  
/////////////////////////////////////////////////////////////////////////////////////////
void Init_Ports(void)
{
  // Setup GPIOs
  /* Pin  Function Type       Init Value	Interrupt           DM365 Conn	Schematic Name
   * P1.0	GPIO	Input					YES (Rising Edge)				INIT_DONE_DPP
   * P1.1	GPIO	Input	                 						YES		BSL_TX_MSP
   * P1.2	GPIO	Input					YES (Rising Edge)				RED_EN
   * P1.3	GPIO	Input					YES (Rising Edge)				GRN_EN
   * P1.4	GPIO	Input					YES (Rising Edge)				BLU_EN
   * P1.5	GPIO	Input					YES (Falling Edge)				LED_ENABLE
   * P1.6	GPIO	Input					YES (Falling Edge)				SW_POWER
   * P1.7	GPIO	Open Drain	HIGH		YES (Falling Edge)		YES		MSP_WAKEUP
   */
    P1OUT   = 0;    // Set Output low
    P1SEL   = 0;    // Select GPIO capability in Port P1
    P1DIR   = 0;    // Set GPIO as inputs
 

                       

  /* P2.0	ADC_IN	A0						Timely POLL						THERMISTOR
   * P2.1	ADC_IN	A1						Timely POLL						LOW_BAT
   * P2.2	GPIO	Input											YES		BSL_RX_MSP
   * P2.3	GPIO	Output		HIGH								YES		SYS_FAULTZ
   * P2.4	GPIO	Output		HIGH										SYS_PWRONZ
   * P2.5	GPIO	Input					Read during power down	YES		MSP_P2_5
   * P2.6	GPIO	OpenDrain	HIGH										SYSRESETZ
   * P2.7	GPIO	Input					Yes (Falling Edge)				PROG_EN_DM
   */ 
      // Set P2.7 low, P2.6 low, P2.5 low, P2.4 high, P2.3 high, P2.2 low, P2.1 low, P2.0 low
      // Select GPIO capability in Port P2, ADC capability is set in ADC10AE0
      // Set P2.7(I), P2.6(O), P2.5(I), P2.4(O), P2.3(I), P2.2(I), P2.1(I), P2.0(I)
    P2OUT   = (SYS_PWRONZ | SYS_FAULTZ ); 
    P2SEL   = 0;    
    P2DIR   = (SYS_PWRONZ | SYSRESETZ); 
 
  /* P3.0	ADC_IN	A5						Read At PowerUp					DMD_VOLTAGE
   * P3.1	I2C_SDA	Bidi		HIGH								YES		SDA
   * P3.2	I2C_SCL	Bidi		HIGH								YES		SCL
   * P3.3	GPIO	Output		LOW									YES		EXT_I2COE
   * P3.4	GPIO	Output		HIGH								YES		SPIPROGZ
   * P3.5	GPIO	Output		HIGH										LED_ENABLEZ
   * P3.6	GPIO	Output		LOW											PWREN_INTF
   * P3.7	GPIO	Output		LOW											POWERGOOD
   */
      // Set P3.7 low, P3.6 low, P3.5 high, P3.4 high, P3.3 high, P3.2 low, P3.1 low, P3.0 low
      // Select GPIO capability in Port P3, except for I2C. ADC capability is set in ADC10AE0
      // Set P3.7(O), P3.6(O), P3.5(O), P3.4(I), P3.3(I), P3.2(I), P3.1(I), P3.0(I)   
    P3OUT   = (LED_ENABLEZ | EXT_I2COE | SPIPROGZ ); 
    P3SEL   = (SDA | SCL); 
    P3DIR   = (POWERGOOD | PWREN_INTF | LED_ENABLEZ ); 

  /* P4.0	GPIO	Output		LOW											FAN_PWM
   * P4.1	GPIO	Input					Poll					YES		DLP_OFF_DM
   * P4.2	GPIO	OpenDrain	HIGH										DM_3V3_PWR_SEQ
   * P4.3	ADC_IN	A12						ADC Sample 						LEDDRV_SENSE
   * P4.4	ADC_IN	A13						Read At PowerUp					VREF2
   * P4.5	GPIO	Input					Poll         			YES		MSP_P4_5
   * P4.6	GPIO	Output		LOW											PWREN_CORE
   * P4.7	GPIO	OpenDrain	LOW											RESETZ
   */
      // Set P4.7 low, P4.6 low, P4.5 low, P4.4 low, P4.3 low, P4.2 low, P4.1 low, P4.0 low
      // Select GPIO capability in Port P4, ADC capability is set in ADC10AE1
      // Set P4.7(O), P4.6(O), P4.5(I), P4.4(I), P4.3(I), P4.2(O), P4.1(I), P4.0(O) 
#if FAN_ON
	P4OUT = (FAN_PWM);
#else
    P4OUT   = 0; 
#endif
    P4SEL   = 0;    
    P4DIR   = (RESETZ | PWREN_CORE | FAN_PWM | DM_3V3_PWR_SEQ);    
 
  

 	
 
  	// Setup Interrupts
    // Set P1.7, P1.6, and P1.5 as falling edge interrupt
    // Set P1.4, P1.3, P1.2, P1.1, P1.0 as rising edge interrupt
    P1IES   = (MSP_WAKEUP | SW_POWER | LED_ENABLE); 

      // Set P2.7 as rising edge interrupt
    P2IES   = 0; 
    P2IFG   = 0;     // Clear flags
    P2IE     = PROG_EN_DM; // Enable PROG_EN_MSP interrupt

  // Setup Analog Inputs
  ADC10AE0  =  ((DMD_VOLTAGE << 5) | LOW_BAT | THERMISTOR);  // analog pins A5, A1 and A0
  ADC10AE1  = (VREF2 | LEDDRV_SENSE) << 1;        // analog pins A13, A12

  // hold I2C in reset to conserve power
  UCB0CTL1 |= UCSWRST;

  	// 1MHz main clock
  	// Read valid calibration data and store in appropriate registers
  	if ( (CALDCO_1MHZ != 0xFF) || (CALBC1_1MHZ != 0xFF) ) 
  	{
    	BCSCTL1 = CALBC1_1MHZ;
    	DCOCTL = CALDCO_1MHZ;
  	} 
  	else 
  	{ // calibration data is invalid
    	BCSCTL1 = 0x87;
    	DCOCTL = 0x6F;
  	}
  	
  	P1IFG   = 0;     // Clear flags
  	P1IE = (SW_POWER);  
}


////////////////////////////////////////////////
// Timer0 A0 interrupt service routine.

#pragma vector=TIMERA0_VECTOR
__interrupt void TIMERA_ISR(void)
{
	
#if BLINKING_LED
   		if (P2OUT & SYS_FAULTZ)
		{
			P2OUT &= ~SYS_FAULTZ;		// turn on LED indicator
		}
		else
   		{
   			P2OUT |= SYS_FAULTZ;		// turn off LED indicator
   		}
#endif
   
   	// disable timer A interrupt to prevent re-entrance problems
  	TACCTL0 = 0;
  	
  	// Configure ADC to measure ch 1 & 0, multi channel -> THERMISTOR and SYSPWR
  	ADC10CTL1 = ADC10SSEL_3 | INCH_1 | CONSEQ_1;
  	ADC10DTC1 = 2;
  	ADC10CTL0 =  SREF_1 | REFBURST | REF2_5V | REFON | ADC10SR | ADC10SHT_3 | MSC | ADC10ON | ADC10IE ;
  	ADC10CTL0 &= ~ENC;                      // Enable conversion = 0
  	while (ADC10CTL1 & BUSY);               // Wait if ADC10 core is active
  	ADC10SA = (unsigned short)&adc_vals;    // Data buffer start
  	ADC10CTL0 |= (ENC | ADC10SC);           // Sampling and conversion start
	 
  	// go to LPM0 and wait for ADC interrupt   
  	__bis_SR_register(LPM0_bits + GIE);
  
  	thermistor 	= adc_vals[1];
  	sys_voltage = adc_vals[0];
	  
#if THERMISTOR_SENSING
   		// check the thermistor temperature
  		if ( PowerIsOn && (thermistor < ADCh0_70) ) 
  		{
      		P2OUT &= ~SYS_FAULTZ;		// turn on LED indicator
      		if(EnableSafetyChecks)	shutdown(OVERHEAT_SHUTDOWN);
  		}
#endif
 
#if LED_ENABLE_SENSING
		// Check that the LightCrafter is powered, that the LED_ANODE voltage is too high, and there is a connected light engine operating above -15 degC
	    if ( PowerIsOn && (!(P1IN & LED_ENABLE)) && (thermistor <= ADCh0_n15))		 
    	{
	   		P2OUT &= ~SYS_FAULTZ;		// turn on LED indicator
   			if(EnableSafetyChecks)	shutdown(LED_OVERVOLT_SHUTDOWN);
  		}
#endif


#if SYS_VOLTAGE_SENSING
		// First check absolute maximum voltage to avoid damaging parts
		if ( sys_voltage > MAX_SYS_VOLTAGE)  
		{
	  		P2OUT &= ~SYS_FAULTZ;		// turn on LED indicator
  			if(EnableSafetyChecks)	shutdown(LOW_BATT_SHUTDOWN);
  		}
  		
		// Wait until the correct number of sys_voltage measurements have been made
		if( sys_voltage_cnt < SYS_VOLTAGE_AVE_NUM )
		{
			// Add current sys_voltage to sum
			sys_voltage_sum = sys_voltage_sum + sys_voltage;
			
			// Increment counter
			sys_voltage_cnt++;
		}
		else	// Correct number of measurements have been made
		{
			// Calculate average
			sys_voltage_avg = sys_voltage_sum / SYS_VOLTAGE_AVE_NUM;
			
			// Check that average is within operating voltage, otherwise shutdown device
			if ( (sys_voltage_avg < LOW_SYS_VOLTAGE) || (sys_voltage_avg > HIGH_SYS_VOLTAGE) ) 
			{
	  			P2OUT &= ~SYS_FAULTZ;		// turn on LED indicator
  				if(EnableSafetyChecks)	shutdown(LOW_BATT_SHUTDOWN);
  			}
  			else // device is operating at an allowable voltage
			{
				// Reset averaging variables
				sys_voltage_sum = 0;
				sys_voltage_cnt = 0;
			}
		}
#endif

  
  	if(!(P3OUT & LED_ENABLEZ))
  	{
  		if(led_timeout++ >= LED_MAX_ON_TIME)
  		{
  			P3OUT  |= LED_ENABLEZ;
  		} 
  	}
  	// re-enable timer A interrupt
  	TACCTL0 = CCIE;
}

////////////////////////////////////////////////
// Port 1 interrupt service routine. 
//    Detects Power-On

#pragma vector=PORT1_VECTOR
__interrupt void Port1_ISR(void)
{
  uint08 active_interrupt = P1IFG;
  uint08 IntEdgeSel;
  if ( active_interrupt & SW_POWER ) {  // Was interrupt caused by power switch?
    // disable further Port 1 interrupts until we have serviced this one
    P1IE &= ~(SW_POWER | RED_EN | GRN_EN | BLU_EN);
    P1IFG &= ~SW_POWER;
    
    PowerIsOn = PowerIsOn ^ 0x01;
    if ( !PowerIsOn ) {
 	  //DM365 will set MSP_P2_5 HIGH when it is busy in writing bulk data into microSD or NAND flash
      //in such case prevent MSP430 to power down else cause corrupting the memory
	  if((P2IN & MSP_P2_5) == 0x00) 
	  {
 		  shutdown(NORMAL_SHUTDOWN);		 // Turn off DMD
 	  }
	  else
	  {
		  //Since user attempting to power down when saving solution the first thing that should happen after
		  //solution save is power down. So always set flag to '1' to make sure it will power down after
		  //saving solution.
		  PowerIsOn = 1;
	  }

 	  P1IFG = 0;                            // Clears all other pending interrupt
                                            // before proceeding after shutdown
#if FAN_ON
	  P4OUT &= ~(FAN_PWM);
#endif

    } 
    else 
    {
 	  //	  write_dlpc300_i2c(FPGA_DEV_ADDR, 0x20, 0x1, 0);  // switch to DM365 mode
#if FAN_ON
	  P4OUT |= FAN_PWM;
#endif
      powerup();
      P1IFG = 0;
    }
  } else if ( active_interrupt & (RED_EN | GRN_EN | BLU_EN ) ) {  // Was interrupt on of the LED enables?
    // disable further Port 1 interrupts until we have serviced this one
    P1IE  &= ~(RED_EN | GRN_EN | BLU_EN);
 	P1IFG &= ~(RED_EN | GRN_EN | BLU_EN);

	led_timeout = 0;
	P3OUT &= ~LED_ENABLEZ;
    
#if LED_DRIVER_SENSING
  	// Configure ADC to measure ch 12, single channel -> LEDDRV_SENSE = .12 * (LED Current)
  	//	LED Current = 635mA, LEDDRV_SENSE = 0.76V
  	ADC10CTL1 =  ADC10SSEL_3 | INCH_12 | CONSEQ_0;
  	ADC10DTC1 = 1;
  	ADC10CTL0 = SREF_1 | REFBURST | REF2_5V | REFON | ADC10SR | ADC10SHT_3 | ADC10ON | ADC10IE ;
  	ADC10CTL0 &= ~ENC;                      // Enable conversion = 0
  	while (ADC10CTL1 & BUSY);               // Wait if ADC10 core is active
  	ADC10SA = (unsigned short)&adc_vals;    // Data buffer start
  	ADC10CTL0 |= (ENC | ADC10SC);           // Sampling and conversion start

   	// go to LPM0 and wait for ADC interrupt   
  	__bis_SR_register(LPM0_bits + GIE);
  
  	led_driver_sense = adc_vals[0];

   	// check the LED cable connection
    if ( (led_driver_sense > LED_OVERVOLTAGE) ) {
   		P2OUT &= ~SYS_FAULTZ;		// turn on LED indicator
   		if(EnableSafetyChecks)	shutdown(LED_OVERVOLT_SHUTDOWN);
  	}
#endif



    // Configure LED STROBES interrupt to next edge (rising / falling based on current state)
    IntEdgeSel  = P1IES;
    IntEdgeSel &=        ~(RED_EN | GRN_EN | BLU_EN);
    IntEdgeSel |= (P1IN & (RED_EN | GRN_EN | BLU_EN));  
    P1IES       = IntEdgeSel;
    // Clear interrupt flag if get set because of edge select operation
    P1IFG &= ~(RED_EN | GRN_EN | BLU_EN);
    P1IE  |=  (RED_EN | GRN_EN | BLU_EN);
  } else {                  // MSP_WAKEUP must have caused the interrupt
   	__bic_SR_register_on_exit(LPM3_bits);
  } 
    
  // set a timer to mask off further Port 1 interrupts
  WDTCTL = WDT_MDLY_32;                 // WDT Interval timer (32ms)
  IE1 |= WDTIE;                         // Enable WDT interrupt
  
  if(active_interrupt & SW_POWER)
  __delay_cycles(010000); // debounce

  // re-enable the power button interrupt
  P1IE |= (SW_POWER | RED_EN | GRN_EN | BLU_EN);
  if ( PowerIsOn )
    __bic_SR_register_on_exit(LPM4_bits);
  else
    __bic_SR_register_on_exit(LPM0_bits);
}

///////////////////////////////////////////////
// Port 2 interrupt service routine for response

#pragma vector=PORT2_VECTOR
__interrupt void Port2_ISR(void)
{
   uint08 active_interrupt = P2IFG;

 	if ( active_interrupt & PROG_EN_DM ) {  // Was interrupt caused by request from DM365 for SPI bus?
         // disable further Port 1 interrupts until we have serviced this one
    	P2IE &= ~PROG_EN_DM;
    	P1OUT |= BSL_TX_MSP;    // Drive TX high
    	P4OUT &= ~(RESETZ);	    // Place DLPC300 in reset
    	P4DIR |= RESETZ;        // Set RESETZ as output
    	P2DIR &= ~BSL_RX_MSP;   // Ensure that BSL_RX_MSP is an input
    	P1DIR |= BSL_TX_MSP;    // Set BSL_TX_MSP as output
    	
    	P3DIR |= SPIPROGZ;    	// Set SPIPROGZ as output
    	P3OUT &= ~(SPIPROGZ);   // Drive SPIPROGZ low to connect SPI from DM365 to DLPC300
        //P2DIR |= MSP_P2_5;		// Set MSP_P2_5 high
 	    while ( P2IN & PROG_EN_DM );  // Wait until PROG_EN_DM is lowered
        //__delay_cycles(500000);   // Wait for 500 ms to allow DM365 to respond success to command and prevent GUI from crashing   
		// Jump to BSL
		
		P4DIR &= ~RESETZ;
		P3OUT |= SPIPROGZ;

		if(P2IN & BSL_RX_MSP)
		{
			BCSCTL2 = 0;
			_bic_SR_register(0xFFFF);
			(*((voidFcn *)(BOOT_BSL_ENTRY)))();
		}
		else
		{
			P1OUT &= ~BSL_TX_MSP;
		}
 	}
   
    P2IES   = 0; 
    P2IFG   = 0;
    P2IE    = PROG_EN_DM;
}

// Watchdog ISR used to re-enable Port 1 interrupts
#pragma vector=WDT_VECTOR
__interrupt void Watchdog_ISR(void)
{

  // disable the watchdog since we don't need it again until after the next port 1 interrupt
  IE1 &= ~WDTIE;
}

// ADC10 interrupt service routine
#pragma vector=ADC10_VECTOR
__interrupt void ADC10_ISR(void)
{
    ADC10CTL0 &= ~ENC;                      // Disable conversion
    __bic_SR_register_on_exit(LPM0_bits);
}


void write_dlpc300_i2c(uint08 addr, uint08 subaddr, uint32 data, uint08 verify)
{
  uint32 read_data;
  uint08 num_written;
  uint08 i2c_array[5];
  
  i2c_array[0] =           subaddr;
  i2c_array[1] = (uint08) (data>>24);
  i2c_array[2] = (uint08) (data>>16);
  i2c_array[3] = (uint08) (data>>8);
  i2c_array[4] = (uint08) data;
  i2c_master_polled_write(addr, i2c_array, 5, &num_written, 30); 
    
  if ( verify ) {
    read_dlpc300_i2c(addr, subaddr, &read_data);

  }
}

void read_dlpc300_i2c(uint08 addr, uint08 subaddr, uint32* data)
{
  uint08 num_written, num_read;
  uint08 i2c_array[4];
  uint08 status;
  
  // setup the readback
  i2c_array[0] = (uint08) READ_REG_SELECT;
  i2c_array[1] =          subaddr;
  status = i2c_master_polled_write(addr, i2c_array, 2, &num_written, 30); 
  
  // flag an error if necessary
  if ( (status != 0) || num_written != 2 ) {
    *data = 0;
    return;
  }
    
  // perform the read
  status = i2c_master_polled_read(addr, i2c_array, 4, &num_read, 30); 
  
  // flag an error if necessary
  if ( (status != 0) || num_read != 4 ) {
    *data = 0;
    return;
  }
    
  // concatenate the bytes to make a word
  *data = (uint32)i2c_array[0]<<24 |
          (uint32)i2c_array[1]<<16 |
          (uint32)i2c_array[2]<<8  |
          (uint32)i2c_array[3];
}

void write_i2c_1byte(uint08 addr, uint08 subaddr, uint08 data)
{
  uint08 num_written, i2c_array[4];
  
  i2c_array[0] = subaddr;
  i2c_array[1] = data;
  i2c_master_polled_write(addr, i2c_array, 2, &num_written, 30); 
    
}

void read_i2c_1byte(uint08 addr, uint08 subaddr, uint08* data)
{
  uint08 status, num_written, num_read;
  
  status = i2c_master_polled_write(addr, &subaddr, 1, &num_written, 30);
  
  // no sense continuing if the slave does not respond
  if ( status == I2C_NO_ACK ) {
    return;
  }
  
  status = i2c_master_polled_read(addr, data, 1, &num_read, 30); 

}

// PowerUp function
void powerup(void)
{
	  volatile uint32 i;
	  
	  P4OUT |= PWREN_CORE;		// Turn on 2.5 V power supply
      __delay_cycles(120000);	// Delay 100ms for power supply sequencing
 
      P3OUT |= ( PWREN_INTF );		// Turn on 3.3 V power supply 
      P4DIR &= ~DM_3V3_PWR_SEQ; // Make sure 3.3V is controlled by core supply pull-up
      __delay_cycles(100000);   // Wait for 100 ms to release reset   
  
#if EVT
      P3DIR |= (SPIPROGZ | EXT_I2COE);      // When 3.3V is up, set SPIPROG, output EXT_I2COE for EVT boards
      										// to enable pull-ups on I2C
#else
      P3DIR |= (SPIPROGZ);      // When 3.3V is up, set SPIPROG
#endif
      P2DIR |= SYS_FAULTZ;		// When 3.3V is up, set SYS_FAULTZ to output
      P2DIR &= ~SYSRESETZ;		// When 3.3V is up, release DM365 reset by setting to input an using pull-up
      P4DIR &= ~RESETZ;			// Release reset to DLPC300 by setting to input an using pull-up
      P3OUT |= POWERGOOD | LED_ENABLEZ;       // Assert PowerGood with LEDs disabled

  	  
	 i = 0;    
      // wait for DLPC300 GPIO4 to go high
      while ( (P1IN & INIT_DONE_DPP) == 0 ) {
        i++;
        // 500000 / 6 - 1 , 500msec
        if ( i == 83332 )
          break;
      }
  		

  	  i = 0;
      // wait for DLPC300 GPIO4 to go low
      while ( (P1IN & INIT_DONE_DPP) == 1 ) {
        i++;
        // 500000 / 6 - 1 , 500msec
        if ( i == 83332 )
          break;
      }

  	  P2DIR &= ~SYSRESETZ;    // Release DM365 from reset by setting to input and using pull-up
      P2OUT &= ~SYS_PWRONZ;   // Turn System Power LED ON by setting SYS_PWRONZ low

      i2c_master_init();	 // I2C controller setup
      
	  // unpark the DMD
      write_dlpc300_i2c(DLPC300_DEV_ADDR, DMD_PARK_TRIGGER, 0, 0);

      EnableSafetyChecks = dlpc300_main_configure();
      
      P2OUT |= SYS_FAULTZ;		// turn off system fault LED indicator
          
      // Configure ADC to measure ch 13, single channel, 15 conversions  -> VRef2 ~ 0.9V
      ADC10CTL1 = ADC10DIV_4 | ADC10SSEL_3 | INCH_13 | CONSEQ_2;
      ADC10DTC1 = 15;
      ADC10CTL0 = SREF_1 | REFBURST | REF2_5V | REFON | ADC10SR | ADC10SHT_2 | MSC | ADC10ON | ADC10IE ;
      ADC10CTL0 &= ~ENC;                      // Enable conversion = 0
      while (ADC10CTL1 & BUSY);               // Wait if ADC10 core is active
      ADC10SA = (unsigned short)&adc_vals;    // Data buffer start
      ADC10CTL0 |= (ENC | ADC10SC);           // Sampling and conversion start
      
      // go to LPM0 and wait for ADC interrupt   
      __bis_SR_register(LPM0_bits + GIE);
      
      for ( i=0; i < 16; i++ ) {
      	VRef2 += adc_vals[i];
      }
      VRef2 = VRef2 / 15;
      
      
      // Configure ADC to measure ch 5, single channel, 15 conversions  -> DMD_VOLTAGE ~ 1.0V
      ADC10CTL1 = ADC10DIV_4 | ADC10SSEL_3 | INCH_5 | CONSEQ_2;
      ADC10DTC1 = 15;
      ADC10CTL0 = SREF_1 | REFBURST | REF2_5V | REFON | ADC10SR | ADC10SHT_2 | MSC | ADC10ON | ADC10IE ;
      ADC10CTL0 &= ~ENC;                      // Enable conversion = 0
      while (ADC10CTL1 & BUSY);               // Wait if ADC10 core is active
      ADC10SA = (unsigned short)&adc_vals;    // Data buffer start
      ADC10CTL0 |= (ENC | ADC10SC);           // Sampling and conversion start
      
      // go to LPM0 and wait for ADC interrupt   
      __bis_SR_register(LPM0_bits + GIE);
      
      for ( i=0; i < 16; i++ ) {
      	DMDVoltage += adc_vals[i];
      }
      DMDVoltage = DMDVoltage / 15;
      
      P3DIR |= (EXT_I2COE);        // When finished EXT_I2COE to output to indicate ASIC_READY
      
      // Start Timer
      TACCTL0 = CCIE;
      

}



// call this function when either a fault occurs or the user requests shutdown
//  normal user generated shutdown - reason = 0
//  if the battery is low          - reason = 1
//  if LED overvoltage             - reason = 2
void shutdown(uint08 reason)
{
  
  	// enable interrupts to allow I2C to work
  	__bis_SR_register(GIE);

  	// turn off LEDs
  	write_dlpc300_i2c(DLPC300_DEV_ADDR, LED_CONTROL, 0, 1);

	// park the DMD
  	write_dlpc300_i2c(DLPC300_DEV_ADDR, DMD_PARK_TRIGGER, 1, 0);

  	// turn off timers
  	TACCTL0 = 0;
  	TBCCTL0 = 0;
  
  	// make sure the watchdog interrupt is off
  	IE1 &= ~WDTIE;
  
  	// ADC10 controls can only be modified when ENC = 0, so clear it first,
  	//  then clear the rest of the register
  	ADC10CTL0 &= ~ENC;
  	ADC10CTL0 = 0;
  
  	P2OUT |= SYS_PWRONZ;   // Turn System Power LED Off
 
  	P3OUT &= ~(POWERGOOD);
  	P3OUT |= LED_ENABLEZ;    // De-assert PowerGood and disable LEDs
  
  	UCB0CTL1 |= UCSWRST;    // hold I2C in reset to conserve power
	  
  	__delay_cycles(510);	  // Delay 510us to let DMD get parked
  
  	P2OUT &= ~SYSRESETZ;   // Hold DM365 in Reset
  	P2DIR |= SYSRESETZ;    // Allow shutdown to hold SYSRESETZ low
  	P3DIR &= ~(EXT_I2COE | SPIPROGZ);		// Set pins connected to DM365 to input before shutting down its I/O
	//  P2DIR &= ~SYS_FAULTZ;		// When 3.3V is down, set SYS_FAULTZ to input
  	P4OUT &= ~(RESETZ | DM_3V3_PWR_SEQ);	// Place in reset, shut DM365 3.3V
  
  	P4DIR |= (RESETZ | DM_3V3_PWR_SEQ); // Make sure DM365 3.3V is off
  
  	__delay_cycles(100000);   // Wait for 100 ms after reset and before 1.8V power-off   
 
  	P3OUT &= ~PWREN_INTF;		// Turn off 3.3 V power supply 
  	// set I/Os connected to DLPC300 to input
  	P1DIR = 0;
	  
  	P4OUT &= ~PWREN_CORE;		// Turn off 2.5 V power supply

  	// disable global interrupts
  	__bic_SR_register(GIE);

  	// Blink the System fault LED to indicate failure -- Added 08-29-2012
  	//  LOW_BATT_SHUTDOWN : ON for 5sec OFF for 500msec
  	//  LED_OVERVOLT_SHUTDOWN : ON for 5sec OFF for 5sec
  	//  OVERHEAT_SHUTDOWN : ON 1.5sec off 1sec
  	//  LED_FAULT_SHUTDOWN : ON for 1sec off for 10sec

	switch( reason )
	{
		case 0:
			// Continue normal shutdown mode
			break;
		case 1:	//LOW_BATT_SHUTDOWN
			blink_sys_fault_LED(5000,500);
			break;
		case 2: //LED_OVERVOLT_SHUTDOWN
			blink_sys_fault_LED(5000,5000);
			break;
		case 3: //OVERHEAT_SHUTDOWN
			blink_sys_fault_LED(1500,1000);
			break;
		case 4: //LED_FAULT_SHUTDOWN
			blink_sys_fault_LED(1000,10000);
			break;
	}
}

unsigned int delay_ms( uint16 ms )
{
	uint16 i;
	for( i=0; i < ms; i++)
	{
		// Delay for 1ms
		__delay_cycles(1000);
	}
	return(i);	// Return number of ms delayed
}

unsigned int blink_sys_fault_LED( uint16 on_time, uint16 off_time )
{
	// Stop watchdog timer to prevent time out reset
	WDTCTL = WDTPW + WDTHOLD + WDTNMI;
	
	// Set LED pin as output
	P2DIR |= SYS_FAULTZ;
	
	// Blink LED until complete reboot
	while(1)
	{
		P2OUT &= 0;				// Turn on LED indicator
		delay_ms(on_time);		// Keep LED on for off_time in ms
		P2OUT |= SYS_FAULTZ;	// Turn off LED indicator
		delay_ms(off_time);		// Keep LED off for on_time in ms
	}
}

