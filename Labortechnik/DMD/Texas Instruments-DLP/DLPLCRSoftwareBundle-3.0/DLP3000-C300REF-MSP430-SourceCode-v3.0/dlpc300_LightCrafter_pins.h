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

// DPP2607 EVM MSP430F2132 pin definition

#ifndef __dlpc300_indpico_pins
#define __dlpc300_indpico_pins

#ifdef __cplusplus
extern "C" {
#endif
///////////////
/* Industrial Pico mapping of MSP430 pins
 * Pin  Function Type       Init Value	Interrupt                 		Schematic Name
 * P1.0	GPIO	Input					YES (Rising Edge)				INIT_DONE_DPP
 * P1.1	GPIO	Input	                 								BSL_TX_MSP
 * P1.2	GPIO	Input					YES (Rising Edge)				RED_EN
 * P1.3	GPIO	Input					YES (Rising Edge)				GRN_EN
 * P1.4	GPIO	Input					YES (Rising Edge)				BLU_EN
 * P1.5	GPIO	Input					YES (Falling Edge)				LED_ENABLE
 * P1.6	GPIO	Input					YES (Falling Edge)				SW_POWER
 * P1.7	GPIO	Open Drain	HIGH		YES (Falling Edge)				MSP_WAKEUP
 * P2.0	ADC_IN	A0						Timely POLL						THERMISTOR
 * P2.1	ADC_IN	A1						Timely POLL						LOW_BAT
 * P2.2	GPIO	Input													BSL_RX_MSP
 * P2.3	GPIO	Output		HIGH										SYS_FAULTZ
 * P2.4	GPIO	Output		HIGH										SYS_PWRONZ
 * P2.5	GPIO	Input					Read while power down			MSP_P2_5
 * P2.6	GPIO	OpenDrain	HIGH										SYSRESETZ
 * P2.7	GPIO	Input					Poll							PROG_EN_DM
 * P3.0	ADC_IN	A5						Read At PowerUp					DMD_VOLTAGE
 * P3.1	I2C_SDA	Bidi		HIGH										SDA
 * P3.2	I2C_SCL	Bidi		HIGH										SCL
 * P3.3	GPIO	Output		LOW											EXT_I2COE
 * P3.4	GPIO	Output		HIGH										SPIPROGZ
 * P3.5	GPIO	Output		HIGH										LED_ENABLEZ
 * P3.6	GPIO	Output		LOW											PWREN_INTF
 * P3.7	GPIO	Output		LOW											POWERGOOD
 * P4.0	GPIO	Output		LOW											FAN_PWM
 * P4.1	GPIO	Input					Poll							DPP_OFF_MSP
 * P4.2	GPIO	OpenDrain	HIGH										DM_3V3_PWR_SEQ
 * P4.3	ADC_IN	A12						ADC Sample 						LEDDRV_SENSE
 * P4.4	ADC_IN	A13						Read At PowerUp					VREF2
 * P4.5	GPIO	Input					Poll							MSP_P4_5
 * P4.6	GPIO	Output		LOW											PWREN_CORE
 * P4.7	GPIO	OpenDrain	LOW											RESETZ
 */


// Port 1 defines
#define INIT_DONE_DPP   BIT0     //P1.0 input  DLPC300 Init Done / Error Condition 
#define BSL_TX_MSP    	BIT1     //P1.1 input  Boot Strap Loader TX
#define RED_EN		    BIT2     //P1.2 input  LED Red Enable
#define GRN_EN        	BIT3     //P1.3 input  LED Green Enable
#define BLU_EN     		BIT4     //P1.4 input  LED Blue Enable              
#define LED_ENABLE	    BIT5     //P1.5 input  LED Enable
#define SW_POWER	    BIT6     //P1.6 input  Push button input for low / normal power control
#define MSP_WAKEUP	    BIT7     //P1.7 Open Drain: Open drain input to request for low 
                                 //                 power from MSP430 to DM365 or vice versa.

// Port 2 defines
#define THERMISTOR      BIT0     //P2.0 ainput Temperature Sensor
#define LOW_BAT         BIT1     //P2.1 ainput Battery Voltage Sense
#define BSL_RX_MSP      BIT2     //P2.2 input  Boot Strap Loader RX
#define SYS_FAULTZ      BIT3     //P2.3 output System Fault / Warning LED indicator
#define SYS_PWRONZ      BIT4     //P2.4 output Power ON LED indicator
#define MSP_P2_5     	BIT5     //P2.5 input  GPIO connected to DM365 GIO43 for preventing power down if DM365 writing bulk data into memory
#define SYSRESETZ     	BIT6     //P2.6 Open Drain: DM365 Reset
#define PROG_EN_DM      BIT7     //P2.7 input  Request from DM365 for DLPC300 SPI Flash / 
								 //            MSP430 firmware upgrade

// Port 3 defines
#define DMD_VOLTAGE     BIT0     //P3.0 ainput ~1.0V --> 0.3 WVGA; Communicate to DM365 by 
								 //            putting 1 (WVGA) / 0 (nHD) on BSL_TX_MSP.
#define SDA             BIT1     //P3.1 output I2C data, make it output = 1
#define SCL       		BIT2     //P3.2 output IIC clock 
#define EXT_I2COE       BIT3     //P3.3 output When DPP2607 initialization is done, this signal 
                                 //            is asserted to indicate DM365 that ASIC is ready 
                                 //            and also enable the I2C between DM365 and DLPC300
#define SPIPROGZ   		BIT4     //P3.4 output Enable the SPI interface between DM365 and DLPC300 SPI Flash
#define LED_ENABLEZ     BIT5     //P3.5 output Enable LED Strobes
#define PWREN_INTF      BIT6     //P3.6 output Interface + System 3.3V enable
#define POWERGOOD       BIT7     //P3.7 output DLPC300 Power good indicator

// Port 4 defines
#define FAN_PWM     	BIT0     //P4.0 output Control Fan Speed
#define DPP_OFF_MSP     BIT1     //P4.1 input  Request for Power Up/Down command from MSP430 to DM365
#define DM_3V3_PWR_SEQ	BIT2     //P4.2 Open Drain Enable / Disable 3.3V for DM365 
#define LEDDRV_SENSE    BIT3     //P4.3 ainput LED Current value
#define VREF2   		BIT4     //P4.4 ainput LED Current Setting (1mV = 2mA)
#define MSP_P4_5		BIT5     //P4.5 GPIO connected to DM365 GIO42
#define PWREN_CORE      BIT6     //P4.6 output DLPC300 Core Power Enable
#define RESETZ       	BIT7     //P4.7 Open Drain DLPC300 Reset


#ifdef __cplusplus		/* matches __cplusplus construct above */
}
#endif

#endif /* #ifndef __dlpc300_indpico_pins */
