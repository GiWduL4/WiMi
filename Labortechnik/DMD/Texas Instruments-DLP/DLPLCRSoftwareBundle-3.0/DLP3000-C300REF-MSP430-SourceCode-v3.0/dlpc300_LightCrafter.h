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

#ifndef _DLPC300_INDPICO_H_
#define _DLPC300_INDPICO_H_
#include <math.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define LED_MAX_ON_TIME		35	// (2.24sec @ 65ms timer tick)

#define EVT					0
#define nHD					0
 
#define BOOT_BSL_ENTRY		0xC00

#define DLPC300_DEV_ADDR      0x36
#define FPGA_DEV_ADDR         0x34

// Equation for voltage divider for ADC & Supply Monitor:
//
// 3.9V <= SYSPWR <= 5.2V
// [((SYSPWR * 30)/130) / 2.5] * 1024
// [((3.9 * 30)/130) / 2.5] * 1024 = 369
// [((5.5 * 30)/130) / 2.5] * 1024 = 519
// [((7.0 * 30)/130) / 2.5] * 1024 = 661
#define LOW_SYS_VOLTAGE   	369
#define HIGH_SYS_VOLTAGE  	519
#define MAX_SYS_VOLTAGE		661

// Thermistor Resistor (KOhm) vs Temperature Table NCP15WF104F03RC
#define TEMP_n40	4397.119
#define TEMP_n35	3088.599
#define TEMP_n30	2197.225
#define TEMP_n25	1581.881
#define TEMP_n20	1151.037
#define TEMP_n15	846.579 // Assumption that LightEngine is disconnected
#define TEMP_n10	628.988	// Minimum operating temperature of DLP3000
#define TEMP_n5		471.632
#define TEMP_0		357.012
#define TEMP_5		272.500
#define	TEMP_10		209.710
#define	TEMP_15		162.651
#define	TEMP_20		127.080
#define	TEMP_25		100.000
#define	TEMP_30		79.222
#define	TEMP_35		63.167
#define	TEMP_40		50.677
#define	TEMP_45		40.904
#define	TEMP_50		33.195  // Will want to keep case temp under 50C
#define	TEMP_55		27.091
#define	TEMP_60		22.224
#define	TEMP_65		18.323
#define	TEMP_70		15.184	// Maximum tmeperature of DLP3000

// Thermistor Voltage = 1.8 * [RThermistor / (RThermistor + 30)]
#define ThermV_n40  (1.8*(TEMP_n40/(TEMP_n40+30)))
#define ThermV_n35  (1.8*(TEMP_n35/(TEMP_n35+30)))
#define ThermV_n30  (1.8*(TEMP_n30/(TEMP_n30+30)))
#define ThermV_n25  (1.8*(TEMP_n25/(TEMP_n25+30)))
#define ThermV_n20  (1.8*(TEMP_n20/(TEMP_n20+30)))
#define ThermV_n15  (1.8*(TEMP_n15/(TEMP_n15+30)))
#define ThermV_n10  (1.8*(TEMP_n10/(TEMP_n10+30)))
#define ThermV_n5  	(1.8*(TEMP_n5/(TEMP_n5+30)))
#define ThermV_0  	(1.8*(TEMP_0/(TEMP_0+30)))
#define ThermV_5  	(1.8*(TEMP_5/(TEMP_5+30)))
#define ThermV_10 	(1.8*(TEMP_10/(TEMP_10+30)))
#define ThermV_15 	(1.8*(TEMP_15/(TEMP_15+30)))
#define ThermV_20 	(1.8*(TEMP_20/(TEMP_20+30)))
#define ThermV_25 	(1.8*(TEMP_25/(TEMP_25+30)))
#define ThermV_30 	(1.8*(TEMP_30/(TEMP_30+30)))
#define ThermV_35 	(1.8*(TEMP_35/(TEMP_35+30))) 
#define ThermV_40 	(1.8*(TEMP_40/(TEMP_40+30))) 
#define ThermV_45 	(1.8*(TEMP_45/(TEMP_45+30)))
#define ThermV_50 	(1.8*(TEMP_50/(TEMP_50+30)))
#define ThermV_55 	(1.8*(TEMP_55/(TEMP_55+30)))
#define ThermV_60 	(1.8*(TEMP_60/(TEMP_60+30)))
#define ThermV_65 	(1.8*(TEMP_65/(TEMP_65+30)))
#define ThermV_70 	(1.8*(TEMP_70/(TEMP_70+30)))

// Thermistor Voltage = (ADCh0 * 2.5) / 1024 
// ADCh0 = ThermV * 1024 / 2.5
#define ADCh0_n40  (ceil(ThermV_n40  * 1024 / 2.5))
#define ADCh0_n35  (ceil(ThermV_n35  * 1024 / 2.5))
#define ADCh0_n30  (ceil(ThermV_n30  * 1024 / 2.5))
#define ADCh0_n25  (ceil(ThermV_n25  * 1024 / 2.5))
#define ADCh0_n20  (ceil(ThermV_n20  * 1024 / 2.5))
#define ADCh0_n15  (ceil(ThermV_n15  * 1024 / 2.5))
#define ADCh0_n10  (ceil(ThermV_n10  * 1024 / 2.5))
#define ADCh0_n5   (ceil(ThermV_n5  * 1024 / 2.5))
#define ADCh0_0    (ceil(ThermV_0  * 1024 / 2.5))
#define ADCh0_5    (ceil(ThermV_5  * 1024 / 2.5))
#define ADCh0_10   (ceil(ThermV_10 * 1024 / 2.5))
#define ADCh0_15   (ceil(ThermV_15 * 1024 / 2.5))
#define ADCh0_20   (ceil(ThermV_20 * 1024 / 2.5))
#define ADCh0_25   (ceil(ThermV_25 * 1024 / 2.5))
#define ADCh0_30   (ceil(ThermV_30 * 1024 / 2.5))
#define ADCh0_35   (ceil(ThermV_35 * 1024 / 2.5))
#define ADCh0_40   (ceil(ThermV_40 * 1024 / 2.5))
#define ADCh0_45   (ceil(ThermV_45 * 1024 / 2.5))
#define ADCh0_50   (ceil(ThermV_50 * 1024 / 2.5))
#define ADCh0_55   (ceil(ThermV_55 * 1024 / 2.5))
#define ADCh0_60   (ceil(ThermV_60 * 1024 / 2.5))
#define ADCh0_65   (ceil(ThermV_65 * 1024 / 2.5))
#define ADCh0_70    (ceil(ThermV_70 * 1024 / 2.5))


// Micromirror Array Temp
// Tarray = Tcer + (Qarray * Rarray2ceramic)
// Qarray = Qelec + Cl2w * SL
// At 635mA, SL = 35, Cl2w = 0.00274, Rarray2ceramic = 5, Qelec = 0.15
// Tarray = Tcer + (0.15 + 0.00274 * 30) * 5 = Tcer + 1.2295
// For array under 55C, we will set limit to ADCh0_50


// Equation for voltage divider for ADC & Thermistor:
//
// ADC = [((RThermistor) / (RThermistor + 30) / 2.5] * 1024


// Equation for voltage divider for ADC & LED Driver Sense:
//
// LEDDRV_SENSE = [((LED Current) * 0.12) / 2.5] * 1024
//
// led_driver_sense = [0.5 * (0.12/4120.12)] * 1024/2.5 ~0 when LED off
// led_driver_sense = [633e-3 * 0.12] * 1024/2.5 = 31 when LED on at 633mA
// led_driver_sense = [1.5 * 0.12] * 1024/2.5 = 74 when LED on at 1.5A

#define LED_OVERVOLTAGE   76

#define NORMAL_SHUTDOWN			0
#define LOW_BATT_SHUTDOWN		1
#define LED_OVERVOLT_SHUTDOWN	2
#define OVERHEAT_SHUTDOWN		3
#define LED_FAULT_SHUTDOWN		4

typedef enum display_mode_t {
  disp_mode_rgb,
  disp_mode_tpg,
  disp_mode_splash,
  disp_mode_cpu,
  disp_mode_656
} display_mode_t;

typedef enum adjust_mode_t {
  adjust_mode_disp,
  adjust_mode_motor,
  adjust_mode_audio
} adjust_mode_t;

//functions
void Init_TimerA3(void);
void Init_Ports(void);
void write_dlpc300_i2c(uint08 addr, uint08 subaddr, uint32 data, uint08 verify);
void read_dlpc300_i2c(uint08 addr, uint08 subaddr, uint32* data);
void write_i2c_1byte(uint08 addr, uint08 subaddr, uint08 data);
void read_i2c_1byte(uint08 addr, uint08 subaddr, uint08* data);
void setup_tvp5150(void);
void disable_tvp5150(void);
void shutdown(uint08 reason);
void powerup(void);
void process_button(uint08 active_interrupt);
void process_display_adjust(uint08 button);
void process_motor_adjust(uint08 button);
void process_audio_adjust(uint08 button);
unsigned int delay_ms( uint16 ms );
unsigned int blink_sys_fault_LED( uint16 on_time, uint16 off_time );


#ifdef __cplusplus
}
#endif
#endif /* _dlpc300_indpico_H_ */
/*------------------------ Nothing Below This Line --------------------------*/

