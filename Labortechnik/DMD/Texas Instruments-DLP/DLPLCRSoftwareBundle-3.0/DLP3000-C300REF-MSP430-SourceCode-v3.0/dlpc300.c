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
#include "msp430x22x2.h"
#include "dlpc300.h"
#include "version.h"
#include "dlpc300_LightCrafter.h"
#include "dlpc300_LightCrafter_pins.h"
#include "cfg_map.h"

// external variables
extern uint08 tpg_select;
extern uint08 splash_image_select;

uint08 dlpc300_main_configure(void)
/**
 * Common dlpc300 setup for all input types. This must be executed first
 * before attempting to display an image.
 *
 * @return  none
 *
 */
{
	uint08 EnableSafetyChecks = TRUE;
	
  // enable LEDs
  write_dlpc300_i2c(DLPC300_DEV_ADDR, LED_CONTROL, 0x0D, 1);  

  // write the software version number to a spare register field
  write_dlpc300_i2c(DLPC300_DEV_ADDR, MISC_REG, MAJOR<<8 | MINOR, 1);

  // enable HW Test Points 
  //   HW TST PT 3 = Red LED
  //   HW TST PT 2 = INV PAT
  //   HW TST PT 1 = BUF_SWAP
  //   HW TST PT 0 = RD_BUF0
  write_dlpc300_i2c(DLPC300_DEV_ADDR, HW_TEST_P1, 0x14151624, 1);  
  
  // enable HW Test Points 
  //   HW TST PT 7 = RD_BUF1
  //   HW TST PT 6 = CAM_TRIG
  //   HW TST PT 5 = Blue LED
  //   HW TST PT 4 = Green LED
  write_dlpc300_i2c(DLPC300_DEV_ADDR, HW_TEST_P2, 0x25111213, 1);  

  write_dlpc300_i2c(DLPC300_DEV_ADDR, DATA_FORMAT, RGB565, 1);
  write_dlpc300_i2c(DLPC300_DEV_ADDR, INPUT_RESOLUTION, QWVGA_LANDSCAPE, 1);
  write_dlpc300_i2c(DLPC300_DEV_ADDR, INPUT_SOURCE, SPLASH_SCREEN, 1);
  dlpc300_config_splash(  0);
#if nHD
#else
  write_dlpc300_i2c(DLPC300_DEV_ADDR, SHORT_FLIP, 0, 1);
#endif
  write_dlpc300_i2c(DLPC300_DEV_ADDR, LONG_FLIP, 1, 1);
  
if ( !(P1IN & LED_ENABLE) ) {	// LEDs were enabled, so LED_ENABLE must be set
  P2OUT &= ~SYS_FAULTZ;   // Turn System Fault LED ON by setting SYS_FAULTZ low
  shutdown( LED_FAULT_SHUTDOWN );
  } else {
	P3OUT &= ~LED_ENABLEZ;		// Enable LEDs
}
    
  	// Give DM365 time to boot  
  	__delay_cycles(5000000);      // Leave splash for 5 seconds
  	
  	// If power switch is pressed (SW_POWER = LOW) disable safety features
  	if(!(P1IN & SW_POWER))	
  	{
  		// Wait until the power switch is released
  		//while(!(P1IN & SW_POWER))
  		//{
  			EnableSafetyChecks = FALSE;
  		//}
  	}

  	dlpc300_config_splash( 1 );

  	__delay_cycles(5000000);      // Leave splash for 5 seconds
  	dlpc300_config_splash( 2 );

  	__delay_cycles(5000000);      // Leave splash for 5 seconds
  	dlpc300_config_splash( 3 );
 
  	return(EnableSafetyChecks);
 }

uint08 dlpc300_cmt_lut_load(uint08 color)
/**
 * Look-up table loading of CMT
 *
 * @param   color - I - color table to load
 *
 * @return  0 - no errors
 *          1 - invalid color specified
 *
 */
{
  uint32 address, lut_destination, size;

  switch (color) {
    case 1:
      //address = CMT_GRN_LUT_0_START_ADDR;
      address = CMT_LUT_0_START_ADDR;
      lut_destination = CMT_LUT_GREEN;
      size = 512;
      break;
    case 2:
      //address = CMT_RED_LUT_0_START_ADDR;
      address = CMT_LUT_0_START_ADDR+512;
      lut_destination = CMT_LUT_RED;
      size = 512;
      break;
    case 3:
      //address = CMT_BLU_LUT_0_START_ADDR;
      address = CMT_LUT_0_START_ADDR+1024;
      lut_destination = CMT_LUT_BLUE;
      size = 512;
      break;
    case 4:
      address = CMT_LUT_0_START_ADDR;
      lut_destination = CMT_LUT_ALL;
      size = CMT_LUT_0_SIZE;
      break;
    default:
      return 1;
  };

  // configure DMA from flash to LUT
  dlpc300_flash_dma(address, size, 1, lut_destination);

  return 0;
}

uint08 dlpc300_seq_lut_load(uint08 table_number)
/**
 * Look-up table loading of sequence or DRC data
 *
 * @param   table_number - I - sequence or DRC table to load
 *
 * @return  0 - no errors
 *          1 - invalid table_number specified
 *
 */
{
  uint32 address, lut_destination;
  uint32 size = DRC_TABLE_0_SIZE;

  switch (table_number) {
    case 1:
      address = DRC_TABLE_0_START_ADDR;
      lut_destination = SEQ_DRC_LUT_0;
      break;
    case 2:
      address = DRC_TABLE_0_START_ADDR;
      lut_destination = SEQ_DRC_LUT_1;
      break;
    case 3:
      address = DRC_TABLE_0_START_ADDR;
      lut_destination = SEQ_DRC_LUT_2;
      break;
    case 4:
      address = DRC_TABLE_0_START_ADDR;
      lut_destination = SEQ_DRC_LUT_3;
      break;
    case 5:
      address = SEQUENCE_0_START_ADDR;
      size = SEQUENCE_0_SIZE;
      lut_destination = SEQ_SEQ_LUT;
      break;
    case 6:
      address = DRC_TABLE_0_START_ADDR;
      lut_destination = SEQ_DRC_LUT_ALL;
      break;
    case 7:
      address = 0x0; // TODO
      lut_destination = WPC_PROGRAM_LUT;
      break;
    default:
      return 1;
  };

  // configure DMA from flash to LUT
  dlpc300_flash_dma(address, size, 0, lut_destination);

  return 0;
}

uint08 dlpc300_config_tpg(uint08 pattern_select)
/**
 * Configure datapath for test pattern generator operation
 *
 * @param   pattern_select - I - color table to load
 *
 * @return  0 - no errors
 *          1 - invalid pattern specified
 *
 */
{
  if ( pattern_select > TPG_ANSI_CHECKERBOARD )
    return 1;

  write_dlpc300_i2c(DLPC300_DEV_ADDR, SEQ_CONTROL, 0, 1);
  write_dlpc300_i2c(DLPC300_DEV_ADDR, INPUT_RESOLUTION, WVGA_DMD_OPTICAL_TEST, 1);
  write_dlpc300_i2c(DLPC300_DEV_ADDR, SEQUENCE_MODE, SEQ_LOCK, 1);
  write_dlpc300_i2c(DLPC300_DEV_ADDR, TEST_PAT_SELECT, pattern_select, 1);
  write_dlpc300_i2c(DLPC300_DEV_ADDR, INPUT_SOURCE, 1, 1);
  write_dlpc300_i2c(DLPC300_DEV_ADDR, SEQ_CONTROL, 1, 1);

  // no errors
  return 0;
}

uint08 dlpc300_config_splash(uint08 image_number)
/**
 * Configure datapath for splash image operation
 *
 * @param   image_number - I - splash image to load from flash
 *
 * @return  0 - no errors
 *          1 - invalid image_number specified
 *
 */
{
  uint32 address, size;

  switch (image_number) {
    case 0:
      address = SPLASH_0_START_ADDR;
      size = SPLASH_0_SIZE;
      break;
    case 1:
      address = SPLASH_1_START_ADDR;
      size = SPLASH_1_SIZE;
      break;
    case 2:
      address = SPLASH_2_START_ADDR;
      size = SPLASH_2_SIZE;
      break;
    case 3:
      address = SPLASH_3_START_ADDR;
      size = SPLASH_3_SIZE;
      break;
    default:
      return 0;
  };

  // turn off LEDs
  //write_dlpc300_i2c(DLPC300_DEV_ADDR, LED_CONTROL, 0, 1);

  // configure sequence, data format and resolution
  //write_dlpc300_i2c(DLPC300_DEV_ADDR, SEQ_CONTROL, 0, 1);
  //write_dlpc300_i2c(DLPC300_DEV_ADDR, SEQUENCE_MODE, SEQ_FREE_RUN, 1);

  dlpc300_flash_dma(address, size, 1, SPLASH_LUT);

  // turn image back on
  //write_dlpc300_i2c(DLPC300_DEV_ADDR, SEQ_CONTROL, 1, 1);

  // turn on LEDs
  //write_dlpc300_i2c(DLPC300_DEV_ADDR, LED_CONTROL, 0xD, 1);
  
  // no errors
  return 0;
}

uint08 dlpc300_flash_dma(uint32 flash_address, uint32 flash_num_bytes, uint08 CMT_SEQz, uint08 table_number)
/**
 * Configure datapath for splash image operation
 *
 * @param   flash_address   - I - splash image to load from flash
 * @param   flash_num_bytes - I - splash image to load from flash
 * @param   CMT_SEQz        - I - select mailbox to load data to: 0=sequence/DRC, 1=CMT/splash
 * @param   table_number    - I - splash image to load from flash
 *
 * @return  0 - no errors
 *          1 - invalid flash address specified
 *          2 - invalid mailbox specified
 *          3 - invalid table_number / mailbox combination
 *
 */
{
  uint32 dlpc300_status;
  uint16 timer_val, timer_diff;
  uint08 mailbox_address, mailbox_select;
  uint16 baseline_timer_val = TAR;

  // check argument validity
  if ( flash_address > 0x1fffff )
    return 1;
  if ( CMT_SEQz > 1 )
    return 2;
  if ( (CMT_SEQz == 0 && table_number > 6) ||
       (CMT_SEQz == 1 && table_number > 5) )
    return 3;

  // set mailbox parameters
  if ( CMT_SEQz ) {
    mailbox_address = CMT_SPLASH_LUT_START_ADDR;
    mailbox_select = CMT_SPLASH_LUT_DEST_SELECT;
  } else {
    mailbox_address = SEQ_RESET_LUT_START_ADDR;
    mailbox_select = SEQ_RESET_LUT_DEST_SELECT;
  }

  // configure DMA from flash to LUT
  write_dlpc300_i2c(DLPC300_DEV_ADDR, PBC_CONTROL, 0, 0);
  write_dlpc300_i2c(DLPC300_DEV_ADDR, FLASH_START_ADDR, flash_address, 1);
  write_dlpc300_i2c(DLPC300_DEV_ADDR, FLASH_READ_BYTES, flash_num_bytes, 1);
  write_dlpc300_i2c(DLPC300_DEV_ADDR, mailbox_address, 0, 1);
  write_dlpc300_i2c(DLPC300_DEV_ADDR, mailbox_select, table_number, 1);
  // transfer control to flash controller
  write_dlpc300_i2c(DLPC300_DEV_ADDR, PBC_CONTROL, 1, 0);

  // check status of DMA transfer
  // read status until the transfer is complete
  do {
    read_dlpc300_i2c(DLPC300_DEV_ADDR, MAIN_STATUS, &dlpc300_status);

    // get current timer value
    timer_val = TAR;

    // check for TAR rollover since we baselined
    if ( timer_val > baseline_timer_val )
      timer_diff = timer_val - baseline_timer_val;
    else
      timer_diff = (65535 - baseline_timer_val) + timer_val;

    // break if we've waited for ~100ms
    if ( (timer_diff>>10) > 100 )
      break;
  } while ( (dlpc300_status & DMA_STATUS) == DMA_STATUS );

  // return register access to I2c
  write_dlpc300_i2c(DLPC300_DEV_ADDR, PBC_CONTROL, 0, 0);

  // close LUT access
  write_dlpc300_i2c(DLPC300_DEV_ADDR, mailbox_select, 0, 1);

  // no errors
  return 0;
}

void dlpc300_config_rgb(void)
/**
 * Configure datapath for parallel RGB operation
 *
 */
{
  // turn off LEDs
  write_dlpc300_i2c(DLPC300_DEV_ADDR, LED_CONTROL, 0, 1);

  write_dlpc300_i2c(DLPC300_DEV_ADDR, SEQ_CONTROL, 0, 1);
  write_dlpc300_i2c(DLPC300_DEV_ADDR, ACTGEN_CONTROL, 0x10, 1);
  write_dlpc300_i2c(DLPC300_DEV_ADDR, SEQUENCE_MODE, SEQ_LOCK, 1);
  write_dlpc300_i2c(DLPC300_DEV_ADDR, DATA_FORMAT, RGB888, 1);
  write_dlpc300_i2c(DLPC300_DEV_ADDR, INPUT_RESOLUTION, WVGA_DMD_OPTICAL_TEST, 1);
  write_dlpc300_i2c(DLPC300_DEV_ADDR, INPUT_SOURCE, PARALLEL_RGB, 1);

  // turn image back on
  write_dlpc300_i2c(DLPC300_DEV_ADDR, SEQ_CONTROL, 1, 1);

  // turn on LEDs
  write_dlpc300_i2c(DLPC300_DEV_ADDR, LED_CONTROL, 0xD, 1);
}

uint08 dlpc300_config_dvi(void)
/**
 * Configure datapath for BT-656 operation with DVI input
 *
 * @return  0 - no errors
 *          1 - invalid or no source found
 *
 */
{
  uint32 resolution = NTSC_LANDSCAPE;

  // setup dlpc300 datapath
  write_dlpc300_i2c(DLPC300_DEV_ADDR, SEQ_CONTROL, 0, 1);
  write_dlpc300_i2c(DLPC300_DEV_ADDR, ACTGEN_CONTROL, 0x10, 1);
  write_dlpc300_i2c(DLPC300_DEV_ADDR, SEQUENCE_MODE, SEQ_LOCK, 1);
  write_dlpc300_i2c(DLPC300_DEV_ADDR, DATA_FORMAT, RGB888, 1);
  write_dlpc300_i2c(DLPC300_DEV_ADDR, INPUT_RESOLUTION, resolution, 1);
  write_dlpc300_i2c(DLPC300_DEV_ADDR, INPUT_SOURCE, BT656, 1);

  // turn image back on
  write_dlpc300_i2c(DLPC300_DEV_ADDR, SEQ_CONTROL, 1, 1);


  // no errors
  return 0;
}


