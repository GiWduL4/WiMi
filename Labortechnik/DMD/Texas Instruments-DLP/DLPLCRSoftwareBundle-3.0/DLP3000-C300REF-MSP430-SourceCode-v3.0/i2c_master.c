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
#include "i2c_master.h"

// local variables
uint08 i2c_desired_byte_count;
uint16 i2c_baseline_timer;
uint08* i2c_data_ptr;
uint08 i2c_actual_byte_count;
uint08 i2c_complete = 0;
uint08 i2c_return_val = 0;

// local functions
uint08 i2c_master_check_nak_timeout(uint08 timeout, uint08 check_nak);
void i2c_master_cleanup(void);

void i2c_master_init(void)
{
  // pin setup should have been done prior to calling this function
  // make sure it is in reset
  UCB0CTL1 = UCSWRST;

  // master mode configuration
  UCB0CTL0 = UCMST + UCMODE_3 + UCSYNC;
  UCB0CTL1 |= UCSSEL_2;
  // assume 1MHz clock with 100KHz SCL rate
  UCB0BR0 = 10;
  UCB0BR1 = 0;

  // configure Timer B for use with the timeout checking of read or write operations
  TBCTL = TBSSEL_2 + MC_2;
}

void i2c_master_set_clockrate(uint32 scl_rate)
/**
 * Configure the SCL clock rate of the I2C bus
 *
 * @param   scl_rate   - I - desired SCL frequency; i.e. 100,000 or 400,000
 *
 */
{
}

uint08 i2c_master_polled_write(uint08 device_addr, uint08* write_data, uint08 num_bytes, uint08* bytes_written, uint08 timeout)
/**
 * Writes data to the specified device address
 *
 * @param   device_addr   - I - 7 Bit device Address
 * @param   write_data    - I - Pointer to data buffer to be written to slave
 * @param   num_bytes     - I - Number of bytes to be written
 * @param   bytes_written - O - Actual number of bytes written to slave
 * @param   timeout       - I - Timeout max (in msec) between each transmitted byte
 *                              Timeout=0, permits an infinite timeout (not recommended). Max value = 65
 * @return  0 - Completed successfully 
 *          I2C_NO_ACK - Slave NAck'ed            
 *          I2C_WRITE_TIMEOUT - Slave did not respond before timeout period expired
 *          I2C_INVALID_TIMEOUT - timeout parameter is greater than 65
 *
 */
{
  uint08 status;
  uint08* data_ptr = write_data;
  uint08 i;
  uint16 current_sr;
  uint08 int_disabled = 0;
  
  // check for argument validity
  if ( timeout > 65 )
    return I2C_INVALID_TIMEOUT;
    
  // since this version is polled, we don't want an interrupt firing
  //  and the ISR executing
  current_sr = __get_SR_register();
  if ( current_sr & GIE ) {
    int_disabled = 1;
    __disable_interrupt();
  }
  
  // turn on the timer
  TBCTL |= MC_2;
  i2c_baseline_timer = TBR;
  
  // write the slave address and configure i2c hardware
  UCB0CTL1 = UCSWRST;
  UCB0CTL1 = UCSSEL_2 + UCSWRST;
  UCB0I2CSA = device_addr>>1;
  UCB0CTL1 &= ~UCSWRST;

  // enable module interrupts and start the transfer
  UCB0I2CIE |= UCNACKIE;
  IE2 |= UCB0TXIE;
  UCB0CTL1 |= UCTR + UCTXSTT;
  
  *bytes_written = 0;
  
  // special handling of 1 byte case
  if ( num_bytes == 1 ) {
    while ( (IFG2 & UCB0TXIFG) == 0 ) {
      status = i2c_master_check_nak_timeout(timeout, 1);
      
      if ( status != 0 ) {
        i2c_master_cleanup();
        if ( int_disabled )
          __enable_interrupt();
        return status;
      }
    }
    
    UCB0TXBUF = *data_ptr;
    while ( (IFG2 & UCB0TXIFG) == 0 ) {
      status = i2c_master_check_nak_timeout(timeout, 1);
      
      if ( status != 0 ) {
        i2c_master_cleanup();
        if ( int_disabled )
          __enable_interrupt();
        return status;
      }
    }
    *bytes_written = 1;
  }
  else {
    for ( i=0; i<num_bytes; i++ ) {
      // spin until transmit buffer needs to be reloaded
      while ( (IFG2 & UCB0TXIFG) == 0 ) {
        status = i2c_master_check_nak_timeout(timeout, 1);
        
        if ( status != 0 ) {
          i2c_master_cleanup();
          if ( int_disabled )
            __enable_interrupt();
          return status;
        }
      }
      
      // load another data byte
      UCB0TXBUF = *data_ptr;
      
      // increment actual byte counter
      *bytes_written = *bytes_written + 1;
      
      // increment the pointer address
      data_ptr++;
      
      // take a new baseline timer value
      i2c_baseline_timer = TBR;
    }
  }
  
  // wait until the last byte has been transferred
  while ( (IFG2 & UCB0TXIFG) == 0 ) {
    status = i2c_master_check_nak_timeout(timeout, 1);
    
    if ( status != 0 ) {
      i2c_master_cleanup();
      if ( int_disabled )
        __enable_interrupt();
      return status;
    }
  }
  // generate a stop
  UCB0CTL1 |= UCTXSTP;

  // turn off the timer to save power
  TBCTL &= ~MC_2;
  IFG2 &= ~UCB0TXIFG;
  
  // no errors, all bytes transmitted
  UCB0I2CIE &= ~UCNACKIE;
  IE2 &= ~UCB0TXIE;
  if ( int_disabled )
    __enable_interrupt();
  return 0;
}

uint08 i2c_master_polled_read(uint08 device_addr, uint08* read_data, uint08 num_bytes, uint08* bytes_read, uint08 timeout)
/**
 * Reads data from the specified device address
 *
 * @param   device_addr   - I - 7 Bit device Address
 * @param   read_data     - I - Pointer to buffer to hold received data from slave
 * @param   num_bytes     - I - Number of bytes to be read
 * @param   bytes_read    - O - Actual number of bytes read from slave
 * @param   timeout       - I - Timeout max (in msec) between each received byte
 *                              Timeout=0, permits an infinite timeout (not recommended). Max value = 65
 * @return  0 - Completed successfully 
 *          I2C_NO_ACK - Slave NAck'ed            
 *          I2C_TIMEOUT - Slave did not respond before timeout period expired
 *          I2C_INVALID_TIMEOUT - timeout parameter is greater than 65
 *
 */
{
  uint08 status;
  uint08* data_ptr = read_data;
  uint08 i;
  uint16 current_sr;
  uint08 int_disabled = 0;
  
  // check for argument validity
  if ( timeout > 65 )
    return I2C_INVALID_TIMEOUT;
    
  // since this version is polled, we don't want an interrupt firing
  //  and the ISR taking over
  current_sr = __get_SR_register();
  if ( current_sr & GIE ) {
    int_disabled = 1;
    __disable_interrupt();
  }
    
  // turn on the timer
  TBCTL |= MC_2;
  i2c_baseline_timer = TBR;

  // write the slave address and kick off the transfer
  UCB0CTL1 = UCSWRST;
  UCB0CTL1 = UCSSEL_2 + UCSWRST;
  UCB0I2CSA = device_addr>>1;
  UCB0CTL1 &= ~UCSWRST;
  UCB0I2CIE |= UCNACKIE;
  IE2 |= UCB0RXIE;

  UCB0CTL1 |= UCTXSTT;
    
  if ( num_bytes == 1 ) {
    // spin until start condition cleared
    while ( UCB0CTL1 & UCTXSTT ) {
      status = i2c_master_check_nak_timeout(timeout, 1);
      
      if ( status != 0 ) {
        i2c_master_cleanup();
        if ( int_disabled )
          __enable_interrupt();
        return status;
      }
    }
    UCB0CTL1 |= UCTXSTP;
    while ( (IFG2 & UCB0RXIFG) == 0 ) {
      status = i2c_master_check_nak_timeout(timeout, 1);
      
      if ( status != 0 ) {
        i2c_master_cleanup();
        if ( int_disabled )
          __enable_interrupt();
        return status;
      }
    }
    *read_data = UCB0RXBUF;
    *bytes_read = 1;
  } else {
    *bytes_read = 0;
    
    for ( i=0; i<num_bytes; i++ ) {
      // spin until data is available
      while ( (IFG2 & UCB0RXIFG) == 0 ) {
        status = i2c_master_check_nak_timeout(timeout, 1);
        
        if ( status != 0 ) {
          i2c_master_cleanup();
          if ( int_disabled )
            __enable_interrupt();
          return status;
        }
      }
      
      // increment actual byte counter
      *bytes_read = *bytes_read + 1;
      
      // copy received byte
      *data_ptr = UCB0RXBUF;
      
      // increment the pointer address
      data_ptr++;
      
      // take a new baseline timer value
      i2c_baseline_timer = TBR;
  
      // send stop after the next byte
      if ( i == (num_bytes-2) )
        UCB0CTL1 |= UCTXSTP;
    }
  }    
  
  // turn off the timer to save power
  TBCTL &= ~MC_2;
  UCB0I2CIE &= ~UCNACKIE;
  IE2 &= ~UCB0RXIE;
  
  // no errors, all bytes received
  if ( int_disabled )
    __enable_interrupt();
  return 0;
}

uint08 i2c_master_polled_write_restart_read(uint08 device_addr, uint08* write_data, uint08 num_write_bytes, uint08* bytes_written, uint08* read_data, uint08 num_read_bytes, uint08* bytes_read, uint08 timeout)
/**
 * Reads data from the specified device address using the write-restart-read method
 *
 * @param   device_addr     - I - 7 Bit device Address
 * @param   write_data      - I - Pointer to data buffer to be written to slave
 * @param   num_write_bytes - I - Number of bytes to be written
 * @param   bytes_written   - O - Actual number of bytes written to slave
 * @param   read_data       - I - Pointer to buffer to hold received data from slave
 * @param   num_read_bytes  - I - Number of bytes to be read
 * @param   bytes_read      - O - Actual number of bytes read from slave
 * @param   timeout         - I - Timeout max (in msec) between each received byte
 *                                Timeout=0, permits an infinite timeout (not recommended). Max value = 65
 * @return  0 - Completed successfully 
 *          I2C_NO_ACK - Slave NAck'ed            
 *          I2C_TIMEOUT - Slave did not respond before timeout period expired
 *          I2C_INVALID_TIMEOUT - timeout parameter is greater than 65
 *
 */
{
  uint08* data_ptr = write_data;
  uint08 i, status;
  uint16 current_sr;
  uint08 int_disabled = 0;
  
  // check for argument validity
  if ( timeout > 65 )
    return I2C_INVALID_TIMEOUT;
    
  // since this version is polled, we don't want an interrupt firing
  //  and the ISR taking over
  current_sr = __get_SR_register();
  if ( current_sr & GIE ) {
    int_disabled = 1;
    __disable_interrupt();
  }
    
  // turn on the timer
  TBCTL |= MC_2;
  i2c_baseline_timer = TBR;

  // write the slave address and kick off the transfer
  UCB0CTL1 = UCSWRST;
  UCB0CTL1 = UCSSEL_2 + UCSWRST;
  UCB0I2CSA = device_addr>>1;
  UCB0CTL1 &= ~UCSWRST;
  UCB0CTL1 |= UCTR + UCTXSTT;
  
  *bytes_written = 0;
  *bytes_read =0;
  
  for ( i=0; i<num_write_bytes; i++ ) {
    // spin until transmit buffer needs to be reloaded
    while ( (IFG2 & UCB0TXIFG) == 0 ) {
      // check for NAK and timeout
      status = i2c_master_check_nak_timeout(timeout, 1);
      
      if ( status != 0 ) {
        i2c_master_cleanup();
        if ( int_disabled )
          __enable_interrupt();
        return status;
      }
    }
    
    // load another data byte
    UCB0TXBUF = *data_ptr;
    
    // increment actual byte counter
    *bytes_written = *bytes_written + 1;
    
    // increment the pointer address
    data_ptr++;
    
    // take a new baseline timer value
    i2c_baseline_timer = TBR;
  } // end of write loop
  
  // kick off repeated start for reading
  UCB0CTL1 &= ~UCTR;
  UCB0CTL1 |= UCTXSTT;

  // reset locals for the read phase  
  data_ptr = read_data;
  i2c_baseline_timer = TBR;
    
  // special code for 1 byte case
  if ( num_read_bytes == 1 ) {
    // spin until start condition cleared
    while (UCB0CTL1 & UCTXSTT) {
      // check for timeout
      status = i2c_master_check_nak_timeout(timeout, 0);
      
      if ( status != 0 ) {
        i2c_master_cleanup();
        if ( int_disabled )
          __enable_interrupt();
        return status;
      }
    }
    UCB0CTL1 |= UCTXSTP;
    
    // spin until data is available
    while ( (IFG2 & UCB0RXIFG) == 0 ) {
      // check for NAK and timeout
      status = i2c_master_check_nak_timeout(timeout, 1);
      
      if ( status != 0 ) {
        i2c_master_cleanup();
        if ( int_disabled )
          __enable_interrupt();
        return status;
      }
    }
    
    // increment actual byte counter
    *bytes_read = *bytes_read + 1;
    
    // copy received byte
    *data_ptr = UCB0RXBUF;
  }
  else {
    for ( i=0; i<num_read_bytes; i++ ) {
      // spin until data is available
      while ( (IFG2 & UCB0RXIFG) == 0 ) {
        // check for NAK and timeout
        status = i2c_master_check_nak_timeout(timeout, 1);
        
        if ( status != 0 ) {
          i2c_master_cleanup();
          if ( int_disabled )
            __enable_interrupt();
          return status;
        }
      }
      
      // increment actual byte counter
      *bytes_read = *bytes_read + 1;
      
      // copy received byte
      *data_ptr = UCB0RXBUF;
      
      // increment the pointer address
      data_ptr++;
      
      // wait for flag to be cleared
      while ( IFG2 & UCB0RXIFG) {
        // check for timeout
        status = i2c_master_check_nak_timeout(timeout, 0);
        
        if ( status != 0 ) {
          i2c_master_cleanup();
          if ( int_disabled )
            __enable_interrupt();
          return status;
        }
      }
  
      // take a new baseline timer value
      i2c_baseline_timer = TBR;
  
      // send stop after the next byte
      if ( i == (num_read_bytes-2) )
        UCB0CTL1 |= UCTXSTP;
  
    } // end of read loop
  }
    
  // no errors, all bytes received
  if ( int_disabled )
    __enable_interrupt();
  return 0;
}

uint08 i2c_master_write(uint08 device_addr, uint08* write_data, uint08 num_bytes, uint08* bytes_written, uint08 timeout)
/**
 * Writes data to the specified device address
 *
 * @param   device_addr   - I - 7 Bit device Address
 * @param   write_data    - I - Pointer to data buffer to be written to slave
 * @param   num_bytes     - I - Number of bytes to be written
 * @param   bytes_written - O - Actual number of bytes written to slave
 * @param   timeout       - I - Timeout max (in msec) for the entire transaction
 *                              Timeout=0, permits an infinite timeout (not recommended). Max value = 65
 * @return  0 - Completed successfully 
 *          I2C_NO_ACK - Slave NAck'ed            
 *          I2C_WRITE_TIMEOUT - Slave did not respond before timeout period expired
 *          I2C_INVALID_TIMEOUT - timeout parameter is greater than 65
 *
 */
{
  uint08 status = 0;
  uint08 ret_val;
  uint16 current_sr;
  uint08 int_enabled = 0;
  
  // check for argument validity
  if ( timeout > 65 )
    return I2C_INVALID_TIMEOUT;
    
  // interrupts must be enabled to use this function
  current_sr = __get_SR_register();
  if ( ((current_sr & GIE) == 0) && (num_bytes > 1) ) {
    int_enabled = 1;
    __enable_interrupt();
  }
      
  // configure driver variables
  i2c_actual_byte_count = 0;
  i2c_desired_byte_count = num_bytes;
  i2c_data_ptr = write_data;
  i2c_complete = 0;
  
  // turn on the timer
  TBCTL |= MC_2;
  i2c_baseline_timer = TBR;

  // write the slave address and clear reset
  UCB0CTL1 |= UCSWRST;
  UCB0I2CSA = device_addr>>1;
  UCB0CTL1 &= ~UCSWRST;

  // enable NAK and TX interrupts
  UCB0I2CIE = UCNACKIE;
  IE2 = UCB0TXIE;

  // start the transfer
  UCB0CTL1 |= (UCTR + UCTXSTT);
  
  // spin until variable is set
  while ( i2c_complete == 0 ) {
    status = i2c_master_check_nak_timeout(timeout, 0);
    
    if ( status != 0 )
      break;
  }
  
  if ( status == I2C_TIMEOUT ) {
    *bytes_written = i2c_actual_byte_count;
    ret_val = status;
  } else if ( i2c_return_val == I2C_NO_ACK ) {
    *bytes_written = 0;
    ret_val = I2C_NO_ACK;
  } else if ( i2c_return_val == 0 ) {
    *bytes_written = i2c_actual_byte_count;
    ret_val = 0;
  }
    
  // disable the interrupt if necessary
  if ( int_enabled )
    __disable_interrupt();
          
  return ret_val;
}

uint08 i2c_master_read(uint08 device_addr, uint08* read_data, uint08 num_bytes, uint08* bytes_read, uint08 timeout)
/**
 * Reads data from the specified device address
 *
 * @param   device_addr   - I - 7 Bit device Address
 * @param   read_data     - I - Pointer to buffer to hold received data from slave
 * @param   num_bytes     - I - Number of bytes to be read
 * @param   bytes_read    - O - Actual number of bytes read from slave
 * @param   timeout       - I - Timeout max (in msec) for the entire transaction
 *                              Timeout=0, permits an infinite timeout (not recommended). Max value = 65
 * @return  0 - Completed successfully 
 *          I2C_NO_ACK - Slave NAck'ed            
 *          I2C_TIMEOUT - Slave did not respond before timeout period expired
 *          I2C_INVALID_TIMEOUT - timeout parameter is greater than 65
 *
 */
{
  uint08 status = 0;
  uint08 ret_val;
  uint16 current_sr;
  uint08 int_enabled = 0;
  
  // check for argument validity
  if ( timeout > 65 )
    return I2C_INVALID_TIMEOUT;
    
  // interrupts must be enabled to use this function
  current_sr = __get_SR_register();
  if ( ((current_sr & GIE) == 0) && (num_bytes > 1) ) {
    int_enabled = 1;
    __enable_interrupt();
  }
    
  // turn on the timer
  TBCTL |= MC_2;
  
  // configure driver variables
  i2c_actual_byte_count = 0;
  i2c_desired_byte_count = num_bytes;
  i2c_data_ptr = read_data;
  i2c_complete = 0;
  i2c_baseline_timer = TBR;

  // write the slave address and kick off the transfer
  UCB0CTL1 |= UCSWRST;
  UCB0CTL1 &= ~UCTR;
  UCB0I2CSA = device_addr>>1;
  UCB0CTL1 &= ~UCSWRST;

  // enable NAK and RX interrupts
  UCB0I2CIE = UCNACKIE;
  IE2 = UCB0RXIE;

  if ( num_bytes == 1 ) {
    // spin until start condition cleared
    while ( UCB0CTL1 & UCTXSTT ) {
      status = i2c_master_check_nak_timeout(timeout, 0);
      
      if ( status != 0 ) {
        i2c_master_cleanup();
        if ( int_enabled )
          __disable_interrupt();
        return status;
      }
    }
    UCB0CTL1 |= UCTXSTP;
    while ( (IFG2 & UCB0RXIFG) == 0 ) {
      status = i2c_master_check_nak_timeout(timeout, 0);
      
      if ( status != 0 ) {
        i2c_master_cleanup();
        if ( int_enabled )
          __disable_interrupt();
        return status;
      }
    }
    *read_data = UCB0RXBUF;
    *bytes_read = 1;
  } else {
    // start the transfer
    UCB0CTL1 |= UCTXSTT;
  }
    
  // spin until variable is set 
  while ( i2c_complete == 0 ) {
    status = i2c_master_check_nak_timeout(timeout, 0);
    
    if ( status != 0 )
      break;
  }
  
  if ( status == I2C_TIMEOUT ) {
    *bytes_read = i2c_actual_byte_count;
    ret_val = status;
  } else if ( i2c_return_val == I2C_NO_ACK ) {
    *bytes_read = 0;
    ret_val = I2C_NO_ACK;
  } else if ( i2c_return_val == 0 ) {
    *bytes_read = i2c_actual_byte_count;
    ret_val = 0;
  }
    
  // disable the interrupt if necessary
  if ( int_enabled )
    __disable_interrupt();
          
  return ret_val;
}

uint08 i2c_master_check_nak_timeout(uint08 timeout, uint08 check_nak)
/**
 * 
 * @param   timeout   - I - Timeout max (in msec)
 *                          Timeout=0, permits an infinite timeout (not recommended). Max value = 65
 * @param   check_nak - I - Interrupt driven routines do not need to check for no acknowledge from
 *                          slave devices. 1=check for nak, 0=do not check for nak
 *
 * @return  0 - Completed successfully 
 *          I2C_NO_ACK - Slave NAck'ed            
 *          I2C_TIMEOUT - Slave did not respond before timeout period expired
 *
 */
{
  uint16 current_timer_val;
  uint16 timer_diff;
  
  // check for NAK
  if ( check_nak ) {
    if ( UCB0STAT & UCNACKIFG ) {
      // send a stop and clear the interrupt flag
      UCB0CTL1 |= UCTXSTP;
      UCB0STAT &= ~UCNACKIFG;
      TBCTL &= ~MC_2;
      UCB0CTL1 = UCSWRST;
      return I2C_NO_ACK;
    }
  }

  // do we need to check for timeout?
  if ( timeout != 0 ) {
    current_timer_val = TBR;

    // check for TBR rollover since we baselined
    if ( current_timer_val > i2c_baseline_timer )
      timer_diff = current_timer_val - i2c_baseline_timer;
    else
      timer_diff = (65535 - i2c_baseline_timer) + current_timer_val;
    
    // a count of 1000 is equal to 1ms, approximate with shift by 10
    // have we reached the timeout for this byte?
    if ( (timer_diff>>10) > timeout ) {
      // generate a stop and return the error flag
      TBCTL &= ~MC_2;
      UCB0CTL1 |= UCTXSTP;
      return I2C_TIMEOUT;
    }
  }
  
  // nak or timeout did not occur
  return 0;
}

void i2c_master_cleanup(void)
/**
 * Cleanup after a failed transaction. Turn off the timer, disable interrupts,
 * and put the I2C hardware into reset.
 *
 */
{
  // turn off the timer to save power
  TBCTL &= ~MC_2;
  
  // clear interrupt flags
  IFG2 &= ~UCB0TXIFG;
  
  // no errors, all bytes transmitted
  UCB0I2CIE &= ~UCNACKIE;
  IE2 &= ~UCB0TXIE;

  // put i2c hardware into reset  
  UCB0CTL1 = UCSWRST;
}

#pragma vector = USCIAB0RX_VECTOR
__interrupt void USCIAB0RX_ISR(void)
{
  // send a stop if the slave NAKs
  if ( UCB0STAT & UCNACKIFG ) {
    UCB0CTL1 |= UCTXSTP;
    UCB0STAT &= ~UCNACKIFG;
    
    // set variables for return from this driver
    i2c_return_val = I2C_NO_ACK;
    i2c_complete = 1;
  }
}

#pragma vector = USCIAB0TX_VECTOR
__interrupt void USCIAB0TX_ISR(void)
{
  // which interrupt occurred?
  if ( IFG2 & UCB0RXIFG ) {
     i2c_actual_byte_count++;
     if ( i2c_actual_byte_count == i2c_desired_byte_count ) {
      // send a stop
      UCB0CTL1 |= UCTXSTP;
      // get the byte out of the buffer      
      *i2c_data_ptr = UCB0RXBUF;
      
      // set variables for return from this driver
      i2c_return_val = 0;
      i2c_complete = 1;
      
      // clear interrupt enables
      UCB0I2CIE &= ~UCNACKIE;
      IE2 &= ~UCB0RXIE;
    } else {
      // get the byte out of the buffer      
      *i2c_data_ptr = UCB0RXBUF;
      i2c_data_ptr++;
    }
  }
  else if ( IFG2 & UCB0TXIFG ) {
    if ( i2c_actual_byte_count == i2c_desired_byte_count ) {
      // send a stop and clear the interrupt flag
      UCB0CTL1 |= UCTXSTP;
      IFG2 &= ~UCB0TXIFG;
      
      // set variables for return from this driver
      i2c_return_val = 0;
      i2c_complete = 1;
      
      // clear interrupt enables
      UCB0I2CIE &= ~UCNACKIE;
      IE2 &= ~UCB0TXIE;
    } else {
      // transfer another byte
      UCB0TXBUF = *i2c_data_ptr;
      i2c_data_ptr++;
      i2c_actual_byte_count++;
    }
  }
}

