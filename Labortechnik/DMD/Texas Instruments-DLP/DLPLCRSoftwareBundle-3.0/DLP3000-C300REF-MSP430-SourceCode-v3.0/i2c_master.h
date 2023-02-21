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

#ifndef __i2c_master
#define __i2c_master

#ifdef __cplusplus
extern "C" {
#endif

// defines
#define I2C_NO_ACK                 1 
#define I2C_TIMEOUT                2
#define I2C_INVALID_TIMEOUT        3

// functions
void i2c_master_init(void);
void i2c_master_set_clockrate(uint32 rate);
void i2c_master_config_timer(void);
uint08 i2c_master_polled_write(uint08 device_addr, uint08* write_data, uint08 num_bytes, uint08* bytes_written, uint08 timeout);
uint08 i2c_master_polled_read(uint08 device_addr, uint08* read_data, uint08 num_bytes, uint08* bytes_read, uint08 timeout);
uint08 i2c_master_polled_write_restart_read(uint08 device_addr, uint08* write_data, uint08 num_write_bytes, uint08* bytes_written, uint08* read_data, uint08 num_read_bytes, uint08* bytes_read, uint08 timeout);
uint08 i2c_master_write(uint08 device_addr, uint08* write_data, uint08 num_bytes, uint08* bytes_written, uint08 timeout);
uint08 i2c_master_read(uint08 device_addr, uint08* read_data, uint08 num_bytes, uint08* bytes_read, uint08 timeout);

#ifdef __cplusplus      /* matches __cplusplus construct above */
}
#endif

#endif // #ifndef __i2c_master
