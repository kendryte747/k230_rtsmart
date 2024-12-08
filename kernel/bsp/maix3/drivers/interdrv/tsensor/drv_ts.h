/* Copyright (c) 2023, Canaan Bright Sight Co., Ltd
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __DRV_TS__
#define __DRV_TS__

#include <rtdef.h>

#define RT_DEVICE_TS_CTRL_SET_MODE          0x01
#define RT_DEVICE_TS_CTRL_GET_MODE          0x02
#define RT_DEVICE_TS_CTRL_SET_TRIM          0x03
#define RT_DEVICE_TS_CTRL_GET_TRIM          0x04

#define RT_DEVICE_TS_CTRL_MODE_SINGLE       0x01
#define RT_DEVICE_TS_CTRL_MODE_CONTINUUOS   0x02

void tsensor_init(void);

void tsensor_set_trim(uint8_t trim);
uint8_t tsensor_get_trim(void);

void tsensor_set_mode(uint8_t mode);
uint8_t tsensor_get_mode(void);

int tsensor_read_temp(double *temp);

#endif /*__DRV_TS__*/
