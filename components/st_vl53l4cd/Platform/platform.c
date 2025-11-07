/*
 Copyright (c) 2021, STMicroelectronics - All Rights Reserved

 This file : part of VL53L4CD Ultra Lite Driver and : dual licensed, either
 'STMicroelectronics Proprietary license'
 or 'BSD 3-clause "New" or "Revised" License' , at your option.

*******************************************************************************

 'STMicroelectronics Proprietary license'

*******************************************************************************

 License terms: STMicroelectronics Proprietary in accordance with licensing
 terms at www.st.com/sla0081

 STMicroelectronics confidential
 Reproduction and Communication of this document : strictly prohibited unless
 specifically authorized in writing by STMicroelectronics.


*******************************************************************************

 Alternatively, VL53L4CD Ultra Lite Driver may be distributed under the terms of
 'BSD 3-clause "New" or "Revised" License', in which case the following
 provisions apply instead of the ones mentioned above :

*******************************************************************************

 License terms: BSD 3-clause "New" or "Revised" License.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice, this
 list of conditions and the following disclaimer.

 2. Redistributions in binary form must reproduce the above copyright notice,
 this list of conditions and the following disclaimer in the documentation
 and/or other materials provided with the distribution.

 3. Neither the name of the copyright holder nor the names of its contributors
 may be used to endorse or promote products derived from this software
 without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************
*/

/*******************************************************************************
 * Copyright 2024, LooUQ Incorporated
 * 
 * ESP-IDF platform abstraction and make directives are copyright LooUQ 
 * Incorporated. 
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the above conditions are met. 
 * Including the inclusion of this notice.
 * 
 * Licensed under the MIT open source license.
*******************************************************************************/

#include "platform.h"

#include <string.h>
#include <time.h>
#include <math.h>


#include "driver/i2c_master.h"
#include "driver/i2c_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"


static i2c_master_dev_handle_t devHndl = NULL;

void VL53L4CD_SetDeviceHandle(i2c_master_dev_handle_t esp_DeviceHandle)
{
    devHndl = esp_DeviceHandle;
}


/* ST platform 
 * --------------------------------------------------------------------------------------------- */

uint8_t VL53L4CD_RdDWord(Dev_t dev, uint16_t RegisterAdress, uint32_t *value)
{
    ESP_ERROR_CHECK(devHndl == NULL || dev != TOF_ESP);

    uint8_t indxBffr[2] = {0}; 
    indxBffr[0] = RegisterAdress >> 8; 
    indxBffr[1] = RegisterAdress & 0xff;

    uint8_t valueBffr[4];
    ESP_ERROR_CHECK(i2c_master_transmit_receive(devHndl, indxBffr, 2, valueBffr, 4, VL53L4CD_DEFAULT_TIMEOUT));
    *value = valueBffr[0] << 24 | valueBffr[1] << 16 | valueBffr[2] << 8 | valueBffr[3];
	return 0;
}


uint8_t VL53L4CD_RdWord(Dev_t dev, uint16_t RegisterAdress, uint16_t *value)
{
    ESP_ERROR_CHECK(devHndl == NULL || dev != TOF_ESP);

    uint8_t indxBffr[2] = {0}; 
    indxBffr[0] = RegisterAdress >> 8; 
    indxBffr[1] = RegisterAdress & 0xff;

    uint8_t valueBffr[2];
    ESP_ERROR_CHECK(i2c_master_transmit_receive(devHndl, indxBffr, 2, valueBffr, 2, VL53L4CD_DEFAULT_TIMEOUT));
    *value = valueBffr[0] << 8 | valueBffr[1];
	return 0;
}


uint8_t VL53L4CD_RdByte(Dev_t dev, uint16_t RegisterAdress, uint8_t *value)
{
    ESP_ERROR_CHECK(devHndl == NULL || dev != TOF_ESP);

    uint8_t indxBffr[2] = {0}; 
    indxBffr[0] = RegisterAdress >> 8; 
    indxBffr[1] = RegisterAdress & 0xff;

    ESP_ERROR_CHECK(i2c_master_transmit_receive(devHndl, indxBffr, 2, value, 1, VL53L4CD_DEFAULT_TIMEOUT));
	return 0;
}

uint8_t VL53L4CD_WrByte(Dev_t dev, uint16_t RegisterAdress, uint8_t value)
{
    ESP_ERROR_CHECK(devHndl == NULL || dev != TOF_ESP);

    uint8_t wrBffr[3];
    wrBffr[0] = RegisterAdress >> 8; 
    wrBffr[1] = RegisterAdress & 0xff;
    wrBffr[2] = value;

    ESP_ERROR_CHECK(i2c_master_transmit(devHndl, wrBffr, 3, VL53L4CD_DEFAULT_TIMEOUT));
	return 0;
}


uint8_t VL53L4CD_WrWord(Dev_t dev, uint16_t RegisterAdress, uint16_t value)
{
    ESP_ERROR_CHECK(devHndl == NULL || dev != TOF_ESP);

    uint8_t wrBffr[4];
    wrBffr[0] = RegisterAdress >> 8; 
    wrBffr[1] = RegisterAdress & 0xff;
    wrBffr[2] = value >> 8; 
    wrBffr[3] = value & 0xff;

    ESP_ERROR_CHECK(i2c_master_transmit(devHndl, wrBffr, 4, VL53L4CD_DEFAULT_TIMEOUT));
	return 0;
}


uint8_t VL53L4CD_WrDWord(Dev_t dev, uint16_t RegisterAdress, uint32_t value)
{
    ESP_ERROR_CHECK(devHndl == NULL || dev != TOF_ESP);

    uint8_t wrBffr[6];
    wrBffr[0] = RegisterAdress >> 8; 
    wrBffr[1] = RegisterAdress & 0xff;
    wrBffr[2] = value >> 24; 
    wrBffr[3] = value >> 16;
    wrBffr[4] = value >> 8; 
    wrBffr[5] = value & 0xff;

    ESP_ERROR_CHECK(i2c_master_transmit(devHndl, wrBffr, 4, VL53L4CD_DEFAULT_TIMEOUT));
	return 0;
}

uint8_t WaitMs(Dev_t dev, uint32_t TimeMs)
{
    // dependent on sdkConfig (menuConfig) for portTICK_PERIOD_MS
    vTaskDelay(TimeMs / portTICK_PERIOD_MS);
	return 0;
}
