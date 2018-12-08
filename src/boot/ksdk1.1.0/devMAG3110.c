/*
	Authored 2016-2018. Phillip Stanley-Marbell.

	All rights reserved.

	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions
	are met:

	*	Redistributions of source code must retain the above
		copyright notice, this list of conditions and the following
		disclaimer.

	*	Redistributions in binary form must reproduce the above
		copyright notice, this list of conditions and the following
		disclaimer in the documentation and/or other materials
		provided with the distribution.

	*	Neither the name of the author nor the names of its
		contributors may be used to endorse or promote products
		derived from this software without specific prior written
		permission.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
	"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
	LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
	FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
	COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
	INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
	BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
	CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
	LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
	ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
	POSSIBILITY OF SUCH DAMAGE.
*/
#include <stdlib.h>

#include "fsl_misc_utilities.h"
#include "fsl_device_registers.h"
#include "fsl_i2c_master_driver.h"
#include "fsl_spi_master_driver.h"
#include "fsl_rtc_driver.h"
#include "fsl_clock_manager.h"
#include "fsl_power_manager.h"
#include "fsl_mcglite_hal.h"
#include "fsl_port_hal.h"

#include "gpio_pins.h"
#include "SEGGER_RTT.h"
#include "warp.h"


extern volatile WarpI2CDeviceState	deviceMAG3110State;
extern volatile uint32_t		gWarpI2cBaudRateKbps;



void
initMAG3110(const uint8_t i2cAddress, WarpI2CDeviceState volatile *  deviceStatePointer)
{
	deviceStatePointer->i2cAddress	= i2cAddress;
	deviceStatePointer->signalType	= (
						kWarpTypeMaskMagneticX |
						kWarpTypeMaskMagneticY |
						kWarpTypeMaskMagneticZ |
						kWarpTypeMaskTemperature
					);

	return;
}

WarpStatus
writeSensorRegisterMAG3110(uint8_t deviceRegister, uint8_t payload, uint16_t menuI2cPullupValue)
{
	uint8_t		payloadByte[1], commandByte[1];
	i2c_status_t	returnValue;

	switch (deviceRegister)
	{
		case 0x09: case 0x0A: case 0x0B: case 0x0C:
		case 0x0D: case 0x0E: case 0x10: case 0x11:
		{
			/* OK */
			break;
		}
		
		default:
		{
			return kWarpStatusBadDeviceCommand;
		}
	}

	i2c_device_t slave =
	{
		.address = deviceMAG3110State.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};

	enableI2Cpins(menuI2cPullupValue);

	/*
	 *	Wait for supply and pull-ups to settle.
	 */
	OSA_TimeDelay(100);

	commandByte[0] = deviceRegister;
	payloadByte[0] = payload;
	returnValue = I2C_DRV_MasterSendDataBlocking(
							0 /* I2C instance */,
							&slave,
							commandByte,
							1,
							payloadByte,
							1,
							1000);
	if (returnValue != kStatus_I2C_Success)
	{
		SEGGER_RTT_printf(0, "\r\n\tI2C write failed, error %d.\n\n", returnValue);
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

WarpStatus
configureSensorMAG3110(uint8_t payloadCTRL_REG1, uint8_t payloadCTRL_REG2, uint8_t menuI2cPullupValue)
{
	i2c_status_t	returnValue;
	returnValue = writeSensorRegisterMAG3110(0x10 /* register address CTRL_REG1 */,
							payloadCTRL_REG1 /* payload */,
							menuI2cPullupValue);
	if (returnValue != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	returnValue = writeSensorRegisterMAG3110(0x11 /* register address CTRL_REG2 */,
							payloadCTRL_REG2 /* payload */,
							menuI2cPullupValue);
	if (returnValue != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

WarpStatus
readSensorRegisterMAG3110(uint8_t deviceRegister)
{
	uint8_t		cmdBuf[1]	= {0xFF};
	i2c_status_t	returnValue;


	i2c_device_t slave =
	{
		.address = deviceMAG3110State.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};


	/*
	 *	Steps (Repeated single-byte read. See Section 4.2.2 of MAG3110 manual.):
	 *
	 *	(1) Write transaction beginning with start condition, slave address, and pointer address.
	 *
	 *	(2) Read transaction beginning with start condition, followed by slave address, and read 1 byte payload
	*/

	cmdBuf[0] = deviceRegister;

	returnValue = I2C_DRV_MasterSendDataBlocking(
							0 /* I2C peripheral instance */,
							&slave,
							cmdBuf,
							1,
							NULL,
							0,
							500 /* timeout in milliseconds */);

	//SEGGER_RTT_printf(0, "\r\nI2C_DRV_MasterSendDataBlocking returned [%d] (set pointer)\n", returnValue);

	returnValue = I2C_DRV_MasterReceiveDataBlocking(
							0 /* I2C peripheral instance */,
							&slave,
							cmdBuf,
							1,
							(uint8_t *)deviceMAG3110State.i2cBuffer,
							1,
							500 /* timeout in milliseconds */);

	//SEGGER_RTT_printf(0, "\r\nI2C_DRV_MasterReceiveData returned [%d] (read register)\n", returnValue);

	if (returnValue == kStatus_I2C_Success)
	{
		//SEGGER_RTT_printf(0, "\r[0x%02x]	0x%02x\n", cmdBuf[0], deviceMAG3110State.i2cBuffer[0]);
	}
	else
	{
		//SEGGER_RTT_printf(0, kWarpConstantStringI2cFailure, cmdBuf[0], returnValue);

		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

void
printSensorDataMAG3110(void)
{
	uint8_t readSensorRegisterValueLSB;
	uint8_t readSensorRegisterValueMSB;
	uint16_t readSensorRegisterValueCombined;

	readSensorRegisterMAG3110(0x01);
	readSensorRegisterValueMSB = deviceMAG3110State.i2cBuffer[0];
	readSensorRegisterMAG3110(0x02);
	readSensorRegisterValueLSB = deviceMAG3110State.i2cBuffer[0];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF)<<8) + (readSensorRegisterValueLSB & 0xFF);
	SEGGER_RTT_printf(0, " %d,",readSensorRegisterValueCombined);
	
	readSensorRegisterMAG3110(0x03);
	readSensorRegisterValueMSB = deviceMAG3110State.i2cBuffer[0];
	readSensorRegisterMAG3110(0x04);
	readSensorRegisterValueLSB = deviceMAG3110State.i2cBuffer[0];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF)<<8) + (readSensorRegisterValueLSB & 0xFF);
	SEGGER_RTT_printf(0, " %d,",readSensorRegisterValueCombined);
	
	readSensorRegisterMAG3110(0x05);
	readSensorRegisterValueMSB = deviceMAG3110State.i2cBuffer[0];
	readSensorRegisterMAG3110(0x06);
	readSensorRegisterValueLSB = deviceMAG3110State.i2cBuffer[0];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF)<<8) + (readSensorRegisterValueLSB & 0xFF);
	SEGGER_RTT_printf(0, " %d,",readSensorRegisterValueCombined);
}