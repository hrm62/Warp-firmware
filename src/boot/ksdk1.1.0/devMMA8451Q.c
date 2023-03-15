/*
	Authored 2016-2018. Phillip Stanley-Marbell. Additional contributors,
	2018-onwards, see git log.

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

/*
 *	config.h needs to come first
 */
#include "config.h"

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

#define	SAMPLE_WINDOW_LENGTH	50

// Made no difference
#pragma pack(1)

extern volatile WarpI2CDeviceState	deviceMMA8451QState;
extern volatile uint32_t		gWarpI2cBaudRateKbps;
extern volatile uint32_t		gWarpI2cTimeoutMilliseconds;
extern volatile uint32_t		gWarpSupplySettlingDelayMilliseconds;

void
initMMA8451Q(const uint8_t i2cAddress, uint16_t operatingVoltageMillivolts)
{
	deviceMMA8451QState.i2cAddress			= i2cAddress;
	deviceMMA8451QState.operatingVoltageMillivolts	= operatingVoltageMillivolts;

	return;
}

WarpStatus
writeSensorRegisterMMA8451Q(uint8_t deviceRegister, uint8_t payload)
{
	uint8_t		payloadByte[1], commandByte[1];
	i2c_status_t	status;

	switch (deviceRegister)
	{
		case 0x09: case 0x0a: case 0x0e: case 0x0f:
		case 0x11: case 0x12: case 0x13: case 0x14:
		case 0x15: case 0x17: case 0x18: case 0x1d:
		case 0x1f: case 0x20: case 0x21: case 0x23:
		case 0x24: case 0x25: case 0x26: case 0x27:
		case 0x28: case 0x29: case 0x2a: case 0x2b:
		case 0x2c: case 0x2d: case 0x2e: case 0x2f:
		case 0x30: case 0x31:
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
		.address = deviceMMA8451QState.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};

	warpScaleSupplyVoltage(deviceMMA8451QState.operatingVoltageMillivolts);
	commandByte[0] = deviceRegister;
	payloadByte[0] = payload;
	warpEnableI2Cpins();

	status = I2C_DRV_MasterSendDataBlocking(
							0 /* I2C instance */,
							&slave,
							commandByte,
							1,
							payloadByte,
							1,
							gWarpI2cTimeoutMilliseconds);
	if (status != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

WarpStatus
configureSensorMMA8451Q(uint8_t payloadF_SETUP, uint8_t payloadCTRL_REG1)
{
	WarpStatus	i2cWriteStatus1, i2cWriteStatus2;


	warpScaleSupplyVoltage(deviceMMA8451QState.operatingVoltageMillivolts);

	i2cWriteStatus1 = writeSensorRegisterMMA8451Q(kWarpSensorConfigurationRegisterMMA8451QF_SETUP /* register address F_SETUP */,
							payloadF_SETUP /* payload: Disable FIFO */
							);

	i2cWriteStatus2 = writeSensorRegisterMMA8451Q(kWarpSensorConfigurationRegisterMMA8451QCTRL_REG1 /* register address CTRL_REG1 */,
							payloadCTRL_REG1 /* payload */
							);

	return (i2cWriteStatus1 | i2cWriteStatus2);
}

WarpStatus
readSensorRegisterMMA8451Q(uint8_t deviceRegister, int numberOfBytes)
{
	uint8_t		cmdBuf[1] = {0xFF};
	i2c_status_t	status;


	USED(numberOfBytes);
	switch (deviceRegister)
	{
		case 0x00: case 0x01: case 0x02: case 0x03: 
		case 0x04: case 0x05: case 0x06: case 0x09:
		case 0x0a: case 0x0b: case 0x0c: case 0x0d:
		case 0x0e: case 0x0f: case 0x10: case 0x11:
		case 0x12: case 0x13: case 0x14: case 0x15:
		case 0x16: case 0x17: case 0x18: case 0x1d:
		case 0x1e: case 0x1f: case 0x20: case 0x21:
		case 0x22: case 0x23: case 0x24: case 0x25:
		case 0x26: case 0x27: case 0x28: case 0x29:
		case 0x2a: case 0x2b: case 0x2c: case 0x2d:
		case 0x2e: case 0x2f: case 0x30: case 0x31:
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
		.address = deviceMMA8451QState.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};

	warpScaleSupplyVoltage(deviceMMA8451QState.operatingVoltageMillivolts);
	cmdBuf[0] = deviceRegister;
	warpEnableI2Cpins();

	status = I2C_DRV_MasterReceiveDataBlocking(
							0 /* I2C peripheral instance */,
							&slave,
							cmdBuf,
							1,
							(uint8_t *)deviceMMA8451QState.i2cBuffer,
							numberOfBytes,
							gWarpI2cTimeoutMilliseconds);

	if (status != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

void
printSensorDataMMA8451Q(bool hexModeFlag)
{
	uint16_t	readSensorRegisterValueLSB;
	uint16_t	readSensorRegisterValueMSB;
	int16_t		readSensorRegisterValueCombined;
	WarpStatus	i2cReadStatus;


	warpScaleSupplyVoltage(deviceMMA8451QState.operatingVoltageMillivolts);

	/*
	 *	From the MMA8451Q datasheet:
	 *
	 *		"A random read access to the LSB registers is not possible.
	 *		Reading the MSB register and then the LSB register in sequence
	 *		ensures that both bytes (LSB and MSB) belong to the same data
	 *		sample, even if a new data sample arrives between reading the
	 *		MSB and the LSB byte."
	 *
	 *	We therefore do 2-byte read transactions, for each of the registers.
	 *	We could also improve things by doing a 6-byte read transaction.
	 */
	i2cReadStatus = readSensorRegisterMMA8451Q(kWarpSensorOutputRegisterMMA8451QOUT_X_MSB, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB = deviceMMA8451QState.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceMMA8451QState.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 6) | (readSensorRegisterValueLSB >> 2);

	/*
	 *	Sign extend the 14-bit value based on knowledge that upper 2 bit are 0:
	 */
	readSensorRegisterValueCombined = (readSensorRegisterValueCombined ^ (1 << 13)) - (1 << 13);

	int16_t xReading = readSensorRegisterValueCombined;

	// if (i2cReadStatus != kWarpStatusOK)
	// {
	// 	warpPrint(" ----,");
	// }
	// else
	// {
		
	// 	// if (hexModeFlag)
	// 	// {
	// 	// 	warpPrint(" 0x%02x 0x%02x", readSensorRegisterValueMSB, readSensorRegisterValueLSB);
	// 	// }
	// 	// else
	// 	// {
	// 	// 	warpPrint("X: %d\n", readSensorRegisterValueCombined);
	// 	// }
	// }

	i2cReadStatus = readSensorRegisterMMA8451Q(kWarpSensorOutputRegisterMMA8451QOUT_Y_MSB, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB = deviceMMA8451QState.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceMMA8451QState.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 6) | (readSensorRegisterValueLSB >> 2);

	/*
	 *	Sign extend the 14-bit value based on knowledge that upper 2 bit are 0:
	 */
	readSensorRegisterValueCombined = (readSensorRegisterValueCombined ^ (1 << 13)) - (1 << 13);

	int16_t yReading = readSensorRegisterValueCombined;
	
	if (i2cReadStatus != kWarpStatusOK)
	{
		warpPrint(" ----,");
	}
	else
	{
		
		// if (hexModeFlag)
		// {
		// 	warpPrint(" 0x%02x 0x%02x", readSensorRegisterValueMSB, readSensorRegisterValueLSB);
		// }
		// else
		// {
		// 	warpPrint("Y: %d\n", readSensorRegisterValueCombined);
		// }
	}

	i2cReadStatus = readSensorRegisterMMA8451Q(kWarpSensorOutputRegisterMMA8451QOUT_Z_MSB, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB = deviceMMA8451QState.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceMMA8451QState.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 6) | (readSensorRegisterValueLSB >> 2);

	/*
	 *	Sign extend the 14-bit value based on knowledge that upper 2 bit are 0:
	 */
	readSensorRegisterValueCombined = (readSensorRegisterValueCombined ^ (1 << 13)) - (1 << 13);

	int16_t zReading = readSensorRegisterValueCombined;

	// if (i2cReadStatus != kWarpStatusOK)
	// {
	// 	warpPrint(" ----,");
	// }
	// else
	// {
		
	// 	// if (hexModeFlag)
	// 	// {
	// 	// 	warpPrint(" 0x%02x 0x%02x,", readSensorRegisterValueMSB, readSensorRegisterValueLSB);
	// 	// }
	// 	// else
	// 	// {
	// 	// 	warpPrint("Z: %d\n", readSensorRegisterValueCombined);
	// 	// }
	// }

	// Note: inefficient, change back later or just don't use this function in final version.
	warpPrint("%d %d %d\n", xReading, yReading, zReading);
}

void
detectWalkingMMA8451Q()
{
	// warpPrint("test?");
	uint16_t	readSensorRegisterValueLSB;
	uint16_t	readSensorRegisterValueMSB;
	int16_t		readSensorRegisterValueCombined;
	WarpStatus	i2cReadStatus;
	// Initialise empty data structure to hold past 10 s of values
	int16_t xValues[SAMPLE_WINDOW_LENGTH];
	int16_t yValues[SAMPLE_WINDOW_LENGTH];
	int16_t zValues[SAMPLE_WINDOW_LENGTH];
	int8_t thresholds = 0;
	int16_t xMax;
	int16_t xMin;
	int16_t yMax;
	int16_t yMin;
	int16_t zMax;
	int16_t zMin;

	warpScaleSupplyVoltage(deviceMMA8451QState.operatingVoltageMillivolts);

	// warpPrint("Test before for loop\n");

	for (int j=0; j<10; j++)
	{
		xMax = -8191;
		xMin = 8191;
		yMax = -8191;
		yMin = 8191;
		zMax = -8191;
		zMin = 8191;
		for (int i=0; i<SAMPLE_WINDOW_LENGTH; i++)
		{
			// warpPrint("test inside for loop %d", i);
			// Read sample i from x-axis
			i2cReadStatus = readSensorRegisterMMA8451Q(kWarpSensorOutputRegisterMMA8451QOUT_X_MSB, 2 /* numberOfBytes */);
			readSensorRegisterValueMSB = deviceMMA8451QState.i2cBuffer[0];
			readSensorRegisterValueLSB = deviceMMA8451QState.i2cBuffer[1];
			readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 6) | (readSensorRegisterValueLSB >> 2);

			/*
			*	Sign extend the 14-bit value based on knowledge that upper 2 bit are 0:
			*/
			// warpPrint("test before assigning to xvalues");
			// Sum and divide by 4
			if (i >= 3)
			{
				xValues[i] = ((readSensorRegisterValueCombined ^ (1 << 13)) - (1 << 13) + xValues[i-1] + xValues[i-2] + xValues[i-3]) >> 2;
			} else
			{
				xValues[i] = (readSensorRegisterValueCombined ^ (1 << 13)) - (1 << 13);
			}

			// warpPrint("test after xvalues");
			if (xValues[i] > xMax)
			{
				xMax = xValues[i];
			}
			if (xValues[i] < xMin)
			{
				xMin = xValues[i];
			}

			// Read sample i from y-axis
			i2cReadStatus = readSensorRegisterMMA8451Q(kWarpSensorOutputRegisterMMA8451QOUT_Y_MSB, 2 /* numberOfBytes */);
			readSensorRegisterValueMSB = deviceMMA8451QState.i2cBuffer[0];
			readSensorRegisterValueLSB = deviceMMA8451QState.i2cBuffer[1];
			readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 6) | (readSensorRegisterValueLSB >> 2);

			/*
			*	Sign extend the 14-bit value based on knowledge that upper 2 bit are 0:
			*/
			if (i >= 3)
			{
				yValues[i] = ((readSensorRegisterValueCombined ^ (1 << 13)) - (1 << 13) + yValues[i-1] + yValues[i-2] + yValues[i-3]) >> 2;
			} else
			{
				yValues[i] = (readSensorRegisterValueCombined ^ (1 << 13)) - (1 << 13);
			}

			if (yValues[i] > yMax)
			{
				yMax = yValues[i];
			}
			if (yValues[i] < yMin)
			{
				yMin = yValues[i];
			}

			// Read sample i from z-axis
			i2cReadStatus = readSensorRegisterMMA8451Q(kWarpSensorOutputRegisterMMA8451QOUT_Z_MSB, 2 /* numberOfBytes */);
			readSensorRegisterValueMSB = deviceMMA8451QState.i2cBuffer[0];
			readSensorRegisterValueLSB = deviceMMA8451QState.i2cBuffer[1];
			readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 6) | (readSensorRegisterValueLSB >> 2);

			/*
			*	Sign extend the 14-bit value based on knowledge that upper 2 bit are 0:
			*/
			if (i >= 3)
			{
				zValues[i] = ((readSensorRegisterValueCombined ^ (1 << 13)) - (1 << 13) + zValues[i-1] + zValues[i-2] + zValues[i-3]) >> 2;
			} else
			{
				zValues[i] = (readSensorRegisterValueCombined ^ (1 << 13)) - (1 << 13);
			}

			if (zValues[i] > zMax)
			{
				zMax = zValues[i];
			}
			if (zValues[i] < zMin)
			{
				zMin = zValues[i];
			}

			// Check this is executing
			if (i2cReadStatus != kWarpStatusOK)
			{
				warpPrint(" ----,");
			}
			else
			{
				warpPrint("\n%d %d %d %d %d %d %d %d %d\t", xValues[i], yValues[i], zValues[i], xMax, xMin, yMax, yMin, zMax, zMin);
			}

			// Sample at 10 Hz - wait for 100 ms
			OSA_TimeDelay(20);
		}

		// warpPrint("Out of loop");
		// TODO: make this rolling - this will not be very time-efficient as-is
		// Calculate min and max
		// Axis with highest range (max-min) is the axis of maximum action
		if (xMax - xMin > yMax - yMin && xMax - xMin > zMax - zMin)
		{
			// Calculate the threshold
			// As int not float to save memory - did not work
			int16_t threshold = (xMax + xMin) >> 1;
			// Identify number of negative gradient threshold crossings
			for (int i=1; i<SAMPLE_WINDOW_LENGTH; i++)
			{
				// Difference must be greater than 1000 - early attempt to ignore variation due to vibrations and count each peak once
				// Need better filtering
				// Adding the i+1 part to specify that only large, sharp peaks are to be considered - not working as intended
				// reads 0 or a very low number of steps when walking
				if (xValues[i] < threshold && xValues[i-1] > threshold && xValues[i-1] - xValues[i] > 50)// && xValues[i+1] - xValues[i] > 1000)
				{
					thresholds++;
				}
			}
		}
		else if (yMax - yMin > zMax - zMin)
		{
			// Calculate the threshold
			// As int not float to save memory - did not work
			int16_t threshold = (yMax + yMin) >> 1;
			// Identify number of negative gradient threshold crossings
			for (int i=1; i<SAMPLE_WINDOW_LENGTH; i++)
			{
				if (yValues[i] < threshold && yValues[i-1] > threshold && yValues[i-1] - yValues[i] > 50)// && yValues[i+1] - yValues[i] > 1000)
				{
					thresholds++;
				}
			}
		}
		else
		{
			// Calculate the threshold
			// As int not float to save memory - did not work
			int16_t threshold = (zMax + zMin) >> 1;
			// Identify number of negative gradient threshold crossings
			// If a threshold crossing occurs on the boundary of two windows this step will be missed
			for (int i=1; i<SAMPLE_WINDOW_LENGTH; i++)
			{
				if (zValues[i] < threshold && zValues[i-1] > threshold && zValues[i-1] - zValues[i] > 50)// && zValues[i+1] - zValues[i] > 1000)
				{
					thresholds++;
				}
			}
		}
	}

	// Not sure this is the right thing to do?
	int8_t step_uncertainty = (int8_t) thresholds * 0.7;
	if (step_uncertainty < 1)
	{
		step_uncertainty = 1;	// 1 as Placeholder
	}

	warpPrint("Steps: %d +/- %d\t", thresholds, step_uncertainty);

	// Make classification decision (and calculate uncertainty/walking)
	// For now:
	// Start with the assumption that they are all equally likely
	int8_t walking_pc = 50;
	int8_t rest_pc = 50;
	int8_t other_pc = 0;
	// int8_t unknown_pc = 34;
	// Walking probability increases if number of steps detected in 10 s is at a value
	// consistent with walking pace
	switch (thresholds)
	{
		case 2:
			walking_pc = walking_pc + 5;
			rest_pc = 100 - walking_pc;
		
		case 3:
			walking_pc = walking_pc + 11;
			rest_pc = 100 - walking_pc;
		
		case 4:
			walking_pc = walking_pc + 33;
			rest_pc = 100 - walking_pc;

		case 5:
			walking_pc = walking_pc + 21;
			rest_pc = 100 - walking_pc;
		
		case 6:
			walking_pc = walking_pc + 18;
			rest_pc = 100 - walking_pc;
		
		case 7:
			walking_pc = walking_pc + 9;
			rest_pc = 100 - walking_pc;

		case 8:
			walking_pc = walking_pc + 4;
			rest_pc = 100 - walking_pc;
	}

	// Outside of range of normal walking paces
	if (thresholds < 2)
	{
		rest_pc = 100;
		walking_pc = 0;
	}
	if (thresholds > 8)
	{
		rest_pc = 0;
		walking_pc = 0;
		other_pc = 100;
	}

	if (walking_pc >= rest_pc && walking_pc >= other_pc)
	{
		warpPrint("Walking %d%% \n", walking_pc);
	} else if (rest_pc >= other_pc)
	{
		warpPrint("Rest %d%% \n", rest_pc);
	} else
	{
		warpPrint("Other %d%% \n", other_pc);
	}
}
