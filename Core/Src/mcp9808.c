/*
 * mcp9808.c
 *
 *  Created on: 31 mar 2024
 *      Author: user
 */

#include "main.h"
#include "i2c.h"
#include "mcp9808.h"
#include <math.h>

void Mcp9808SetResolution(uint8_t _resolution)
{
	// Mode Resolution SampleTime
	//  0    0.5 °C       30 ms
	//  1    0.25 °C      65 ms
	//  2    0.125 °C     130 ms
	//  3    0.0625 °C    250 ms
	uint8_t resolution = _resolution & 0b00000011;
	HAL_I2C_Mem_Write(MCP9808_I2C_BUS, MCP9808_I2C_ADDRESS,
	MCP9808_REG_RESOLUTION,
	I2C_MEMADD_SIZE_8BIT, &resolution, 1, 200u);

}

// https://embeddedespresso.com/temperature-measurement-never-so-easy-with-stm32-and-mcp9808/
float Mcp9808GetTemperature(void)
{

	uint8_t _tempReg = 0x05u; /* Temperature register address */
	uint8_t _dataReg[2]; /* Buffer for reading the register content */
	uint16_t _dataRegLong; /* Variable used to store the whole register content */
	float _tempVal = 0; /* Float variable used for storing the temperature value */
	float _tempValDec; /* Float variable used for calculation of the decimal part */
	/* Address the temperature register */
	HAL_I2C_Master_Transmit(MCP9808_I2C_BUS, MCP9808_I2C_ADDRESS, &_tempReg, 1,
			200u);
	/* Read the temperature register content */
	HAL_I2C_Master_Receive(MCP9808_I2C_BUS, MCP9808_I2C_ADDRESS | 0x01,
			_dataReg, 2, 200u);

	/* Compose the register content, regardless of the endianess */
	_dataRegLong = ((_dataReg[0] << 8u) | _dataReg[1]);

	/* Extract the integer part from the fixed point value */
	_tempVal = ((_dataRegLong & 0x0FFF) >> 4);

	/* Extract decimal part */
	_tempValDec = 0.0625;
	for (int i = 0; i < 4; i++)
	{
		_tempVal += ((_dataRegLong >> i) & 0x0001) * _tempValDec;
		_tempValDec *= 2u;
	}

	return _tempVal;
}

uint8_t hysteresisCtrl(float control_error, float hysteresis_width)
{
	static uint8_t control_signal = 0;

	if (control_error > hysteresis_width / 2)
	{
		control_signal = 1;
	}
	else if (control_error < -hysteresis_width / 2)
	{
		control_signal = 0;
	}
	else
	{
		control_signal = control_signal;
	}

	return control_signal;
}

uint32_t piCtrl(float ctrl_error, float T_sampling, float k_P, float k_I,
		float u_min, float u_max)
{

	float control_signal = 0;
	static float e_kminus1 = 0;
	static float u_kminus1 = 0;

	float delta_u = (k_P + T_sampling * k_I) * ctrl_error - k_P * e_kminus1;
	if (u_kminus1 + delta_u > u_max)
	{
		control_signal = u_max;
	}
	else if (u_kminus1 + delta_u < u_min)
	{
		control_signal = u_min;
	}
	else
	{
		control_signal = u_kminus1 + delta_u;
	}

	e_kminus1 = ctrl_error;
	u_kminus1 = control_signal;

	return (uint32_t) (round(control_signal));
}
