/*
 * mcp9808.h
 *
 *  Created on: 31 mar 2024
 *      Author: user
 *
 *      https://www.microchip.com/en-us/product/mcp9808
 *      https://www.adafruit.com/product/1782
 *      https://www.seeedstudio.com/Grove-I2C-High-Accuracy-Temperature-Sensor-MCP9808.html
 *      https://github.com/adafruit/Adafruit_MCP9808_Library
 *      https://github.com/Seeed-Studio/Grove_Temperature_sensor_MCP9808
 *      https://embeddedespresso.com/temperature-measurement-never-so-easy-with-stm32-and-mcp9808/
 *
 */

#ifndef INC_MCP9808_H_
#define INC_MCP9808_H_

#define MCP9808_REG_RESOLUTION (0x08)

//#define MCP9808_I2C_ADDRESS (0x1F << 1) // default for DFRobot
#define MCP9808_I2C_ADDRESS (0x18 << 1) // default for Seed Studio

#define MCP9808_I2C_BUS (&hi2c3)

void Mcp9808SetResolution(uint8_t);
float Mcp9808GetTemperature(void);
uint8_t hysteresisCtrl(float control_error, float hysteresis_width);
uint32_t piCtrl(float ctrl_error, float T_sampling, float k_P, float k_I,
		float u_min, float u_max);

#endif /* INC_MCP9808_H_ */
