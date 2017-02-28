#ifndef _LIS3MDL_H_
#define _LIS3MDL_H_

//#include <platform_config.h>
#include <stm32f4xx_hal.h>
#include <stdbool.h>
#include <stdint.h>

#define LIS3MDL_ADDRESS1  (0b0011110 << 1)
#define LIS3MDL_ADDRESS2  (0b0011100 << 1)

#define LIS3MDL_PERFORMANCE_LOW_POWER  0b00
#define LIS3MDL_PERFORMANCE_MEDIUM     0b01
#define LIS3MDL_PERFORMANCE_HIGH       0b10
#define LIS3MDL_PERFORMANCE_ULTRA_HIGH 0b11

#define LIS3MDL_DATA_RATE_0_625_HZ 0b000
#define LIS3MDL_DATA_RATE_1_25_HZ  0b001
#define LIS3MDL_DATA_RATE_2_5_HZ   0b010
#define LIS3MDL_DATA_RATE_5_HZ     0b011
#define LIS3MDL_DATA_RATE_10_HZ    0b100
#define LIS3MDL_DATA_RATE_20_HZ    0b101
#define LIS3MDL_DATA_RATE_40_HZ    0b110
#define LIS3MDL_DATA_RATE_80_HZ    0b111

#define LIS3MDL_MODE_CONTINUOUS    0b00
#define LIS3MDL_MODE_SINGLE        0b01
#define LIS3MDL_MODE_POWER_DOWN    0b11

#define LIS3MDL_SCALE_4_GAUSS      0b00
#define LIS3MDL_SCALE_8_GAUSS      0b01
#define LIS3MDL_SCALE_12_GAUSS     0b10
#define LIS3MDL_SCALE_16_GAUSS     0b11

#define LIS3MDL_AXIS_X             0
#define LIS3MDL_AXIS_Y             1
#define LIS3MDL_AXIS_Z             2

#define LIS3MDL_STATUS_ZYXOR       0b10000000
#define LIS3MDL_STATUS_ZOR         0b01000000
#define LIS3MDL_STATUS_YOR         0b00100000
#define LIS3MDL_STATUS_XOR         0b00010000
#define LIS3MDL_STATUS_ZYXDA       0b00001000
#define LIS3MDL_STATUS_ZDA         0b00000100
#define LIS3MDL_STATUS_YDA         0b00000010
#define LIS3MDL_STATUS_XDA         0b00000001
#define LIS3MDL_DUMMY			   0x00
#define LIS3MDL_DEVICE_ID          0b00111101

#define LIS3MDL_CSN_GPIO     			SPI3CS8_GPIO_Port			// You need to modify this according to your board connection
#define LIS3MDL_CSN_PIN      			SPI3CS8_Pin		// You need to modify this according to your board connection
#define LIS3MDL_CS_LOW(port,pin)					HAL_GPIO_WritePin(port,pin,0); //Select sensor
#define LIS3MDL_CS_HIGH(port,pin)					HAL_GPIO_WritePin(port,pin,1); // deSelect sensor

typedef struct {
	SPI_HandleTypeDef* spi;
	GPIO_TypeDef* port;
	uint16_t pin;
	uint16_t scale;
	int16_t min[3];
	int16_t max[3];
} LIS3MDL;



HAL_StatusTypeDef _LIS3MDL_readRegister(LIS3MDL* lis3mdl, uint8_t addr,
		uint8_t* value);
HAL_StatusTypeDef _LIS3MDL_writeRegister(LIS3MDL* lis3mdl, uint8_t addr,
		uint8_t val, uint8_t mask);
HAL_StatusTypeDef SPI_SendRecieveByte(LIS3MDL* lis3mdl, uint8_t txdata,
		uint8_t *rxdata); // This function is a standard old SPI function that writes and reads data
HAL_StatusTypeDef LIS3MDL_setup(LIS3MDL* lis3mdl, SPI_HandleTypeDef* spi,
		GPIO_TypeDef* gpio, uint16_t _pin);
void LIS3MDL_clearMinMax(LIS3MDL* lis3mdl);
void LIS3MDL_setMinMax(LIS3MDL* lis3mdl, uint8_t axis, int16_t min, int16_t max);
HAL_StatusTypeDef LIS3MDL_reset(LIS3MDL* lis3mdl);
HAL_StatusTypeDef LIS3MDL_enableTemperature(LIS3MDL* lis3mdl);
HAL_StatusTypeDef LIS3MDL_setPerformance(LIS3MDL* lis3mdl, uint8_t performance);
HAL_StatusTypeDef LIS3MDL_setDateRate(LIS3MDL* lis3mdl, uint8_t dataRate);
HAL_StatusTypeDef LIS3MDL_setMode(LIS3MDL* lis3mdl, uint8_t mode);
HAL_StatusTypeDef LIS3MDL_setScale(LIS3MDL* lis3mdl, uint8_t scale);
HAL_StatusTypeDef LIS3MDL_readAxis(LIS3MDL* lis3mdl, uint8_t axis,
		int16_t* value);
HAL_StatusTypeDef LIS3MDL_readTemperature(LIS3MDL* lis3mdl, int16_t* value);
HAL_StatusTypeDef LIS3MDL_readStatus(LIS3MDL* lis3mdl, uint8_t *value);
HAL_StatusTypeDef _LIS3MDL_init(LIS3MDL* lis3mdl);
HAL_StatusTypeDef _LIS3MDL_readRegister_int16(LIS3MDL* lis3mdl,
		uint8_t* lowAddr, uint8_t* highAddr, int16_t* value);
#endif
