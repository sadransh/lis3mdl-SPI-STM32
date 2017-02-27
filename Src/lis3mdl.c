// based on Electric Imp drive https://github.com/electricimp/LIS3MDL/blob/master/LIS3MDL.class.nut

#include "lis3mdl.h"
#include <stdio.h>

#ifndef MIN
#  define MIN(X, Y) (((X) < (Y)) ? (X) : (Y))
#endif
#ifndef MAX
#  define MAX(X, Y) (((X) > (Y)) ? (X) : (Y))
#endif

#ifdef LIS3MDL_DEBUG
#define LIS3MDL_DEBUG_OUT(format, ...) printf("%s:%d: LIS3MDL: " format, __FILE__, __LINE__, ##__VA_ARGS__)
#else
#define LIS3MDL_DEBUG_OUT(format, ...)
#endif

#define _LIS3MDL_REG_WHO_AM_I     0x0F
#define _LIS3MDL_REG_CTL_1        0x20
#define _LIS3MDL_REG_CTL_2        0x21
#define _LIS3MDL_REG_CTL_3        0x22
#define _LIS3MDL_REG_CTL_4        0x23
#define _LIS3MDL_REG_STATUS       0x27
#define _LIS3MDL_REG_OUT_X_L      0x28
#define _LIS3MDL_REG_OUT_X_H      0x29
#define _LIS3MDL_REG_OUT_Y_L      0x2A
#define _LIS3MDL_REG_OUT_Y_H      0x2B
#define _LIS3MDL_REG_OUT_Z_L      0x2C
#define _LIS3MDL_REG_OUT_Z_H      0x2D
#define _LIS3MDL_REG_OUT_TEMP_L   0x2E
#define _LIS3MDL_REG_OUT_TEMP_H   0x2F

#define _LIS3MDL_REG_CTL_1_TEMP_EN 0b10000000

#define _LIS3MDL_REG_CTL_2_RESET   0b00000100

const static uint16_t _LIS3MDLGAUSS_TO_SCALE[] = { 4, 8, 12, 16 };

/**
 * @brief : low-level SPI Transmit and Receive functions
 * @param txdata : byte to send
 * @param rxdata : byte to read
 * @return HAL_StatusTypeDef
 * @author : Sadra Naddaf
 */
HAL_StatusTypeDef SPI_SendRecieveByte(LIS3MDL* lis3mdl,uint8_t txdata, uint8_t* rxdata) // This function is a standard old SPI function that writes and reads data
{
	HAL_StatusTypeDef status;
	status = HAL_SPI_TransmitReceive(&hspi3, &(txdata), rxdata, 1, 100); // Send One Byte
	return status;
}

/**
 * @brief : Setup lis3mdl sensor Settings such as spi Handler and etc.
 * @param lis3mdl : Structure Contain Settings
 * @param spi : SPI Settings Structure
 * @return : HAL_StatusTypeDef`
 * @author : Sadra Naddaf
 */
HAL_StatusTypeDef LIS3MDL_setup(LIS3MDL* lis3mdl, SPI_HandleTypeDef* spi,GPIO_TypeDef* csGPIO,uint16_t csPin) {
	lis3mdl->spi = spi;
	lis3mdl->GPIOx = csGPIO;
	lis3mdl->pin = csPin;
	LIS3MDL_clearMinMax(lis3mdl);
	return _LIS3MDL_init(lis3mdl);
}

void LIS3MDL_clearMinMax(LIS3MDL* lis3mdl) {
	for (int axis = 0; axis < 3; axis++) {
		lis3mdl->min[axis] = 32767;
		lis3mdl->max[axis] = -32768;
	}
}

void LIS3MDL_setMinMax(LIS3MDL* lis3mdl, uint8_t axis, int16_t min, int16_t max) {
	lis3mdl->min[axis] = min;
	lis3mdl->max[axis] = -max;
}

HAL_StatusTypeDef LIS3MDL_reset(LIS3MDL* lis3mdl) {
	HAL_StatusTypeDef status;
	status = _LIS3MDL_writeRegister(lis3mdl,_LIS3MDL_REG_CTL_2,
	_LIS3MDL_REG_CTL_2_RESET, _LIS3MDL_REG_CTL_2_RESET);
	if (status != HAL_OK) {
		return status;
	}
	return _LIS3MDL_init(lis3mdl);
}

HAL_StatusTypeDef LIS3MDL_enableTemperature(LIS3MDL* lis3mdl) {
	return _LIS3MDL_writeRegister(lis3mdl,_LIS3MDL_REG_CTL_1,
	_LIS3MDL_REG_CTL_1_TEMP_EN, _LIS3MDL_REG_CTL_1_TEMP_EN);
}

HAL_StatusTypeDef LIS3MDL_setPerformance(LIS3MDL* lis3mdl,uint8_t performance) {
	HAL_StatusTypeDef status;
	status = _LIS3MDL_writeRegister(lis3mdl,_LIS3MDL_REG_CTL_1, performance << 5,
			0b01100000);
	if (status != HAL_OK) {
		return status;
	}
	return _LIS3MDL_writeRegister(lis3mdl,_LIS3MDL_REG_CTL_4, performance << 2,
			0b00001100);
}

HAL_StatusTypeDef LIS3MDL_setDateRate(LIS3MDL* lis3mdl, uint8_t dataRate) {
	return _LIS3MDL_writeRegister(lis3mdl,_LIS3MDL_REG_CTL_1, dataRate << 2, 0b00011100);
}

HAL_StatusTypeDef LIS3MDL_setMode(LIS3MDL* lis3mdl,uint8_t mode) {
	return _LIS3MDL_writeRegister(lis3mdl,_LIS3MDL_REG_CTL_3, mode << 0, 0b00000011);
}

HAL_StatusTypeDef LIS3MDL_setScale(LIS3MDL* lis3mdl, uint8_t scale) {
	HAL_StatusTypeDef status;
	status = _LIS3MDL_writeRegister(lis3mdl,_LIS3MDL_REG_CTL_2, scale << 5, 0b01100000);
	if (status != HAL_OK) {
		return status;
	}

	lis3mdl->scale = _LIS3MDLGAUSS_TO_SCALE[scale];

	return HAL_OK;
}

HAL_StatusTypeDef LIS3MDL_readAxis(LIS3MDL* lis3mdl, uint8_t axis,
		int16_t* value) {
	HAL_StatusTypeDef status;
	uint8_t lowAddr, highAddr;
	switch (axis) {
	case LIS3MDL_AXIS_X:
		lowAddr = _LIS3MDL_REG_OUT_X_L;
		highAddr = _LIS3MDL_REG_OUT_X_H;
		break;
	case LIS3MDL_AXIS_Y:
		lowAddr = _LIS3MDL_REG_OUT_Y_L;
		highAddr = _LIS3MDL_REG_OUT_Y_H;
		break;
	case LIS3MDL_AXIS_Z:
		lowAddr = _LIS3MDL_REG_OUT_Z_L;
		highAddr = _LIS3MDL_REG_OUT_Z_H;
		break;
	default:
		return 0;
	}
	status = _LIS3MDL_readRegister_int16(lis3mdl,lowAddr, highAddr, value);
	if (status == HAL_OK) {
#ifdef LIS3MDL_DEBUG
		char axisCh = (axis == LIS3MDL_AXIS_X) ? 'x' : (axis == LIS3MDL_AXIS_Y ? 'y' : 'z');
		LIS3MDL_DEBUG_OUT("readAxis(%c) OK: %d\n", axisCh, *value);
#endif
		lis3mdl->max[axis] = MAX(lis3mdl->max[axis], *value);
		lis3mdl->min[axis] = MIN(lis3mdl->min[axis], *value);
	}
	return status;
}

HAL_StatusTypeDef LIS3MDL_readTemperature(LIS3MDL* lis3mdl,int16_t* value) {
	return _LIS3MDL_readRegister_int16(lis3mdl,_LIS3MDL_REG_OUT_TEMP_L,
	_LIS3MDL_REG_OUT_TEMP_H, value);
}

HAL_StatusTypeDef LIS3MDL_readStatus(LIS3MDL* lis3mdl,uint8_t *value) {
	return _LIS3MDL_readRegister(lis3mdl,_LIS3MDL_REG_STATUS, value);
}

HAL_StatusTypeDef _LIS3MDL_init(LIS3MDL* lis3mdl) {
	uint8_t deviceId;
	HAL_StatusTypeDef status;

	status = _LIS3MDL_readRegister(lis3mdl,_LIS3MDL_REG_WHO_AM_I,&deviceId);

	if (deviceId != LIS3MDL_DEVICE_ID) {
		LIS3MDL_DEBUG_OUT("invalid device id. expected 0x%02x, found: 0x%02x\n", LIS3MDL_DEVICE_ID, deviceId);
		return HAL_ERROR;
	} else{
		LIS3MDL_DEBUG_OUT("Connected to Lis3MDL, returned 0x%x",deviceId);
	}
	uint8_t reg2;
	status = _LIS3MDL_readRegister(lis3mdl,_LIS3MDL_REG_CTL_2, &reg2);

	lis3mdl->scale = _LIS3MDLGAUSS_TO_SCALE[(reg2 >> 5) & 0b11];
	printf("Scale Was Set as %d\n",lis3mdl->scale);
	return status;
}

HAL_StatusTypeDef _LIS3MDL_readRegister_int16(LIS3MDL* lis3mdl,uint8_t lowAddr, uint8_t* highAddr,
		int16_t* value) {

	HAL_StatusTypeDef status;
	uint8_t low, high;

	status = _LIS3MDL_readRegister(lis3mdl,lowAddr, &low);
	if (status != HAL_OK)
		return status;
	status = _LIS3MDL_readRegister(lis3mdl,highAddr, &high);
	if (status != HAL_OK)
		return status;
	*value = (((uint16_t) high) << 8) | (uint16_t) low;
	return status;
}
/**
 * @brief
 * @param lis3mdl
 * @param addr
 * @param value
 * @return
 */
HAL_StatusTypeDef _LIS3MDL_readRegister(LIS3MDL* lis3mdl,uint8_t addr, uint8_t* value) {
	HAL_StatusTypeDef status;
	LIS3MDL_CS_LOW(lis3mdl->GPIOx,lis3mdl->pin)
	; //CS low
	SPI_SendRecieveByte(lis3mdl,addr | 0x80, value);
	status = SPI_SendRecieveByte(lis3mdl,LIS3MDL_DUMMY, value);
	LIS3MDL_CS_HIGH(lis3mdl->GPIOx,lis3mdl->pin)
	; //CS high
	return status;
}

HAL_StatusTypeDef _LIS3MDL_writeRegister(LIS3MDL* lis3mdl,uint8_t addr, uint8_t val,
		uint8_t mask) {
	HAL_StatusTypeDef status;

	uint8_t x;
	uint8_t valuetoWrite;
	if (mask == 0xff) {
		valuetoWrite = val;
	} else {
		uint8_t currentValue;
		status = _LIS3MDL_readRegister(lis3mdl,addr, &currentValue);
		if (status != HAL_OK) {
			LIS3MDL_DEBUG_OUT("writeRegister: rx current error reg 0x%02x status %d\n", addr, status);
		}
		valuetoWrite = (currentValue & ~mask) | (val & mask);
	}
	LIS3MDL_CS_LOW(lis3mdl->GPIOx,lis3mdl->pin)
	; //CS low
	status = SPI_SendRecieveByte(lis3mdl,addr, &x); //Send address
	if (status != HAL_OK)
		return status;
	status = SPI_SendRecieveByte(lis3mdl,valuetoWrite, &x); //Send data
	if (status != HAL_OK)
		return status;
	LIS3MDL_CS_HIGH(lis3mdl->GPIOx,lis3mdl->pin)
	; //CS high
//	status = _LIS3MDL_readRegister(addr, &x);
	return status;
}

