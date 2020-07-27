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

HAL_StatusTypeDef _LIS3MDL_init(LIS3MDL* lis3mdl);
HAL_StatusTypeDef _LIS3MDL_readRegister(LIS3MDL* lis3mdl, uint8_t reg,
		uint8_t* value);
HAL_StatusTypeDef _LIS3MDL_writeRegister(LIS3MDL* lis3mdl, uint8_t reg,
		uint8_t data, uint8_t mask);
HAL_StatusTypeDef _LIS3MDL_readRegister_int16(LIS3MDL* lis3mdl, uint8_t lowAddr,
		uint8_t highAddr, int16_t* value);

HAL_StatusTypeDef LIS3MDL_setup(LIS3MDL* lis3mdl, I2C_HandleTypeDef* i2c,
		GPIO_TypeDef* gpio, uint16_t _pin, uint8_t address) {
	lis3mdl->i2c = i2c;
	lis3mdl->port = gpio;
	lis3mdl->pin = _pin;
	lis3mdl->address = address;

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

	status = _LIS3MDL_writeRegister(lis3mdl, _LIS3MDL_REG_CTL_2,
			_LIS3MDL_REG_CTL_2_RESET, _LIS3MDL_REG_CTL_2_RESET);
	if (status != HAL_OK) {
		return status;
	}
	return _LIS3MDL_init(lis3mdl);
}

HAL_StatusTypeDef LIS3MDL_enableTemperature(LIS3MDL* lis3mdl, bool enable) {
	HAL_StatusTypeDef s;
	LIS3MDL_CS_HIGH(lis3mdl->port, lis3mdl->pin);
	s = _LIS3MDL_writeRegister(lis3mdl, _LIS3MDL_REG_CTL_1,
			_LIS3MDL_REG_CTL_1_TEMP_EN, _LIS3MDL_REG_CTL_1_TEMP_EN);
	return s;
}

HAL_StatusTypeDef LIS3MDL_setPerformance(LIS3MDL* lis3mdl, uint8_t performance) {
	HAL_StatusTypeDef status;
	LIS3MDL_CS_HIGH(lis3mdl->port, lis3mdl->pin);
	status = _LIS3MDL_writeRegister(lis3mdl, _LIS3MDL_REG_CTL_1,
			performance << 5, 0b01100000);
	if (status != HAL_OK) {
		LIS3MDL_CS_LOW(lis3mdl->port, lis3mdl->pin);

		return status;

	}
	status = _LIS3MDL_writeRegister(lis3mdl, _LIS3MDL_REG_CTL_4,
			performance << 2, 0b00001100);
	LIS3MDL_CS_LOW(lis3mdl->port, lis3mdl->pin);

	return status;
}

HAL_StatusTypeDef LIS3MDL_setDateRate(LIS3MDL* lis3mdl, uint8_t dataRate) {
	HAL_StatusTypeDef s;
	LIS3MDL_CS_HIGH(lis3mdl->port, lis3mdl->pin);

	s = _LIS3MDL_writeRegister(lis3mdl, _LIS3MDL_REG_CTL_1, dataRate << 2,
			0b00011100);
	LIS3MDL_CS_LOW(lis3mdl->port, lis3mdl->pin);

	return s;
}

HAL_StatusTypeDef LIS3MDL_selftest(LIS3MDL* lis3mdl) {
	LIS3MDL_CS_HIGH(lis3mdl->port, lis3mdl->pin);
	_LIS3MDL_writeRegister(lis3mdl, _LIS3MDL_REG_CTL_1, 0x1C,0);
	_LIS3MDL_writeRegister(lis3mdl, _LIS3MDL_REG_CTL_2, 0x40,0);
//	LIS3MDL_CS_LOW(lis3mdl->port, lis3mdl->pin);
	HAL_Delay(20);
//	_LIS3MDL_writeRegister(lis3mdl, _LIS3MDL_REG_CTL_3, 0x00,0);

//	_LIS3MDL_init(lis3mdl);
//
//	LIS3MDL_setScale(lis3mdl, LIS3MDL_SCALE_12_GAUSS);
//	LIS3MDL_setMode(lis3mdl, LIS3MDL_MODE_CONTINUOUS);
//	LIS3MDL_setPerformance(lis3mdl, LIS3MDL_PERFORMANCE_MEDIUM);
//	HAL_Delay(20);
	int val=0;
	int axisNST[3]={0};
	int axisST[3]={0};
	LIS3MDL_CS_HIGH(lis3mdl->port, lis3mdl->pin);
	while ((val & LIS3MDL_STATUS_ZYXDA) != 8)
		LIS3MDL_readStatus(lis3mdl,&val);
//		_LIS3MDL_readRegister(lis3mdl, LIS3MDL_STATUS_ZYXDA, &val);
	LIS3MDL_CS_LOW(lis3mdl->port, lis3mdl->pin);
	if (val) {
		for (int j = 0; j < 3; j++)
			LIS3MDL_readAxis(lis3mdl, j, &val);
	}

	for (int i = 0; i < 5; i++) {
		val=0;
		LIS3MDL_CS_HIGH(lis3mdl->port, lis3mdl->pin);
		while ((val & LIS3MDL_STATUS_ZYXDA) != 8)
			 		LIS3MDL_readStatus(lis3mdl,&val);
		for (int j = 0; j < 3; j++) {
			LIS3MDL_readAxis(lis3mdl, j, &val);
			axisNST[j] += val;
		}
	}
	for(int i=0;i<3;i++) axisNST[i] /= 5;

	////eanable self test
	LIS3MDL_CS_HIGH(lis3mdl->port, lis3mdl->pin);
	_LIS3MDL_writeRegister(lis3mdl, _LIS3MDL_REG_CTL_1, 0x1D,0);
	LIS3MDL_CS_LOW(lis3mdl->port, lis3mdl->pin);
	HAL_Delay(60);
	////
	///
	///
	 val=0;
	 LIS3MDL_CS_HIGH(lis3mdl->port, lis3mdl->pin);
	 while ((val & LIS3MDL_STATUS_ZYXDA) != 8)
	 		LIS3MDL_readStatus(lis3mdl,&val);
	LIS3MDL_CS_LOW(lis3mdl->port, lis3mdl->pin);
	if (val) {
		for (int j = 0; j < 3; j++)
			LIS3MDL_readAxis(lis3mdl, j, &val);
	}
	////////////////////////////
	for (int i = 0; i < 5; i++) {
		LIS3MDL_CS_HIGH(lis3mdl->port, lis3mdl->pin);
		val=0;
		while ((val & LIS3MDL_STATUS_ZYXDA) != 8)
				LIS3MDL_readStatus(lis3mdl,&val);
		for (int j = 0; j < 3; j++) {
			LIS3MDL_readAxis(lis3mdl, j, &val);
			axisST[j] += val;
		}
	}
	for(int i=0;i<3;i++) axisST[i] /= 5;
	float thrs_xy_min = 1.0;
		float thrs_xy_max = 3.0;
		float thrs_z_min = 0.1;
		float thrs_z_max = 1.0;
	if (LIS3MDL_checkselftest(axisNST[0], axisST[0], thrs_xy_min, thrs_xy_max) && LIS3MDL_checkselftest(axisNST[1], axisST[1], thrs_xy_min, thrs_xy_max) && LIS3MDL_checkselftest(axisNST[2], axisST[2], thrs_z_min, thrs_z_max)) {
		LIS3MDL_CS_LOW(lis3mdl->port, lis3mdl->pin);
		return HAL_OK;

		}
	else{
		LIS3MDL_CS_LOW(lis3mdl->port, lis3mdl->pin);
		return HAL_ERROR;
	}
}


uint8_t LIS3MDL_checkselftest(const double val1, const double val2, const double lim1, const double lim2){
    if (fabs(lim1) > fabs(lim2)){
        return ((fabs(val2 - val1) >= fabs(lim2)) && (fabs(val2 - val1) <= fabs(lim1)));
    }
    return ((fabs(val2 - val1) >= fabs(lim1)) && (fabs(val2 - val1) <= fabs(lim2)));

}


HAL_StatusTypeDef LIS3MDL_setMode(LIS3MDL* lis3mdl, uint8_t mode) {
	HAL_StatusTypeDef s;
	LIS3MDL_CS_HIGH(lis3mdl->port, lis3mdl->pin);

	s = _LIS3MDL_writeRegister(lis3mdl, _LIS3MDL_REG_CTL_3, mode << 0,
			0b00000011);
	LIS3MDL_CS_LOW(lis3mdl->port, lis3mdl->pin);

	return s;
}

HAL_StatusTypeDef LIS3MDL_setScale(LIS3MDL* lis3mdl, uint8_t scale) {
	HAL_StatusTypeDef status;
	LIS3MDL_CS_HIGH(lis3mdl->port, lis3mdl->pin);

	status = _LIS3MDL_writeRegister(lis3mdl, _LIS3MDL_REG_CTL_2, scale << 5,
			0b01100000);

	if (status != HAL_OK) {
		LIS3MDL_CS_LOW(lis3mdl->port, lis3mdl->pin);

		return status;
	}
	LIS3MDL_CS_LOW(lis3mdl->port, lis3mdl->pin);

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
	status = _LIS3MDL_readRegister_int16(lis3mdl, lowAddr, highAddr, value);
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

HAL_StatusTypeDef LIS3MDL_readTemperature(LIS3MDL* lis3mdl, int16_t* value) {
	HAL_StatusTypeDef s;
	LIS3MDL_CS_HIGH(lis3mdl->port, lis3mdl->pin);
	s = _LIS3MDL_readRegister_int16(lis3mdl, _LIS3MDL_REG_OUT_TEMP_L,
			_LIS3MDL_REG_OUT_TEMP_H, value);
	LIS3MDL_CS_LOW(lis3mdl->port, lis3mdl->pin);

	return s;
}

HAL_StatusTypeDef LIS3MDL_readDeviceId(LIS3MDL* lis3mdl, uint8_t* deviceId) {
	HAL_StatusTypeDef s;
	LIS3MDL_CS_HIGH(lis3mdl->port, lis3mdl->pin);
	s = _LIS3MDL_readRegister(lis3mdl, _LIS3MDL_REG_WHO_AM_I, deviceId);
	LIS3MDL_CS_LOW(lis3mdl->port, lis3mdl->pin);

	return s;
}

HAL_StatusTypeDef LIS3MDL_readStatus(LIS3MDL* lis3mdl, uint8_t* status) {
	HAL_StatusTypeDef s;
	LIS3MDL_CS_HIGH(lis3mdl->port, lis3mdl->pin);
	s = _LIS3MDL_readRegister(lis3mdl, _LIS3MDL_REG_STATUS, status);
	LIS3MDL_CS_LOW(lis3mdl->port, lis3mdl->pin);

	return s;
}

HAL_StatusTypeDef _LIS3MDL_init(LIS3MDL* lis3mdl) {

	HAL_StatusTypeDef status;
	uint8_t deviceId;

	status = LIS3MDL_readDeviceId(lis3mdl, &deviceId);
	if (status != HAL_OK) {
		LIS3MDL_DEBUG_OUT("readDeviceId status %d\n", status);
		return status;
	}

	if (deviceId != LIS3MDL_DEVICE_ID) {
		LIS3MDL_DEBUG_OUT("invalid device id. expected 0x%02x, found: 0x%02x\n", LIS3MDL_DEVICE_ID, deviceId);
		LIS3MDL_CS_LOW(lis3mdl->port, lis3mdl->pin);

		return HAL_ERROR;
	}

	uint8_t reg2;
	LIS3MDL_CS_HIGH(lis3mdl->port, lis3mdl->pin);

	status = _LIS3MDL_readRegister(lis3mdl, _LIS3MDL_REG_CTL_2, &reg2);
	if (status != HAL_OK) {
		LIS3MDL_DEBUG_OUT("read scale status %d\n", status);
		LIS3MDL_CS_LOW(lis3mdl->port, lis3mdl->pin);

		return status;
	}
	lis3mdl->scale = _LIS3MDLGAUSS_TO_SCALE[(reg2 >> 5) & 0b11];
	LIS3MDL_CS_LOW(lis3mdl->port, lis3mdl->pin);
	return HAL_OK;
}

HAL_StatusTypeDef lis3mdl_offset(LIS3MDL* lis3mdl,int count)
	{
	int val=0;
	int32_t axisST[3]={0};
	for (int i = 0; i < count; i++) {
		LIS3MDL_CS_HIGH(lis3mdl->port, lis3mdl->pin);
		val=0;
		while ((val & LIS3MDL_STATUS_ZYXDA) != 8)
				LIS3MDL_readStatus(lis3mdl,&val);
		for (int j = 0; j < 3; j++) {
			LIS3MDL_readAxis(lis3mdl, j, &val);
			axisST[j] += val;
		}
	}
	LIS3MDL_CS_LOW(lis3mdl->port, lis3mdl->pin);
	for(int i=0;i<3;i++) axisST[i] /= count;
	LIS3MDL_CS_HIGH(lis3mdl->port, lis3mdl->pin);
	_LIS3MDL_writeRegister(lis3mdl, 0x05,  axisST[0] & 0xff,	0xff);//xlow
	_LIS3MDL_writeRegister(lis3mdl, 0x06, (axisST[0] >>8),	0xff);//xhigh
	_LIS3MDL_writeRegister(lis3mdl, 0x07, axisST[1] & 0xff ,	0xff);//ylow
	_LIS3MDL_writeRegister(lis3mdl, 0x08, (axisST[1] >>8),	0xff);//yhigh
	_LIS3MDL_writeRegister(lis3mdl, 0x09, axisST[2] & 0xff ,	0xff);//zlow
	_LIS3MDL_writeRegister(lis3mdl, 0x0A, (axisST[2] >>8) ,	0xff);//zhigh
	LIS3MDL_CS_LOW(lis3mdl->port, lis3mdl->pin);
	return HAL_OK;
	}
HAL_StatusTypeDef _LIS3MDL_readRegister_int16(LIS3MDL* lis3mdl, uint8_t lowAddr,
		uint8_t highAddr, int16_t* value) {
	HAL_StatusTypeDef status;
	uint8_t low, high;
	LIS3MDL_CS_HIGH(lis3mdl->port, lis3mdl->pin);
	status = _LIS3MDL_readRegister(lis3mdl, lowAddr, &low);
	if (status != HAL_OK) {
		LIS3MDL_CS_LOW(lis3mdl->port, lis3mdl->pin);
		return status;
	}

	status = _LIS3MDL_readRegister(lis3mdl, highAddr, &high);
	if (status != HAL_OK) {
		LIS3MDL_CS_LOW(lis3mdl->port, lis3mdl->pin);
		return status;
	}

	*value = (((uint16_t) high) << 8) | (uint16_t) low;
	LIS3MDL_CS_LOW(lis3mdl->port, lis3mdl->pin);
	return HAL_OK;
}

HAL_StatusTypeDef _LIS3MDL_readRegister(LIS3MDL* lis3mdl, uint8_t reg,
		uint8_t* value) {
	HAL_StatusTypeDef status;
	status = HAL_I2C_Mem_Read(lis3mdl->i2c, lis3mdl->address, reg,
			I2C_MEMADD_SIZE_8BIT, value, 1, 100);

	if (status != HAL_OK) {
		LIS3MDL_DEBUG_OUT("readRegister: mem read error reg 0x%02x status %d\n", reg, status);
		return status;
	}

	return HAL_OK;
}

HAL_StatusTypeDef _LIS3MDL_writeRegister(LIS3MDL* lis3mdl, uint8_t reg,
		uint8_t data, uint8_t mask) {
	HAL_StatusTypeDef status;
	uint8_t valueToWrite;

	if (mask == 0xff) {
		valueToWrite = data;
	} else {
		uint8_t currentValue;
//    LIS3MDL_CS_HIGH(lis3mdl->port,lis3mdl->pin);

		status = _LIS3MDL_readRegister(lis3mdl, reg, &currentValue);
		if (status != HAL_OK) {
			LIS3MDL_DEBUG_OUT("writeRegister: rx current error reg 0x%02x status %d\n", reg, status);
		}
		valueToWrite = (currentValue & ~mask) | (data & mask);
	}
	status = HAL_I2C_Mem_Write(lis3mdl->i2c, lis3mdl->address, reg,
			I2C_MEMADD_SIZE_8BIT, &valueToWrite, 1, 100);
	if (status != HAL_OK) {
		LIS3MDL_DEBUG_OUT("writeRegister: tx error reg 0x%02x status %d\n", reg, status);
//    LIS3MDL_CS_LOW(lis3mdl->port,lis3mdl->pin);
		return status;
	}
//  LIS3MDL_CS_LOW(lis3mdl->port,lis3mdl->pin);

	return status;
}
