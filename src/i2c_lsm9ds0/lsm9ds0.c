#include <libopencm3/stm32/i2c.h>
#include "lsm9ds0.h"
#include "../common/i2c.h"

/* global variables */
lsm9ds0AccelRange_t _accel_mg_lsb;
lsm9ds0MagGain_t _mag_mgauss_lsb;
lsm9ds0GyroScale_t _gyro_dps_digit;

void lsm9ds0_enable_accel(uint32_t i2c, uint8_t sensor)
{
	// Enable the accelerometer continous
	uint8_t reg_value;
	reg_value = 0x67;	/* 100 hz XYZ */
	i2c_write_buffer(i2c, sensor, LSM9DS0_REGISTER_CTRL_REG1_XM,
			 1, &reg_value);

	reg_value = 0b11110000;
	i2c_write_buffer(i2c, sensor, LSM9DS0_REGISTER_CTRL_REG5_XM,
			 1, &reg_value);
}

void lsm9ds0_enable_mag(uint32_t i2c, uint8_t sensor)
{
	// enable mag continuous
	uint8_t reg_value = 0b00000000;
	i2c_write_buffer(i2c, sensor, LSM9DS0_REGISTER_CTRL_REG7_XM,
			 1, &reg_value);
}

void lsm9ds0_enable_temp(uint32_t i2c, uint8_t sensor)
{
	uint8_t mode;
	i2c_read_buffer(i2c, sensor, LSM9DS0_REGISTER_CTRL_REG5_XM,
			1, &mode);
	mode |= (1<<7);		/* enable temperature sensor */
	i2c_write_buffer(i2c, sensor, LSM9DS0_REGISTER_CTRL_REG5_XM,
			 1, &mode);
}

void lsm9ds0_enable_gyro(uint32_t i2c, uint8_t sensor)
{
	/* TODO */
}

lsm9ds0_setup_accel(uint32_t i2c, uint8_t sensor, lsm9ds0AccelRange_t range)
{
	uint8_t reg;
	i2c_read_buffer(i2c, sensor, LSM9DS0_REGISTER_CTRL_REG2_XM,
			1, &reg);

	reg &= ~(0b00111000);
	reg |= range;
	i2c_write_buffer(i2c, sensor, LSM9DS0_REGISTER_CTRL_REG2_XM,
			 1, &reg );

	switch (range)
	{
	case LSM9DS0_ACCELRANGE_2G:
		_accel_mg_lsb = LSM9DS0_ACCEL_MG_LSB_2G;
		break;
	case LSM9DS0_ACCELRANGE_4G:
		_accel_mg_lsb = LSM9DS0_ACCEL_MG_LSB_4G;
		break;
	case LSM9DS0_ACCELRANGE_6G:
		_accel_mg_lsb = LSM9DS0_ACCEL_MG_LSB_6G;
		break;
	case LSM9DS0_ACCELRANGE_8G:
		_accel_mg_lsb = LSM9DS0_ACCEL_MG_LSB_8G;
		break;
	case LSM9DS0_ACCELRANGE_16G:
		_accel_mg_lsb =LSM9DS0_ACCEL_MG_LSB_16G;
		break;
	}
}

lsm9ds0_setup_mag(uint32_t i2c, uint8_t sensor, lsm9ds0MagGain_t gain)
{
	uint8_t reg;
	i2c_read_buffer(i2c, sensor, LSM9DS0_REGISTER_CTRL_REG6_XM,
			1, &reg);

	reg &= ~(0b01100000);
	reg |= gain;
	i2c_write_buffer(i2c, sensor, LSM9DS0_REGISTER_CTRL_REG6_XM,
			 1, &reg);

	switch(gain)
	{
	case LSM9DS0_MAGGAIN_2GAUSS:
		_mag_mgauss_lsb = LSM9DS0_MAG_MGAUSS_2GAUSS;
		break;
	case LSM9DS0_MAGGAIN_4GAUSS:
		_mag_mgauss_lsb = LSM9DS0_MAG_MGAUSS_4GAUSS;
		break;
	case LSM9DS0_MAGGAIN_8GAUSS:
		_mag_mgauss_lsb = LSM9DS0_MAG_MGAUSS_8GAUSS;
		break;
	case LSM9DS0_MAGGAIN_12GAUSS:
		_mag_mgauss_lsb = LSM9DS0_MAG_MGAUSS_12GAUSS;
		break;
	}
}

lsm9ds0_setup_temp(uint32_t i2c, uint8_t sensor)
{
	/* only for convenience */
}

lsm9ds0_setup_gyro(uint32_t i2c, uint8_t sensor, lsm9ds0GyroScale_t scale)
{
	uint8_t reg;
	i2c_read_buffer(i2c, sensor, LSM9DS0_REGISTER_CTRL_REG4_G,
			1, &reg);

	reg &= ~(0b00110000);
	reg |= scale;
	i2c_write_buffer(i2c, sensor, LSM9DS0_REGISTER_CTRL_REG4_G,
			 1, &reg);

	switch(scale)
	{
	case LSM9DS0_GYROSCALE_245DPS:
		_gyro_dps_digit = LSM9DS0_GYRO_DPS_DIGIT_245DPS;
		break;
	case LSM9DS0_GYROSCALE_500DPS:
		_gyro_dps_digit = LSM9DS0_GYRO_DPS_DIGIT_500DPS;
		break;
	case LSM9DS0_GYROSCALE_2000DPS:
		_gyro_dps_digit = LSM9DS0_GYRO_DPS_DIGIT_2000DPS;
		break;
	}
}

void lsm9ds0_init_sensor(uint32_t i2c, uint8_t sensor)
{
	lsm9ds0_enable_accel(i2c,  sensor);
	lsm9ds0_enable_mag(i2c, sensor);
	lsm9ds0_enable_temp(i2c, sensor);
	lsm9ds0_enable_gyro(i2c, sensor);

	/* setup sensors */
	lsm9ds0_setup_accel(i2c, sensor, LSM9DS0_ACCELRANGE_2G);
	lsm9ds0_setup_mag(i2c, sensor, LSM9DS0_MAGGAIN_2GAUSS);
	lsm9ds0_setup_temp(i2c, sensor);
	lsm9ds0_setup_gyro(i2c, sensor, LSM9DS0_GYROSCALE_245DPS);
}

lsm9ds0Vector_t lsm9ds0_read_accel(uint32_t i2c, uint8_t sensor)
{
	// Read the accelerometer
	lsm9ds0Vector_t accelData;
	uint8_t buffer[6];
	i2c_read_buffer(i2c, sensor, 0x80 | LSM9DS0_REGISTER_OUT_X_L_A,
			6, buffer);

	uint8_t xlo = buffer[0];
	uint16_t xhi = buffer[1];
	uint8_t ylo = buffer[2];
	uint16_t yhi = buffer[3];
	uint8_t zlo = buffer[4];
	uint16_t zhi = buffer[5];

	// Shift values to create properly formed integer (low byte first)
	xhi <<= 8; xhi |= xlo;
	yhi <<= 8; yhi |= ylo;
	zhi <<= 8; zhi |= zlo;

	accelData.x = xhi * _accel_mg_lsb;
	accelData.y = yhi * _accel_mg_lsb;
	accelData.z = zhi * _accel_mg_lsb;

	return accelData;
}

lsm9ds0Vector_t lsm9ds0_read_mag(uint32_t i2c, uint8_t sensor)
{
	// Read the magnetometer
	lsm9ds0Vector_t magData;
	byte buffer[6];
	i2c_read_buffer(i2c, sensor, 0x80 | LSM9DS0_REGISTER_OUT_X_L_M,
			6, buffer);

	uint8_t xlo = buffer[0];
	uint16_t xhi = buffer[1];
	uint8_t ylo = buffer[2];
	uint16_t yhi = buffer[3];
	uint8_t zlo = buffer[4];
	uint16_t zhi = buffer[5];

	// Shift values to create properly formed integer (low byte first)
	xhi <<= 8; xhi |= xlo;
	yhi <<= 8; yhi |= ylo;
	zhi <<= 8; zhi |= zlo;

	magData.x = xhi * _mag_mgauss_lsb;
	magData.y = yhi * _mag_mgauss_lsb;
	magData.z = zhi * _mag_mgauss_lsb;

	return magData;
}

float lsm9ds0_read_temp(uint32_t i2c, uint8_t sensor)
{
	uint8_t buffer[2];
	i2c_read_buffer(i2c, sensor, 0x80 | LSM9DS0_REGISTER_TEMP_OUT_L_XM,
					2, buffer);
	uint16_t result = buffer[1];
	result <<= 8;
	result |= buffer[0];

	float temperature;
	// This is just a guess since the staring point (21C here)
	// isn't documented :(
	temperature = 21.0 + (float)result/8;
	//temperature /= LSM9DS0_TEMP_LSB_DEGREE_CELSIUS;

	return temperature;
}
