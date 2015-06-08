#include <libopencm3/stm32/i2c.h>
#include "lsm9ds0.h"
#include "../i2c_srf10/i2c.h"

void lsm9ds0_setup_temp(uint32_t i2c, uint8_t sensor)
{
	uint8_t mode;
	i2c_read_buffer(i2c, sensor, LSM9DS0_REGISTER_CTRL_REG5_XM, 1, &mode);
	mode |= (1<<7);		/* enable temperature sensor */
	i2c_write_buffer(i2c, sensor, LSM9DS0_REGISTER_CTRL_REG5_XM, 1, &mode);
}


uint16_t lsm9ds0_read_temp(uint32_t i2c, uint8_t sensor)
{
	uint8_t buffer[2];
	i2c_read_buffer(i2c, sensor, 0x80 | LSM9DS0_REGISTER_TEMP_OUT_L_XM,
					2, buffer);
	uint16_t result = buffer[1];
	result <<= 8;
	result |= buffer[0];
	return result;
}
