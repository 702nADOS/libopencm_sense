#include <libopencm3/stm32/i2c.h>
#include "mpl3115a2.h"
#include "../common/i2c.h"

/* TODO: Wait via systick */
static void delay(uint32_t t)
{
	t *= 1000;
	while (t--) ;
}

void mpl3115a2_init_sensor(uint32_t i2c, uint8_t sensor)
{
	uint8_t mode = 0;
	mode |= MPL3115A2_CTRL_REG1_SBYB |
		MPL3115A2_CTRL_REG1_OS128 |
		MPL3115A2_CTRL_REG1_ALT;
	i2c_write_buffer(i2c, sensor, MPL3115A2_CTRL_REG1,
			 1, &mode);

	mode = 0;
	mode |=  MPL3115A2_PT_DATA_CFG_TDEFE |
		MPL3115A2_PT_DATA_CFG_PDEFE |
		MPL3115A2_PT_DATA_CFG_DREM;
	i2c_write_buffer(i2c, sensor, MPL3115A2_PT_DATA_CFG,
			 1, &mode);
}

void mpl3115a2_get_pressure(uint32_t i2c, uint8_t sensor, float *result)
{
	uint8_t pressure[3];
	uint32_t p = 0;
	uint8_t mode = 0;
	mode |= MPL3115A2_CTRL_REG1_SBYB |
		MPL3115A2_CTRL_REG1_OS128 |
		MPL3115A2_CTRL_REG1_BAR;
	i2c_write_buffer(i2c, sensor, MPL3115A2_CTRL_REG1,
			 1, &mode);

	uint8_t sta = 0;
	while (! (sta & MPL3115A2_REGISTER_STATUS_PDR)) {
		i2c_read_buffer(i2c, sensor, MPL3115A2_REGISTER_STATUS,
						1, &sta);
		/* ms */
		delay(10);
	}

	i2c_read_buffer(i2c, sensor, MPL3115A2_REGISTER_PRESSURE_MSB,
			 3, pressure);

	p = pressure[0]; p <<= 8;
	p |= pressure[1]; p <<= 8;
	p |= pressure[2]; p >>= 4;

	/* Attention: integer/float divsion */
	*result = p / 4.0;
}

void mpl3115a2_get_altitude(uint32_t i2c, uint8_t sensor, float *result)
{
	uint8_t altitude[3];
	uint32_t a = 0;
	uint8_t mode = 0;
	mode |= MPL3115A2_CTRL_REG1_SBYB |
		MPL3115A2_CTRL_REG1_OS128 |
		MPL3115A2_CTRL_REG1_ALT;

	i2c_write_buffer(i2c, sensor, MPL3115A2_CTRL_REG1,
					 1, &mode);

	uint8_t sta = 0;
	while (! (sta & MPL3115A2_REGISTER_STATUS_PDR)) {
		i2c_read_buffer(i2c, sensor, MPL3115A2_REGISTER_STATUS,
						1, &sta);
		/* does this function exists */
		delay(10);
	}

	i2c_read_buffer(i2c, sensor, MPL3115A2_REGISTER_PRESSURE_MSB,
			 3, altitude);

	a = altitude[0]; a <<= 8;
	a |= altitude[1]; a <<= 8;
	a |= altitude[2]; a >>= 4;

	if(a & 0x800000) a |= 0xFF000000;

	/* Attention: integer/float divsion */
	*result = a / 16.0;
}

void mpl3115a2_get_temperature(uint32_t i2c, uint8_t sensor, float *result)
{
	uint8_t temp[2];
	uint16_t t = 0;

	uint8_t sta = 0;
	while (! (sta & MPL3115A2_REGISTER_STATUS_TDR)) {
		i2c_read_buffer(i2c, sensor, MPL3115A2_REGISTER_STATUS,
						1, &sta);
		/* does this function exists */
		delay(10);
	}

	i2c_read_buffer(i2c, sensor, MPL3115A2_REGISTER_TEMP_MSB,
			 2, temp);

	t = temp[0]; t <<= 8;
	t = temp[1]; t >>= 4;

	*result = t / 16.0;
}
