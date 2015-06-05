#include <libopencm3/stm32/i2c.h>
#include "srf10.h"
#include "i2c.h"

void srf10_start_measurement(uint32_t i2c, uint8_t sensor, uint8_t mode) {
	i2c_write_buffer(i2c, sensor, 0x00, 1, &mode);
}

uint8_t srf10_measurement_successful(uint32_t i2c, uint8_t sensor) {
	uint8_t result;
	i2c_read_buffer(i2c, sensor, 0x00, 1, &result);

	return result == 0xff;
}

uint16_t srf10_get_last_measurement(uint32_t i2c, uint8_t sensor) {
        uint8_t buffer[2];
        i2c_read_buffer(i2c, sensor, 0x02, 2, buffer);

	uint16_t result = buffer[0];
	result <<= 8;
	result |= buffer[1];

        return result;
}

/*
 * factor = range(0, 16)
 *
 * (temporary value)
 */
void srf10_change_gain(uint32_t i2c, uint8_t sensor, uint8_t factor) {
	i2c_write_buffer(i2c, sensor, 0x01, 1, &factor);
}

/*
 * range = (43mm * factor) + 43mm
 * factor = range(0x00, 0xff)
 *
 * (temporary value)
 */
void srf10_change_range(uint32_t i2c, uint8_t sensor, uint8_t factor) {
	i2c_write_buffer(i2c, sensor, 0x02, 1, &factor);
}

