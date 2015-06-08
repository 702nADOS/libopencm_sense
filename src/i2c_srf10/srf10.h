#ifndef _SRF10_H_
#define _SRF10_H_

/* The specification of the SRF10 uses 8-bit addresses.
 * I2C uses 7-bit addresses, so we need to SHIFT RIGHT them by one bit,
 * because the highest bit is used for the WRITE/READ flag.
 */
typedef enum {
	SRF10_SENSOR0	= 0x70, // 0xE0 >> 1
	SRF10_SENSOR1	= 0x71, // 0xE2 >> 1
	SRF10_SENSOR2	= 0x72, // 0xE4 >> 1
	SRF10_SENSOR3	= 0x73, // 0xE6 >> 1
	SRF10_SENSOR4	= 0x74, // 0xE8 >> 1
	SRF10_SENSOR5	= 0x75, // 0xEA >> 1
	SRF10_SENSOR6	= 0x76, // 0xEC >> 1
	SRF10_SENSOR7	= 0x77, // 0xEE >> 1
	SRF10_SENSOR8	= 0x78, // 0xF0 >> 1
	SRF10_SENSOR9	= 0x79, // 0xF2 >> 1
	SRF10_SENSOR10	= 0x7A, // 0xF4 >> 1
	SRF10_SENSOR11	= 0x7B, // 0xF6 >> 1
	SRF10_SENSOR12	= 0x7C, // 0xF8 >> 1
	SRF10_SENSOR13	= 0x7D, // 0xFA >> 1
	SRF10_SENSOR14	= 0x7E, // 0xFC >> 1
	SRF10_SENSOR15	= 0x7F // 0xFE >> 1
} srf10Sensors_t;

typedef enum {
	SRF10_MEAS_INCH	= 0x50,
	SRF10_MEAS_CM	= 0x51,
	SRF10_MEAS_MS	= 0x52
} srf10MeasurementModes_t;

void srf10_start_measurement(uint32_t i2c, uint8_t sensor, uint8_t mode);
uint8_t srf10_measurement_successful(uint32_t i2c, uint8_t sensor);
uint16_t srf10_get_last_measurement(uint32_t i2c, uint8_t sensor);

void srf10_change_range(uint32_t i2c, uint8_t sensor, uint8_t factor);
void srf10_change_gain(uint32_t i2c, uint8_t sensor, uint8_t factor);

#endif
