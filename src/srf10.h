#define SRF10_SENSOR0	0xE0 >> 1
#define SRF10_SENSOR1	0xE2 >> 1
#define SRF10_SENSOR2	0xE4 >> 1
#define SRF10_SENSOR3	0xE6 >> 1
#define SRF10_SENSOR4	0xE8 >> 1
#define SRF10_SENSOR5	0xEA >> 1
#define SRF10_SENSOR6	0xEC >> 1
#define SRF10_SENSOR7	0xEE >> 1
#define SRF10_SENSOR8	0xF0 >> 1
#define SRF10_SENSOR9	0xF2 >> 1
#define SRF10_SENSOR10	0xF4 >> 1
#define SRF10_SENSOR11	0xF6 >> 1
#define SRF10_SENSOR12	0xF8 >> 1
#define SRF10_SENSOR13	0xFA >> 1
#define SRF10_SENSOR14	0xFC >> 1
#define SRF10_SENSOR15	0xFE >> 1

#define SRF10_MEAS_INCH	0x50
#define SRF10_MEAS_CM	0x51
#define SRF10_MEAS_MS	0x52

void srf10_start_measurement(uint32_t i2c, uint8_t sensor, uint8_t mode);
uint8_t srf10_measurement_successful(uint32_t i2c, uint8_t sensor);
uint16_t srf10_get_last_measurement(uint32_t i2c, uint8_t sensor);
