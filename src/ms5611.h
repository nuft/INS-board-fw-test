
#include <stdint.h>
#include <i2c.h>

#define MS5611_OSR_256          0x00
#define MS5611_OSR_512          0x01
#define MS5611_OSR_1024         0x02
#define MS5611_OSR_2048         0x03
#define MS5611_OSR_4096         0x04

#define MS5611_I2C_ADDR1    0x76
#define MS5611_I2C_ADDR2    0x77

typedef struct {
    union {
        i2c_dev_t i2c;
        spi_dev_t spi;
    } dev;
    enum {i2c_mode = 0, spi_mode = 1};
    struct {
        uint16_t sens;
        uint16_t off;
        uint16_t tcs;
        uint16_t tco;
        uint16_t tref;
        uint16_t tempsens;
    } prom;
} ms5611_t;

/** Initializes MS5611 device */
void ms5611_i2c_init(ms5611_t *ms5611, i2c_bus_t *bus, uint8_t i2c_addr);

/** Resets the MS5611 device */
void ms5611_reset(ms5611_t *ms5611);

/** Measure the temperature with the given osr over sampling rate.
 *  Returns 0 if measurement failed. */
uint32_t ms5611_temp_meas(ms5611_t *ms5611, uint8_t osr);

/** Measure the pressure with the given osr over sampling rate.
 *  Returns 0 if measurement failed. */
uint32_t ms5611_press_meas(ms5611_t *ms5611, uint8_t osr);

int32_t ms5611_calculate_temperature(int32_t dt);
int32_t ms5611_calculate_dt(uint32_t raw_temp);
uint32_t ms5611_calculate_pressure(uint32_t raw_p, int32_t dt, int32_t temp);
float ms5611_calculate_pressure_float(uint32_t raw_p, int32_t dt, int32_t temp);
uint32_t ms5611_get_pressure(void);
