
#include <stdint.h>
#include <stdbool.h>
#include <platform-abstraction/threading.h>
#include "ms5611.h"

#define PROM_SENS       0
#define PROM_OFF        1
#define PROM_TCS        2
#define PROM_TCO        3
#define PROM_TREF       4
#define PROM_TEMPSENS   5

#define MS5611_CMD_ADC_PRESS(osr)  (0x40 | ((osr)<<1))
#define MS5611_CMD_ADC_TEMP(osr)   (0x50 | ((osr)<<1))
#define MS5611_CMD_RESET            0x1E
#define MS5611_CMD_ADC_READ         0x00
#define MS5611_CMD_PROM_READ_BASE   0XA0

#define SQUARE(x) ((x)*(x))

/* global variables */
const uint16_t ms5611_osr_dly_us[] = {600, 1170, 2280, 4540, 9040};


void ms5611_i2c_init(ms5611_t *ms5611, i2c_bus_t *bus, int csb_pin_value)
{
    uint8_t addr;

    ms5611->mode = ms5611_i2c;

    /* LSbit of addr is complementary of CSB pin */
    if (csb_pin_value == 1) {
        addr = 0x76;
    } else {
        addr = 0x77;
    }

    i2c_device_init(&ms5611->dev.i2c, bus, addr);

    ms5611_reset(ms5611);

    ms5611_prom_read(ms5611);
}

void ms5611_reset(ms5611_t *ms5611)
{
    uint8_t reset_cmd = MS5611_CMD_RESET;
    if (ms5611->mode == ms5611_i2c) {
        i2c_write(&ms5611->dev.i2c, &reset_cmd, 1);
    }
}

static uint16_t ms5611_prom_read_i2c(ms5611_t *ms5611, uint8_t addr)
{
    uint8_t buf[2];

    i2c_write(&ms5611->dev.i2c, &addr, 1);

    i2c_read(&ms5611->dev.i2c, buf, 2);

    return (uint16_t) buf[1] | (buf[0]<<8);
}

int ms5611_prom_read(ms5611_t *ms5611)
{
    uint8_t addr;
    uint16_t crc;

    addr = MS5611_CMD_PROM_READ_BASE + 2;

    if (ms5611->mode == ms5611_i2c) {
        uint8_t i;
        for (i = 0; i < 6; i++) {
            ms5611->prom[i] = ms5611_prom_read_i2c(ms5611, addr);
            addr += 2;
        }
        crc = ms5611_prom_read_i2c(ms5611, addr);
    } else {
        // for (i = 0; i < 6; i++) {
        //     ms5611->prom[i] = ms5611_prom_read_spi(ms5611, addr);
        //     addr += 2;
        // }
        // crc = ms5611_prom_read_spi(ms5611, addr);
    }

    /* todo: check crc */
    (void) crc;

    return 0;
}

static uint32_t ms5611_adc_read_i2c(ms5611_t *ms5611, uint8_t cmd, uint8_t osr)
{
    uint8_t buf[3];

    /* send measurement command */
    i2c_write(&ms5611->dev.i2c, &cmd, sizeof(cmd));

    /* sleep for needed conversion time */
    os_thread_sleep_least_us(ms5611_osr_dly_us[osr]);

    cmd = MS5611_CMD_ADC_READ;
    /* send ADC read command */
    i2c_write(&ms5611->dev.i2c, &cmd, 1);

    /* read result */
    i2c_read(&ms5611->dev.i2c, buf, 3);

    /* setup 24bit result, MSByte received first */
    return (uint32_t) buf[2]|(buf[1]<<8)|(buf[0]<<16);
}

uint32_t ms5611_press_adc_read(ms5611_t *ms5611, uint8_t osr)
{
    uint8_t cmd;

    cmd = MS5611_CMD_ADC_PRESS(osr);

    if (ms5611->mode == ms5611_i2c) {
        return ms5611_adc_read_i2c(ms5611, cmd, osr);
    } else {
        // return ms5611_adc_read_spi(ms5611, cmd);
        return 0;
    }

}

uint32_t ms5611_temp_adc_read(ms5611_t *ms5611, uint8_t osr)
{
    uint8_t cmd;

    cmd = MS5611_CMD_ADC_TEMP(osr);

    if (ms5611->mode == ms5611_i2c) {
        return ms5611_adc_read_i2c(ms5611, cmd, osr);
    } else {
        // return ms5611_adc_read_spi(ms5611, cmd);
        return 0;
    }
}

int32_t ms5611_calc_temp(ms5611_t *ms5611, uint32_t raw_t)
{
    int32_t dt = (int32_t) raw_t - (ms5611->prom[PROM_TREF]<<8);
    return (int32_t) 2000 + (dt * ms5611->prom[PROM_TEMPSENS] / (1<<23));
}

uint32_t ms5611_calc_press(ms5611_t *ms5611, uint32_t raw_p, uint32_t raw_t, int32_t *p_temp)
{
    int32_t dt, temp;
    int64_t off, sens;

    dt = (int32_t) raw_t - ms5611->prom[PROM_TREF] * (1<<8);
    temp = (int32_t) 2000 + dt * ms5611->prom[PROM_TEMPSENS] / (1<<23);

    off = (int64_t) ms5611->prom[PROM_OFF] * (1<<16) + ms5611->prom[PROM_TCO] * dt / (1<<7);
    sens = (int64_t) ms5611->prom[PROM_SENS] * (1<<15) + ms5611->prom[PROM_TCS] * dt / (1<<8);

    /* low temperature correcture, (temp < 20.00 C) */
    if (temp < 2000) {
        uint32_t t2, off2, sens2;

        t2 = (uint32_t) (SQUARE(dt) / (1<<31));
        off2 = (uint32_t) (5 * SQUARE(temp - 2000)) / 2;
        sens2 = off2 / 2;

        /* very low temperature correcture (temp < -15.00 C) */
        if (temp < -1500) {
            off2 = off2 + 7 * SQUARE(temp + 1500);
            sens2 = sens2 + ((11 * SQUARE(temp + 1500)) / 2);
        }

        /* correction */
        temp = temp - t2;
        off = off - off2;
        sens = sens - sens2;
    }

    if (p_temp != NULL) {
        *p_temp = temp;
    }

    /* calculate pressure */
    uint32_t p = (uint32_t) ((raw_p * sens / (1<<21)) - off) / (1<<15);
    return p;
}
