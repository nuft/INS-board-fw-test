
#include <stdint.h>
#include <stdbool.h>
#include <os.h>
#include <systick.h>
#include <i2c.h>
#include "ms5611.h"


// #define PRESSURE_OSR_256        0x40    // D1 pressure
// #define PRESSURE_OSR_512        0x42
// #define PRESSURE_OSR_1024       0x44
// #define PRESSURE_OSR_2048       0x46
// #define PRESSURE_OSR_4096       0x48
// #define TEMPERATURE_OSR_256     0x50    // D2 temperature
// #define TEMPERATURE_OSR_512     0x52
// #define TEMPERATURE_OSR_1024    0x54
// #define TEMPERATURE_OSR_2048    0x56
// #define TEMPERATURE_OSR_4096    0x58

// #define MS5611_OSR_256_DLY_USEC      600
// #define MS5611_OSR_512_DLY_USEC     1170
// #define MS5611_OSR_1024_DLY_USEC    2280
// #define MS5611_OSR_2048_DLY_USEC    4540
// #define MS5611_OSR_4096_DLY_USEC    9040

#define MS5611_CMD_ADC_PRESS(osr) (0x40 | ((osr)<<1))
#define MS5611_CMD_ADC_TEMP(osr) (0x50 | ((osr)<<1))

const uint16_t ms5611_osr_dly_us[] = {600, 1170, 2280, 4540, 9040};


#define MS5611_CMD_RESET            0x1E
#define MS5611_CMD_ADC_READ         0x00
#define MS5611_CMD_PROM_READ_BASE   0XA2
// PROM starts at 0xA0m, but the first 16 bits are reserved for manufacturer
#define MS5611_AD_RESULT_LEN 3  //3 Bytes result

/* configuration defines */

#define MS5611_I2C_ADDR MS5611_I2C_ADDR2

#define PRESSURE_OSR            PRESSURE_OSR_4096
#define TEMPERATURE_OSR         TEMPERATURE_OSR_4096
#define PRESSURE_OSR_DELAY      MS5611_OSR_4096_DELAY
#define TEMPERATURE_OSR_DELAY   MS5611_OSR_4096_DELAY

// define how often to measure the temperature
#define TEMPERATURE_MEASUREMENT_INTERVAL 1  // [1, 255]


#define SQUARE(x) ((x)*(x))

/* global variables */

i2c_dev_t ms5611;

void ms5611_init(void)
{
    i2c_device_init(&ms5611->dev, &dev_i2c2, MS5611_I2C_ADDR);
    ms5611_reset();
    os_sleep(100 * (OS_TIMESTAMP_FREQ/1000));
    uint16_t *prom = &ms5611_prom->dev.sens;
    uint8_t prom_addr = MS5611_CMD_PROM_READ_BASE;
    uint8_t buf[2];
    uint8_t i;
    for (i = 0; i < sizeof(ms5611_prom)/sizeof(uint16_t); i++) {
        i2c_write(&ms5611->dev, &prom_addr, sizeof(prom_addr));
        i2c_read(&ms5611->dev, buf, sizeof(uint16_t));
        prom[i]=buf[1]|((uint16_t)buf[0]<<8);
        prom_addr += 2;
    }
}

void ms5611_reset(void)
{
    uint8_t reset_cmd = MS5611_CMD_RESET;
    i2c_write(&ms5611->dev, &reset_cmd, sizeof(reset_cmd));
}

static inline uint32_t ms5611_adc_read(ms5611_t *ms5611, uint8_t cmd)
{
    uint8_t buf[3];

    /* send measurement command */
    i2c_write(&ms5611->dev, &cmd, sizeof(cmd));

    /* sleep for needed conversion time */
    os_thread_sleep_least_us(ms5611_osr_dly_us[osr]);

    cmd = MS5611_CMD_ADC_READ;
    /* send ADC read command */
    i2c_write(&ms5611->dev, &cmd, 1);

    /* read result */
    i2c_read(&ms5611->dev, buf, 3);

    /* setup 24bit result, MSByte received first */
    return (uint32_t) buf[2]|(buf[1]<<8)|(buf[0]<<16);
}

uint32_t ms5611_press_adc_read(ms5611_t *ms5611, uint8_t osr)
{
    uint8_t cmd;
    cmd = MS5611_CMD_ADC_PRESS(osr);
    return ms5611_adc_read(ms5611, cmd);
}

uint32_t ms5611_temp_adc_read(ms5611_t *ms5611, uint8_t osr)
{
    uint8_t cmd;
    cmd = MS5611_CMD_ADC_TEMP(osr);
    return ms5611_adc_read(ms5611, cmd);
}

// uint32_t ms5611_temp_adc_read(ms5611_t *ms5611, uint8_t osr)
// {
//     uint8_t cmd;
//     uint8_t buf[3];

//     cmd = MS5611_CMD_ADC_TEMP(osr);
//     /* send measurement command */
//     i2c_write(&ms5611->dev, &cmd, sizeof(cmd));

//     /* sleep for needed conversion time */
//     os_thread_sleep_least_us(ms5611_osr_dly_us[osr]);

//     cmd = MS5611_CMD_ADC_READ;
//     /* send ADC read command */
//     i2c_write(&ms5611->dev, &cmd, 1);

//     /* read result */
//     i2c_read(&ms5611->dev, buf, 3);

//     /* setup 24bit result, MSByte received first */
//     return (uint32_t) buf[2]|(buf[1]<<8)|(buf[0]<<16);
// }

int32_t ms5611_calculate_temperature(int32_t dt)
{
    int32_t temp = (int32_t) (2000 + (((int64_t)dt*ms5611_prom.tempsens)>>23));
    return temp;
}

int32_t ms5611_calculate_dt(uint32_t raw_temp)
{
    return raw_temp - ((uint32_t)ms5611_prom.tref<<8);
}

uint32_t ms5611_calculate_pressure(uint32_t raw_p, int32_t dt, int32_t temp)
{
    int64_t off = ((int64_t)ms5611_prom.off<<16)+(((int64_t)ms5611_prom.tco*dt)>>7);
    int64_t sens = ((int64_t)ms5611_prom.sens<<15)+(((int64_t)dt*ms5611_prom.tcs)>>8);
    // second order temperature compensation, see datasheet
    if (temp < 2000) {  // low temperature correcture
        uint32_t t2 = (uint32_t)(SQUARE((int64_t)dt)>>31);
        temp = temp - t2;
        uint32_t off2 = (uint32_t) (5*SQUARE((int64_t)temp-2000))>>1;
        uint32_t sens2 = off2>>1;
        if (temp < -1500) { // very low temperature correcture
            uint32_t temp_square = SQUARE(temp+1500);
            off2 = off2 + 7*temp_square;
            sens2 = sens2 + ((11*temp_square)>>1);
        }
        off = off - off2;
        sens = sens - sens2;
    }
    uint32_t p = (uint32_t) ((raw_p*sens>>21)-off)>>15;
    return p;
}

float ms5611_calculate_pressure_float(uint32_t raw_p, int32_t dt, int32_t temp)
{
    int64_t off = ((int64_t)ms5611_prom.off<<16)+(((int64_t)ms5611_prom.tco*dt)>>7);
    int64_t sens = ((int64_t)ms5611_prom.sens<<15)+(((int64_t)dt*ms5611_prom.tcs)>>8);
    // second order temperature compensation, see datasheet
    if (temp < 2000) {  // low temperature correcture
        uint32_t t2 = (uint32_t)(SQUARE((int64_t)dt)>>31);
        temp = temp - t2;
        uint32_t off2 = (uint32_t) (5*SQUARE((int64_t)temp-2000))>>1;
        uint32_t sens2 = off2>>1;
        if (temp < -1500) { // very low temperature correcture
            uint32_t temp_square = SQUARE(temp+1500);
            off2 = off2 + 7*temp_square;
            sens2 = sens2 + ((11*temp_square)>>1);
        }
        off = off - off2;
        sens = sens - sens2;
    }
    uint32_t p = (float) ((raw_p*sens>>21)-off)/(1<<15);
    return p;
}

// uint32_t ms5611_get_pressure(void)
// {
//     static int32_t dt;
//     static uint8_t temperature_measurement_count = 0;
//     if (temperature_measurement_count == 0) {
//         ms5611_start_measurement(TEMPERATURE_OSR);
//         os_sleep(TEMPERATURE_OSR_DELAY);
//         uint32_t rt = ms5611_get_measurement_result();
//         if (rt == 0)
//             return 0;
//         dt = ms5611_calculate_dt(rt);
//         temperature_measurement_count = TEMPERATURE_MEASUREMENT_INTERVAL;
//     }
//     ms5611_start_measurement(PRESSURE_OSR);
//     os_sleep(PRESSURE_OSR_DELAY );
//     uint32_t raw_pressure = ms5611_get_measurement_result();
//     if (raw_pressure == 0)
//         return 0;
//     --temperature_measurement_count;
//     uint32_t pressure = ms5611_calculate_pressure(raw_pressure, dt, ms5611_calculate_temperature(dt));
//     return pressure;
// }
