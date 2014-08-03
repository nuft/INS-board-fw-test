#include <stdint.h>

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>

#include "mpu60X0.h"
#include "mpu60X0_registers.h"


char mpu60X0_reg_read(mpu60X0_t *dev, uint8_t reg)
{
    char ret = 0;
    if (dev->use_spi) {
        gpio_clear(dev->cs_gpio_port, dev->cs_gpio_pin);
        spi_xfer(dev->spi_dev, reg);
        ret = spi_xfer(dev->spi_dev, 0);
        gpio_set(dev->cs_gpio_port, dev->cs_gpio_pin);
    }
    return ret;
}

void mpu60X0_reg_write(mpu60X0_t *dev, uint8_t reg, char val)
{
    if (dev->use_spi) {
        gpio_clear(dev->cs_gpio_port, dev->cs_gpio_pin);
        spi_xfer(dev->spi_dev, reg);
        spi_xfer(dev->spi_dev, val);
        gpio_set(dev->cs_gpio_port, dev->cs_gpio_pin);
    }
}

void mpu60X0_reg_read_multi(mpu60X0_t *dev, uint8_t reg, char *buf, int8_t len)
{
    // TODO
    // i2c_write(dev->, &reg, 1);
    // i2c_read(dev->, buf, len);
}

void mpu60X0_reg_write_multi(mpu60X0_t *dev, uint8_t reg, const char *buf, int8_t len)
{
    // TODO
    // i2c_write(dev->, &reg, 1);
}


void mpu6000_init_using_spi(mpu60X0_t *dev, uint32_t spi_dev, uint32_t cs_port, uint16_t cs_pin)
{
    dev->use_spi = true;
    dev->spi_dev = spi_dev;
    dev->cs_gpio_port = cs_port;
    dev->cs_gpio_pin = cs_pin;
}

void mpu60X0_setup(mpu60X0_t *dev)
{
    // select gyro x as clock source and disable sleep
    mpu60X0_reg_write(dev, MPU6050_RA_PWR_MGMT_1, MPU6050_CLOCK_PLL_XGYRO);
    // leave scale to default (+-250deg/s, +-2g)
    // leave sampling rate to default (DLPF=low pass: off) gyro:8kHz, acc:1kHz
    // enable interrupts
    mpu60X0_reg_write(dev, MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_DATA_RDY);
    // dlpf 5 (10hz)
    mpu60X0_reg_write(dev, MPU6050_RA_CONFIG, MPU6050_DLPF_BW_10);
}

bool mpu60X0_ping(mpu60X0_t *dev)
{
    if (mpu60X0_reg_read(dev, MPU6050_RA_WHO_AM_I) == 0x68) {
        return true;
    } else {
        return false;
    }
}



// ARM: little endian (lowest address = least significant)
// MPU6050: msbyte first

// // s16 data[] = {acc:x, y, z, temp, gyro:x, y, z}
// bool mpu6050_read(s16 *data)
// {
//     // 59 to 64 accelerometer
//     // 65 to 66 temperature
//     // 67 to 72 gyroscope
//     // total 14 bytes
//     u8 reg = 59;
//     i2c_write(&mpu6050, &reg, 1);
//     i2c_read(&mpu6050, (u8*)data, 14);
//     int i;
//     for (i=0; i < 7; i++)
//         data[i] = __REV16(data[i]);
//     return true;
// }
