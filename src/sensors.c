
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>

#include <stdint.h>
#include <i2c.h>
#include <mpu60X0.h>

#include "i2c.h"
#include "sensors.h"

mpu60X0_t mpu;

void mpu_spi_init(void)
{
    // SPI1 SCK (PA5), MISO (PA6), MOSI (PA7) GPIO setup
    gpio_set_af(GPIOA, GPIO_AF5, GPIO5 | GPIO6 | GPIO7);
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO5 | GPIO6 | GPIO7);
    // gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLDOWN, GPIO6);
    // SCK and MOSI as pushpull
    // gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO5 | GPIO7);

    // MPU CS (PC4) GPIO setup
    gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO4);
    gpio_set_output_options(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO4);
    // CS high/inactive
    gpio_set(GPIOC, GPIO4);

    rcc_periph_clock_enable(RCC_SPI1);

    // SPI1 setup
    spi_set_clock_polarity_1(SPI1);
    spi_set_clock_phase_1(SPI1);
    spi_send_msb_first(SPI1);
    spi_set_dff_8bit(SPI1);

    spi_set_baudrate_prescaler(SPI1, SPI_CR1_BR_FPCLK_DIV_256);

    spi_enable_software_slave_management(SPI1);
    spi_set_nss_high(SPI1);

    spi_set_unidirectional_mode(SPI1);
    spi_set_full_duplex_mode(SPI1);
    spi_set_master_mode(SPI1);
    spi_enable(SPI1);

    mpu6000_init_using_spi(&mpu, SPI1, GPIOC, GPIO4);
}

bool mpu_ping(void)
{
    return mpu60X0_ping(&mpu);
}

void i2c_gpio_init(void)
{
    // init SCL (PB8), SDA (PB9)
    gpio_set_af(GPIOB, GPIO_AF4, GPIO8 | GPIO9);
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO8 | GPIO9);
    gpio_set_output_options(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_100MHZ, GPIO8 | GPIO9);
}

bool EEPROM_ping(void)
{
    i2c_dev_t eeprom;
    i2c_device_init(&eeprom, &dev_i2c1, 0x50);

    // start a read operation at addr 0x0000, check if I2C ACK
    uint8_t addr[] = {0, 0};
    uint8_t err = i2c_write(&eeprom, addr, 2);
    // if (err & I2C_ADDR_NACK)
    //     return false;
    if (err)
        return false;
    return true;
}

bool MS5611_ping(void)
{
    // 0xEE -> 0x77
    i2c_dev_t ms5611;
    i2c_device_init(&ms5611, &dev_i2c1, 0x77);

    /*
    uint8_t err = i2c_write(&ms5611, NULL, 0);
    if (err & I2C_ADDR_NACK== 0)
        return false;
    return true;

    /*/
    // send reset command
    uint8_t cmd = 0x1E;
    uint8_t err = i2c_write(&ms5611, &cmd, 1);
    if (err)
        return false;
    return true;
    //*/
}

// bool MS5611_ping(void)
// {
//     // 0xEE -> 0x77
//     i2c_dev_t ms5611;
//     i2c_device_init(&ms5611, &dev_i2c1, 0x77);

//     /*
//     uint8_t err = i2c_write(&ms5611, NULL, 0);
//     if ((err & I2C_ADDR_NACK) == 0)
//         return true;
//     return false;

//     /*/
//     // // send reset command
//     // uint8_t cmd = 0x1E;
//     // uint8_t err = i2c_write(&ms5611, &cmd, 1);
//     // if ((err & I2C_ADDR_NACK) == 0)
//     //     return true;
//     // return false;

//     // send prom read command
//     uint8_t cmd = 0xA0;
//     uint8_t err;
//     err = i2c_write(&ms5611, &cmd, 1);
//     if (err & I2C_ADDR_NACK)
//         return false;
//     uint8_t data[2];
//     err = i2c_read(&ms5611, data, 2);
//     if (err & I2C_ADDR_NACK)
//         return false;
//     return true;

//     //*/
// }

bool H3LIS331DL_ping(void)
{
    i2c_dev_t h3lis331dl;
    i2c_device_init(&h3lis331dl, &dev_i2c1, 0x19);
    // i2c_device_init(&h3lis331dl, &dev_i2c1, 0b00011001);
    // 0x32 -> 0x19

    /*
    uint8_t err = i2c_write(&h3lis331dl, NULL, 0);
    if ((err & I2C_ADDR_NACK) == 0)
        return true;
    return false;

    /*/
    // read WHO_AM_I register at 0x0F as 0b00110010 (0x32)
    uint8_t err;
    uint8_t addr = 0x0F;
    err = i2c_write(&h3lis331dl, &addr, 1);
    if (err & I2C_ADDR_NACK)
        return false;

    uint8_t data = 0;
    err = i2c_read(&h3lis331dl, &data, 1);
    if (err)
        return false;

    if (data == 0x32)
        return true;
    return false;
    //*/
}

bool HMC5883L_ping(void)
{
    i2c_dev_t hmc5883l;
    i2c_device_init(&hmc5883l, &dev_i2c1, 0x1E);
    // 0x3C -> 1E
    /*
    uint8_t err = i2c_read(&hmc5883l, NULL, 0);
    if (err & I2C_ADDR_NACK)
        return false;
    return true;
    /*/
    // read ID_REGISTER_A register at 0x0A as 'H'
    uint8_t err;
    uint8_t addr = 0x0A;
    err = i2c_write(&hmc5883l, &addr, 1);
    if (err)
        return false;
    uint8_t data = 0;
    err = i2c_read(&hmc5883l, &data, 1);
    if (err)
        return false;

    if (data == 'H')
        return true;
    return false;

    // uint8_t data[] = {0,0,0};
    // uint8_t addr = 0x0A;
    // i2c_write(&hmc5883l, &addr, 1);
    // i2c_read(&hmc5883l, &data[0], 1);

    // addr = 0x0B;
    // i2c_write(&hmc5883l, &addr, 1);
    // i2c_read(&hmc5883l, &data[1], 1);

    // addr = 0x0C;
    // i2c_write(&hmc5883l, &addr, 1);
    // i2c_read(&hmc5883l, &data[2], 1);
    // if (data[0] == 'H' && data[1] == '4' && data[0] == '3')
    //     return true;
    // return false;

    //*/
}