#ifndef MPU60X0_H
#define MPU60X0_H

#include <stdint.h>
typedef struct {
    bool use_spi;
    uint32_t spi_dev;
    uint32_t cs_gpio_port;
    uint16_t cs_gpio_pin;
} mpu60X0_t;

// with full scale range of +- 250deg/s
#define MPU6050_LSB_PER_DEGpS 131

char mpu60X0_reg_read(mpu60X0_t *dev, uint8_t reg);
void mpu60X0_reg_write(mpu60X0_t *dev, uint8_t reg, char val);
void mpu60X0_reg_read_multi(mpu60X0_t *dev, uint8_t reg, char *buf, int8_t len);
void mpu60X0_reg_write_multi(mpu60X0_t *dev, uint8_t reg, const char *buf, int8_t len);

void mpu60X0_setup(mpu60X0_t *dev);
bool mpu60X0_ping(mpu60X0_t *dev);

void mpu6000_init_using_spi(mpu60X0_t *dev, uint32_t spi_dev, uint32_t cs_port, uint16_t cs_pin);


#endif // MPU60X0_H
