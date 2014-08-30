#include <stdbool.h>

void mpu_spi_init(void);
bool mpu_ping(void);
void i2c_gpio_init(void);
bool EEPROM_ping(void);
bool MS5611_ping(void);
bool H3LIS331DL_ping(void);
bool HMC5883L_ping(void);
