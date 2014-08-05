// STM32F4 I2C

#ifndef I2C_H
#define I2C_H

#include <stdint.h>
#include <stdbool.h>

#define USE_I2C1
// #define USE_I2C2
// #define USE_I2C3

/* i2c_bus_t: physical I2C bus
 * multiple I2C slaves can be connected to the same bus. Access is thread safe.
 */
typedef struct i2c_bus i2c_bus_t;

/* i2c_dev_t: I2C slave device
 * an I2C slave is a chip with an address on a I2C bus
 * Error handling of an i2c_dev_t is not thread safe (but bus access is).
 * Therefore if multiple threads must access the same slave, external mutual
 * exclusion must be used to get correct error codes.
 */
typedef struct {
	i2c_bus_t *bus;
	uint16_t addr; // 7bit:high=0,low=addr<<1, 10bit:high=header+addr[8:9],low=addr[0:7]
	uint8_t error;
	// sticky_error: if set, the error is not cleared before a new transfer 
	// if an error is present, the transfer will not be executed
	bool sticky_error;
} i2c_dev_t;

// Error bits for i2c_dev_t->error (error status)
#define I2C_ADDR_NACK  (1<<0)
#define I2C_DATA_NACK  (1<<1)
#define I2C_NACK (I2C_ADDR_NACK | I2C_DATA_NACK)
#define I2C_BUS_ERROR  (1<<2)
#define I2C_ARB_LOST   (1<<3)
#define I2C_TIMEOUT    (1<<4)


#ifdef USE_I2C1
extern i2c_bus_t dev_i2c1;
#endif
#ifdef USE_I2C2
extern i2c_bus_t dev_i2c2;
#endif
#ifdef USE_I2C3
extern i2c_bus_t dev_i2c3;
#endif

#ifndef I2C1_SPEED
#define I2C1_SPEED 400000
#endif
#ifndef I2C2_SPEED
#define I2C2_SPEED 400000
#endif
#ifndef I2C3_SPEED
#define I2C3_SPEED 400000
#endif

#if defined(__cplusplus)
extern "C" {
#endif

// initialize all I2C buses
void i2c_init(void);
uint8_t i2c_write(i2c_dev_t *dev, uint8_t *data, uint32_t len);
uint8_t i2c_read(i2c_dev_t *dev, uint8_t *data, uint32_t len);
uint8_t i2c_error_status(i2c_dev_t *dev);
// reset error: only needed if sticky_error is enabled
uint8_t i2c_reset_error(i2c_dev_t *dev);
void i2c_enable_sticky_error(i2c_dev_t *dev);
void i2c_disable_sticky_error(i2c_dev_t *dev);
// initialize I2C slave device (with 7 bit address)
// addr: right aligned 7 bit address
void i2c_device_init(i2c_dev_t *dev, i2c_bus_t *bus, uint8_t addr);
// initialize I2C slave device (with 10 bit address)
// addr: right aligned 10 bit address
void i2c_device10_init(i2c_dev_t *dev, i2c_bus_t *bus, uint16_t addr);

#if defined(__cplusplus)
}
#endif

#endif // I2C_H
