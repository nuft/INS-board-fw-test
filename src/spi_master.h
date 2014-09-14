#ifndef SPI_MASTER_H
#define SPI_MASTER_H

#include <stdint.h>
#include <platform-abstraction/semaphore.h>
#include <platform-abstraction/mutex.h>
#include <libopencm3/stm32/gpio.h>

/* SPI config */

/* Phase */
/* 0: first edge, 1: second edge */
#define SPI_BUS_CFG_CLOCK_PHASE_POS                 0
#define SPI_BUS_CFG_CLOCK_PHASE_MASK                (1<<0)
#define SPI_BUS_CFG_CLOCK_PHASE_FIRST_TRANSITION    (0<<0)
#define SPI_BUS_CFG_CLOCK_PHASE_SECOND_TRANSITION   (1<<0)

/* Polarity */
/* 0: idle low, 1: idle high */
#define SPI_BUS_CFG_CLOCK_POLARITY_POS              1
#define SPI_BUS_CFG_CLOCK_POLARITY_MASK             (1<<1)
#define SPI_BUS_CFG_CLOCK_POLARITY_IDLE_LOW         (0<<1)
#define SPI_BUS_CFG_CLOCK_POLARITY_IDLE_HIGH        (1<<1)

/* Bit order */
/* 0: msb first, 1: lsb first */
#define SPI_BUS_CFG_BIT_ORDER_POS                   2
#define SPI_BUS_CFG_BIT_ORDER_MASK                  (1<<2)
#define SPI_BUS_CFG_BIT_ORDER_MSB_FIRST             (0<<2)
#define SPI_BUS_CFG_BIT_ORDER_LSB_FIRST             (1<<2)

/* SPI prescaler */
#define SPI_BUS_CFG_PRESC_POS                       3
#define SPI_BUS_CFG_PRESC_MASK                      (7<<3)
#define SPI_BUS_CFG_PRESC_2                         (0<<3)
#define SPI_BUS_CFG_PRESC_4                         (1<<3)
#define SPI_BUS_CFG_PRESC_8                         (2<<3)
#define SPI_BUS_CFG_PRESC_16                        (3<<3)
#define SPI_BUS_CFG_PRESC_32                        (4<<3)
#define SPI_BUS_CFG_PRESC_64                        (5<<3)
#define SPI_BUS_CFG_PRESC_128                       (6<<3)
#define SPI_BUS_CFG_PRESC_256                       (7<<3)

/* Reserved bits */
#define SPI_BUS_CFG_RESERVED_BITS_POS               4
#define SPI_BUS_CFG_RESERVED_BITS_MASK              (0xFFFFFFFF<<4)

/* SPI error bits */
#define SPI_ERR_NONE        0
// SPI master mode fault: interface disabled, foced to slave mode
#define SPI_ERR_MODE_FAULT  1
// received CRC does not match, (only when CRC is enabled)
#define SPI_ERR_CRC         2
// RX Buffer overrun and receive data loss (SPI slave mode only)
#define SPI_ERR_OVERRUN     3
// SPI bus is not initialized (after reset)
#define SPI_ERR_RESET       4

/* SPI tansmission data structure */
typedef struct {
    uint8_t *tx;
    uint8_t *rx;
    uint16_t len;
} spi_access_t;

/* SPI bus */
typedef struct {
    uint32_t peripheral;
    semaphore_t done;
    mutex_t mutex;
    int error;
    uint32_t config;
    enum {
        spi_mode_multi_access,
        spi_mode_single_access,
        spi_mode_request
    } mode;
    union {
        struct {
            spi_access_t *access;
            uint16_t rx_pos;
            uint16_t tx_pos;
            uint8_t rx_acc_pos;
            uint8_t tx_acc_pos;
            uint8_t nb_acc;
        } multi;
        struct {
            uint8_t *tx;
            uint8_t *rx;
            uint16_t len;
        } single;
        struct {
            uint8_t reqest;
            uint8_t *response;
            uint16_t len;
        } request;
    } access;
} spi_bus_t;

/* SPI slave device */
typedef struct {
    spi_bus_t *bus;
    uint32_t config;
    uint32_t chip_select_gpio_port;
    uint16_t chip_select_gpio_pin;
} spi_slave_t;

/** Initialize SPI bus */
void spi_bus_init(spi_bus_t *bus, uint32_t peripheral);

/** Initialize a SPI slave for a bus */
void spi_slave_device_init(spi_slave_t *slave, spi_bus_t *bus, uint32_t cs_port, uint16_t cs_pin, uint32_t config);

/** Access SPI bus as master */
int spi_access(spi_slave_t *slave, uint8_t *tx,  uint8_t *rx, uint16_t len);

int spi_multi_access(spi_slave_t *slave, spi_access_t *acc, uint8_t nb_acc);

int spi_request(spi_slave_t *slave, uint8_t reqest, uint8_t *response, uint16_t response_len);

/** SPI Interrupt handler.
 *  Must be called from each SPI interrupt with the corresponding bus pointer.
 */
void spi_master_interrupt_handler(spi_bus_t *bus);

#endif /* SPI_MASTER_H */
