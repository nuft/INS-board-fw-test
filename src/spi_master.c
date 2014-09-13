

// #include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/spi.h>
#include "spi_master.h"

static inline void spi_slave_select(spi_slave_t *slave)
{
    gpio_clear(slave->chip_select_gpio_port, slave->chip_select_gpio_pin);
}

static inline void spi_slave_deselect(spi_slave_t *slave)
{
    gpio_set(slave->chip_select_gpio_port, slave->chip_select_gpio_pin);
}

void spi_bus_init(spi_bus_t *bus, uint32_t peripheral)
{
    bus->peripheral = peripheral;

    bus->config = 0;

    /* reconfiguration at first access */
    bus->error = SPI_ERR_RESET;

    // rcc initialization here ?
}

void spi_slave_device_init(spi_slave_t *slave, spi_bus_t *bus, uint32_t cs_port, uint16_t cs_pin, uint32_t config)
{
    slave->bus = bus;

    /* Bus configuration needed for this slave */
    slave->config = config;

    /* Chip select pin */
    slave->chip_select_gpio_port = cs_port;
    slave->chip_select_gpio_pin = cs_pin;
}

static void spi_bus_config(spi_bus_t *bus, uint32_t config)
{
    uint32_t spi = bus->peripheral;

    /* Control Register 1 default:
       Enable software slave management (SSM).
       Ignore NSS input pin.
       Data frame format (DFF) is 8bit.
       CRC disabled.
       fullduplex 2-line mode. */
    uint16_t cr1 = SPI_CR1_SSM | SPI_CR1_SSI;

    /* Clock phase */
    if (config & SPI_BUS_CFG_CLOCK_PHASE_MASK) {
        cr1 |= SPI_CR1_CPHA;
    }

    /* Clock polarity */
    if (config & SPI_BUS_CFG_CLOCK_POLARITY_MASK) {
        cr1 |= SPI_CR1_CPOL;
    }

    /* Data bit order */
    if (config & SPI_BUS_CFG_BIT_ORDER_MASK) {
        cr1 |= SPI_CR1_LSBFIRST;
    }

    /* Prescaler
       F_SPI_SCL(presc) = f_PCLK >> (presc + 1) */
    cr1 |= ((config & SPI_BUS_CFG_PRESC_MASK) >> SPI_BUS_CFG_PRESC_POS) << 3;

    SPI_CR1(spi) = cr1;

    /* Enable RX not empty and Error interrupts.
       TXE interrrupt is enabled at transmission start. */
    SPI_CR2(spi) =  SPI_CR2_RXNEIE | SPI_CR2_ERRIE;

    /* Enable SPI peripheral */
    SPI_CR1(spi) |= SPI_CR1_MSTR | SPI_CR1_SPE;

    bus->config = config;
}

int spi_master_access(spi_slave_t *slave, struct spi_transfer *t, uint8_t nb_transf)
{
    int err;
    spi_bus_t *bus = slave->bus;

    os_mutex_take(&bus->mutex);

    if (bus->config != slave->config || bus->error != SPI_ERR_NONE) {
        spi_bus_config(slave->bus, slave->config);
    }

    spi_slave_select(slave);

    bus->error = SPI_ERR_NONE;
    bus->transfer = t;
    bus->nb_transfers = nb_transf;
    bus->rx_pos = 0;
    bus->tx_pos = 0;
    bus->rx_transfer = 0;
    bus->tx_transfer = 0;

    /* Enable TXE interrupt to start spi transfer.
       Interrupt is triggered immediately */
    SPI_CR2(bus->peripheral) |= SPI_CR2_TXEIE;

    // os_semaphore_wait(&bus->done);
    os_semaphore_take(&bus->done);

    spi_slave_deselect(slave);

    err = bus->error;

    os_mutex_release(&bus->mutex);

    return err;
}

void spi_interrupt_handler(spi_bus_t *bus)
{
    uint32_t spi = bus->peripheral;
    uint16_t sr = SPI_SR(spi);

    /* Mode fault */
    if (sr & SPI_SR_MODF) {
        /* NSS pin was low during master mode, try to reenable spi */

        // clear MODF bit
        // SPI_SR(spi) &= ~SPI_SR_MODF;

        /* reset to master mode */
        // SPI_CR1(spi) |= SPI_CR1_MSTR | SPI_CR1_SPE;

        bus->error = SPI_ERR_MODE_FAULT;

        // os_semaphore_signal(&bus->done);
        os_semaphore_release(&bus->done);
        /* disable SPI peripheral */
        SPI_CR1(spi) &= ~SPI_CR1_SPE;
        return;
    }

    /* CRC Error flag */
    if (sr & SPI_SR_CRCERR) {
        // crc not used
        bus->error = SPI_ERR_CRC;
        // os_semaphore_signal(&bus->done);
        os_semaphore_release(&bus->done);
        /* disable SPI peripheral */
        SPI_CR1(spi) &= ~SPI_CR1_SPE;
        return;
    }

    /* Overrun flag */
    if (sr & SPI_SR_OVR) {
        // slave mode only
        bus->error = SPI_ERR_OVERRUN;
        // os_semaphore_signal(&bus->done);
        os_semaphore_release(&bus->done);
        /* disable SPI peripheral */
        SPI_CR1(spi) &= ~SPI_CR1_SPE;
        return;
    }

    /* Receive buffer Not Empty */
    if (sr & SPI_SR_RXNE) {
        struct spi_transfer *t = &bus->transfer[bus->rx_transfer];
        uint16_t pos = bus->rx_pos;
        uint8_t rxd = SPI_DR(spi);   /* read clears RXNE flag */

        if (t->r != NULL) {
            t->r[pos] = rxd;
        }

        if (t->len > ++pos) {
            bus->rx_pos = pos;
        } else {
            /* next transfer */
            bus->rx_pos = 0;
            bus->rx_transfer += 1;
            if (bus->rx_transfer >= bus->nb_transfers) {
                /* Last byte received, signal thread */
                // os_semaphore_signal(&bus->done);
                os_semaphore_release(&bus->done);
            }
        }
    }

    /* Transmit buffer Empty */
    if (sr & SPI_SR_TXE) {
        if (bus->tx_transfer < bus->nb_transfers) {
            struct spi_transfer *t = &bus->transfer[bus->tx_transfer];
            uint16_t pos = bus->tx_pos;

            if (t->t != NULL) {
                SPI_DR(spi) = t->t[pos];
            } else {
                SPI_DR(spi) = 0;    /* send 0 per default */
            }

            bus->tx_pos++;
            if (bus->tx_pos >= t->len) {
                /* next transfer */
                bus->tx_pos = 0;
                bus->tx_transfer += 1;

                if (bus->tx_transfer >= bus->nb_transfers) {
                    /* Last byte sent, disable TXE interrupt */
                    SPI_CR2(spi) &= ~SPI_CR2_TXEIE;
                }
            }
        } else {
            SPI_CR2(spi) &= ~SPI_CR2_TXEIE;
        }
    }

    /* Mode fault */
    if (sr & SPI_SR_MODF) {
        /* NSS pin was low during master mode, try to reenable spi */

        // clear MODF bit
        // SPI_SR(spi) &= ~SPI_SR_MODF;

        /* reset to master mode */
        // SPI_CR1(spi) |= SPI_CR1_MSTR | SPI_CR1_SPE;

        bus->error = SPI_ERR_MODE_FAULT;

        // os_semaphore_signal(&bus->done);
        os_semaphore_release(&bus->done);
    }

    /* CRC Error flag */
    if (sr & SPI_SR_CRCERR) {
        // crc not used
        bus->error = SPI_ERR_CRC;
    }

    /* Overrun flag */
    if (sr & SPI_SR_OVR) {
        // slave mode only
        bus->error = SPI_ERR_OVERRUN;
    }
}

int spi_single_access(spi_slave_t *slave, uint8_t *r, uint8_t *t, uint16_t len)
{
    struct spi_transfer tf = {.r = r, .t = t, .len = len};
    return spi_master_access(slave, &tf, 1);
}
