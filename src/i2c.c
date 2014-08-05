#include <stdint.h>
#include <stdbool.h>
#include <stm32f4xx.h>
#include "i2c.h"

// I2C address read/write bit
#define I2C_W 0
#define I2C_R 1

#define I2CDevHas10BitAddress(addr) (addr & 0xFF00)

// todo
extern const uint32_t f_apb1;

typedef struct {
    volatile bool wake_signal;
} thread_queue_t;

void thread_queue_init(thread_queue_t *q)
{
    q->wake_signal = false;
}

void thread_queue_wake(thread_queue_t *q)
{
    q->wake_signal = true;
}

void thread_queue_wait(thread_queue_t *q, uint32_t timeout)
{
    (void) timeout;
    while (!q->wake_signal);
    q->wake_signal = false;
}

// interrupt state machine for I2C master
enum master_transfer_state { idle,
    t_setup, t_sent_start, t_sent_addr10_header,
     t_sent_addr, t_data,
    r_setup, r_sent_start, r_sent_addr10_header, r_sent_addr10_addr,
     r_sent_addr10_repeated_start,
     r_sent_addr, r_data
};

// type i2c_bus_t
struct i2c_bus {
    I2C_TypeDef *hw; // Registers
    uint32_t speed; // [kbps]
    struct {
        i2c_dev_t *dev;
        uint8_t *data;
        uint32_t len;
        uint32_t data_cnt;
        enum master_transfer_state state;
        thread_queue_t transfer_complete;
    } master;
};

#ifdef USE_I2C1
i2c_bus_t dev_i2c1;
#endif
#ifdef USE_I2C2
i2c_bus_t dev_i2c2;
#endif
#ifdef USE_I2C3
i2c_bus_t dev_i2c3;
#endif

static void enable_i2c_interrupts(I2C_TypeDef *i2c);
static void disable_i2c_interrupts(I2C_TypeDef *i2c);
void i2c_event_interrupt_handler(i2c_bus_t *bus);
void i2c_error_interrupt_handler(i2c_bus_t *bus);
static void i2c_bus_reset(i2c_bus_t *bus);
static void i2c_bus_init(i2c_bus_t *bus);
static void i2c_master_transfer(i2c_bus_t *bus);


static void enable_i2c_interrupts(I2C_TypeDef *i2c)
{
    if (i2c == I2C1) {
        NVIC_EnableIRQ(I2C1_EV_IRQn);
        NVIC_EnableIRQ(I2C1_ER_IRQn);
    } else if (i2c == I2C2) {
        NVIC_EnableIRQ(I2C2_EV_IRQn);
        NVIC_EnableIRQ(I2C2_ER_IRQn);
    } else if (i2c == I2C3) {
        NVIC_EnableIRQ(I2C3_EV_IRQn);
        NVIC_EnableIRQ(I2C3_ER_IRQn);
    }
}

static void disable_i2c_interrupts(I2C_TypeDef *i2c)
{
    if (i2c == I2C1) {
        NVIC_DisableIRQ(I2C1_EV_IRQn);
        NVIC_DisableIRQ(I2C1_ER_IRQn);
    } else if (i2c == I2C2) {
        NVIC_DisableIRQ(I2C2_EV_IRQn);
        NVIC_DisableIRQ(I2C2_ER_IRQn);
    } else if (i2c == I2C3) {
        NVIC_DisableIRQ(I2C3_EV_IRQn);
        NVIC_DisableIRQ(I2C3_ER_IRQn);
    }
}

void i2c_event_interrupt_handler(i2c_bus_t *bus)
{
    I2C_TypeDef *i2c = bus->hw;
    uint16_t sr1 = i2c->SR1;
    // Start Bit (master EV5)
    if (sr1 & I2C_SR1_SB) {
        if (bus->master.state == t_setup) {
            bus->master.state = t_sent_start;
            if (I2CDevHas10BitAddress(bus->master.dev->addr)) {
                // send 10bit header + w
                i2c->DR = (uint8_t)((bus->master.dev->addr)>>8) + I2C_W;
            } else {
                // send 7bit address + w
                i2c->DR = (uint8_t)(bus->master.dev->addr) + I2C_W;
            }
        } else if (bus->master.state == r_setup) {
            bus->master.state = r_sent_start;
            if (I2CDevHas10BitAddress(bus->master.dev->addr)) {
                // send 10bit header + w
                i2c->DR = (uint8_t)((bus->master.dev->addr)>>8) + I2C_W;
            } else {
                // send 7bit address + r
                i2c->DR = (uint8_t)(bus->master.dev->addr) + I2C_R;
            }
        } else if (bus->master.state == r_sent_addr10_addr) {
            bus->master.state = r_sent_addr10_repeated_start;
            // send 10bit header + r
            i2c->DR = (uint8_t)((bus->master.dev->addr)>>8) + I2C_R;
        }
    }
    // Address sent (master EV6) / matched (slave)
    if (sr1 & I2C_SR1_ADDR) {
        if (bus->master.state == t_sent_start ||
            bus->master.state == t_sent_addr10_header) {
            bus->master.state = t_sent_addr;
            // Clear ADDR condition sequence
            uint32_t sr2 = i2c->SR2; (void)sr2;
            // start data
            bus->master.state = t_data;
            i2c->CR2 |= I2C_CR2_ITBUFEN;
        } else if (bus->master.state == r_sent_start ||
            bus->master.state == r_sent_addr10_repeated_start) {
            bus->master.state = r_sent_addr;
            if (bus->master.len == 0) {
                i2c->CR1 |= I2C_CR1_STOP;
                bus->master.state = idle;
                thread_queue_wake(&bus->master.transfer_complete);
            } else {
                if (bus->master.len == 2) {
                    i2c->CR1 |= I2C_CR1_POS;
                }
                if (bus->master.len <= 2) {
                    i2c->CR1 &= ~I2C_CR1_ACK;
                }
                // Clear ADDR condition sequence
                uint32_t sr2 = i2c->SR2; (void)sr2;
                // start data
                bus->master.state = r_data;
                i2c->CR2 |= I2C_CR2_ITBUFEN;
            }
        } else if (bus->master.state == r_sent_addr10_header) {
            bus->master.state = r_sent_addr10_addr;
            // Clear ADDR condition sequence
            uint32_t sr2 = i2c->SR2; (void)sr2;
            // generate repeated start
            i2c->CR1 |= I2C_CR1_START;
        } else {
            // slave address matched
            uint32_t sr2 = i2c->SR2; (void)sr2; // clear
        }
    }
    // 10-bit header sent (master EV9)
    if (sr1 & I2C_SR1_ADD10) {
        if (bus->master.state == t_sent_start) {
            bus->master.state = t_sent_addr10_header;
            // send 10bit address
            i2c->DR = (uint8_t)(bus->master.dev->addr);
        } else if (bus->master.state == r_sent_start) {
            bus->master.state = r_sent_addr10_header;
            // send 10bit address
            i2c->DR = (uint8_t)(bus->master.dev->addr);
        }
    }
    // Stop detection (slave)
    if (sr1 & I2C_SR1_STOPF) {

    }
    // Data register empty (transmitters)
    if (sr1 & I2C_SR1_TXE) {
        if (bus->master.state == t_data) {
            if (bus->master.data_cnt < bus->master.len) {
                bus->master.data_cnt++;
                i2c->DR = *bus->master.data++;
            }
            if (bus->master.data_cnt == bus->master.len) { // last byte loaded
                i2c->CR2 &= ~I2C_CR2_ITBUFEN; // disable TXE interrupt
                if (sr1 & I2C_SR1_BTF) {
                    i2c->CR1 |= I2C_CR1_STOP; // send stop
                    uint32_t dr = i2c->DR; (void)dr; // clear BTF interrupt
                    bus->master.state = idle;
                    thread_queue_wake(&bus->master.transfer_complete);
                }
            }
        }
    }
    // Data register not empty (receivers)
    if (sr1 & I2C_SR1_RXNE) {
        if (bus->master.state == r_data) {
            if (bus->master.len == 1) {
                i2c->CR1 |= I2C_CR1_STOP; // stop condition
                *bus->master.data = i2c->DR; // read
                i2c->CR1 |= I2C_CR1_ACK; // enable ack
                bus->master.state = idle;
                thread_queue_wake(&bus->master.transfer_complete);
            } else if (bus->master.len == 2) {
                bus->master.data_cnt++;
                if (bus->master.data_cnt == 1) {
                    i2c->CR1 |= I2C_CR1_STOP; // stop condition
                    *bus->master.data++ = i2c->DR; // read
                } else if (bus->master.data_cnt == 2) {
                    *bus->master.data++ = i2c->DR; // read
                    i2c->CR1 |= I2C_CR1_ACK; // enable ack
                    i2c->CR1 &= ~I2C_CR1_POS; // clear pos
                    bus->master.state = idle;
                    thread_queue_wake(&bus->master.transfer_complete);
                }
            } else if (bus->master.len > 2) {
                if (bus->master.data_cnt +4 <= bus->master.len) {
                    bus->master.data_cnt++;
                    *bus->master.data++ = i2c->DR; // read
                } else if (bus->master.data_cnt == bus->master.len - 3) {
                    i2c->CR2 &= ~I2C_CR2_ITBUFEN; // disable RXNE without BTF interrupt
                    if (sr1 & I2C_SR1_BTF) {
                        i2c->CR1 &= ~I2C_CR1_ACK; // clear ack
                        bus->master.data_cnt++;
                        *bus->master.data++ = i2c->DR; // read N-2
                    }
                } else if (bus->master.data_cnt == bus->master.len - 2) {
                    if (sr1 & I2C_SR1_BTF) {
                        i2c->CR1 |= I2C_CR1_STOP; // stop condition
                        bus->master.data_cnt++;
                        *bus->master.data++ = i2c->DR; // read N-1
                        bus->master.data_cnt++;
                        *bus->master.data++ = i2c->DR; // read N
                        i2c->CR1 |= I2C_CR1_ACK; // enable ack
                        bus->master.state = idle;
                        thread_queue_wake(&bus->master.transfer_complete);
                    }
                }
            }
        }
    }
}

#ifdef USE_I2C1
void i2c1_ev_isr(void)
{
    i2c_event_interrupt_handler(&dev_i2c1);
}
#endif
#ifdef USE_I2C2
void i2c2_ev_isr(void)
{
    i2c_event_interrupt_handler(&dev_i2c2);
}
#endif
#ifdef USE_I2C3
void i2c3_ev_isr(void)
{
    i2c_event_interrupt_handler(&dev_i2c3);
}
#endif

void i2c_error_interrupt_handler(i2c_bus_t *bus)
{
    uint16_t sr1 = bus->hw->SR1;
    // Bus error (external Stop or Start condition during address/data transfer)
    if (sr1 & I2C_SR1_BERR) {
        if (bus->master.state != idle) {
            bus->master.dev->error |= I2C_BUS_ERROR;
            bus->master.state = idle;
            thread_queue_wake(&bus->master.transfer_complete);
        }
        bus->hw->SR1 = ~I2C_SR1_BERR; // clear interrupt flag
    }
    // Acknowledge failure
    if (sr1 & I2C_SR1_AF) {
        if (bus->master.state == t_data) {
            bus->master.dev->error |= I2C_DATA_NACK;
            bus->hw->CR1 |= I2C_CR1_STOP; // stop condition
            bus->master.state = idle;
            thread_queue_wake(&bus->master.transfer_complete);
        } else if (bus->master.state != idle) {
            bus->master.dev->error |= I2C_ADDR_NACK;
            bus->hw->CR1 |= I2C_CR1_STOP; // stop condition
            bus->master.state = idle;
            thread_queue_wake(&bus->master.transfer_complete);
        }
        bus->hw->SR1 = ~I2C_SR1_AF; // clear interrupt flag
    }
    // Arbitration lost
    if (sr1 & I2C_SR1_ARLO) {
        if (bus->master.state != idle) {
            bus->master.dev->error |= I2C_ARB_LOST;
            bus->master.state = idle;
            thread_queue_wake(&bus->master.transfer_complete);
        }
        bus->hw->SR1 = ~I2C_SR1_ARLO; // clear interrupt flag
    }
    // Overrun/underrun error
    if (sr1 & I2C_SR1_OVR) {
        bus->hw->SR1 = ~I2C_SR1_OVR; // clear interrupt flag
    }
    // PEC Error in reception
    if (sr1 & I2C_SR1_PECERR) {
        bus->hw->SR1 = ~I2C_SR1_PECERR; // clear interrupt flag
    }
    // Timeout / Tlow error (SMBus)
    if (sr1 & I2C_SR1_TIMEOUT) {
        bus->hw->SR1 = ~I2C_SR1_TIMEOUT; // clear interrupt flag
    }
    // SMBus alert
    if (sr1 & I2C_SR1_SMBALERT) {
        bus->hw->SR1 = (uint16_t)~I2C_SR1_SMBALERT; // clear interrupt flag
    }
}

#ifdef USE_I2C1
void i2c1_er_isr(void)
{
    i2c_error_interrupt_handler(&dev_i2c1);
}
#endif
#ifdef USE_I2C2
void i2c2_er_isr(void)
{
    i2c_error_interrupt_handler(&dev_i2c2);
}
#endif
#ifdef USE_I2C3
void i2c3_er_isr(void)
{
    i2c_error_interrupt_handler(&dev_i2c3);
}
#endif

// used in task context to recover from a bus error or timeout
static void i2c_bus_reset(i2c_bus_t *bus)
{
    disable_i2c_interrupts(bus->hw);
    bus->hw->CR1 |= I2C_CR1_SWRST;
    // TODO: scl clock 1 byte with NACK, stop
    i2c_bus_init(bus);
}

// initialize the I2C bus struct & hardware
static void i2c_bus_init(i2c_bus_t *bus)
{
    thread_queue_init(&bus->master.transfer_complete);
    if (bus->hw == I2C1) {
        RCC->APB1RSTR |= RCC_APB1RSTR_I2C1RST;
        RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C1RST;
    } else if (bus->hw == I2C2) {
        RCC->APB1RSTR |= RCC_APB1RSTR_I2C2RST;
        RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C2RST;
    } else if (bus->hw == I2C3) {
        RCC->APB1RSTR |= RCC_APB1RSTR_I2C3RST;
        RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C3RST;
    }
    I2C_TypeDef *i2c = bus->hw;
    // Disable the I2C before changing any configuration.
    i2c->CR1 = 0;
    // set clock and interrupts (note: apb1 must be a multiple of 10MHz to reach 400kHz speed)
    i2c->CR2 = (0x003F&(f_apb1/1000000)) | I2C_CR2_ITERREN | I2C_CR2_ITBUFEN | I2C_CR2_ITEVTEN;
    // fast mode, low/high: 16/9
    // 1/f_i2c = tlow+thigh = (9+16)*ccr*tapb1
    // ccr = f_apb1/f_i2c *1/(9+16)
    uint16_t ccr = (f_apb1/(bus->speed*25)) & 0x0FFF;
    i2c->CCR = I2C_CCR_FS | I2C_CCR_DUTY | ccr;
    // rise time 400kHz: 300ns, 100kHz: 1000ns
    // trise = risetime/t_apb1 + 1
    i2c->TRISE = (((300*(f_apb1/1000000))/1000)+1) & 0x1F;
    // configured, enable now
    i2c->CR1 |= I2C_CR1_PE;

    enable_i2c_interrupts(bus->hw);
}

// initialize all I2C buses
void i2c_init(void)
{
#ifdef USE_I2C1
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN; // enable clock
    dev_i2c1.hw = I2C1;
    dev_i2c1.speed = I2C1_SPEED;
    i2c_bus_init(&dev_i2c1);
#endif
#ifdef USE_I2C2
    RCC->APB1ENR |= RCC_APB1ENR_I2C2EN; // enable clock
    dev_i2c2.hw = I2C2;
    dev_i2c2.speed = I2C2_SPEED;
    i2c_bus_init(&dev_i2c2);
#endif
#ifdef USE_I2C3
    RCC->APB1ENR |= RCC_APB1ENR_I2C3EN; // enable clock
    dev_i2c3.hw = I2C3;
    dev_i2c3.speed = I2C3_SPEED;
    i2c_bus_init(&dev_i2c3);
#endif
}

// start master transfer, called from i2c_write or i2c_read
static void i2c_master_transfer(i2c_bus_t *bus)
{
    // wait for idle bus
    while (bus->hw->SR2 & I2C_SR2_BUSY) { // todo : with timeout
    }
    disable_i2c_interrupts(bus->hw);
    bus->hw->CR1 |= I2C_CR1_START;
    enable_i2c_interrupts(bus->hw);
    // wait for transfer completion
    thread_queue_t *q = &bus->master.transfer_complete;
    if (bus->master.state != idle) {
        thread_queue_wait(q, 0);
    }
    // reset bus if an error occured
    if (bus->master.dev->error == (I2C_BUS_ERROR | I2C_TIMEOUT))
        i2c_bus_reset(bus);
}

uint8_t i2c_write(i2c_dev_t *dev, uint8_t *data, uint32_t len)
{
    if (dev->sticky_error) {
        if (dev->error)
            return dev->error;
    } else {
        dev->error = 0;
    }
    dev->bus->master.dev = dev;
    dev->bus->master.data = data;
    dev->bus->master.len = len;
    dev->bus->master.data_cnt = 0;
    dev->bus->master.state = t_setup;
    i2c_master_transfer(dev->bus);
    return dev->error;
}

uint8_t i2c_read(i2c_dev_t *dev, uint8_t *data, uint32_t len)
{


    if (dev->sticky_error) {
        if (dev->error)
            return dev->error;
    } else {
        dev->error = 0;
    }
    dev->bus->master.dev = dev;
    dev->bus->master.data = data;
    dev->bus->master.len = len;
    dev->bus->master.data_cnt = 0;
    dev->bus->master.state = r_setup;
    i2c_master_transfer(dev->bus);

    return dev->error;
}

uint8_t i2c_error_status(i2c_dev_t *dev)
{
    return dev->error;
}

uint8_t i2c_reset_error(i2c_dev_t *dev)
{
    uint8_t error = dev->error;
    dev->error = 0;
    return error;
}

void i2c_enable_sticky_error(i2c_dev_t *dev)
{
    dev->sticky_error = true;
}

void i2c_disable_sticky_error(i2c_dev_t *dev)
{
    dev->sticky_error = false;
}

void i2c_device_init(i2c_dev_t *dev, i2c_bus_t *bus, uint8_t addr)
{
    dev->bus = bus;
    dev->addr = addr<<1;
    dev->error = 0;
    dev->sticky_error = false;
}

void i2c_device10_init(i2c_dev_t *dev, i2c_bus_t *bus, uint16_t addr)
{
    dev->bus = bus;
    // 1111 0xx0  xxxx xxxx
    dev->addr = 0xF000 + (addr & 0x00FF) + ((addr & 0x0300)<<1);
    dev->error = 0;
    dev->sticky_error = false;
}
