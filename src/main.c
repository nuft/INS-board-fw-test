
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>
#include "mpu60X0.h"

void delay(unsigned int n)
{
    while (n-- > 0) {
        __asm__ volatile ("nop":::);
    }
}

mpu60X0_t mpu;

void mpu_spi_init(void)
{
    // SPI1 SCK (PA5), MISO (PA6), MOSI (PA7) GPIO setup
    gpio_set_af(GPIOA, GPIO_AF5, GPIO5 | GPIO6 | GPIO7);
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO5 | GPIO6 | GPIO7);
    // SCK and MOSI as pushpull
    gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO5 | GPIO7);

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

int main(void)
{
    rcc_clock_setup_hse_3v3(&hse_16mhz_3v3[CLOCK_3V3_168MHZ]);

    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOC);

    // LEDs
    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO8 | GPIO10);
    gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO14 | GPIO15);
    gpio_clear(GPIOA, GPIO10);
    gpio_clear(GPIOB, GPIO14);

    // SD LED on
    gpio_set(GPIOB, GPIO15);

    // VCC_IO enable
    gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO5);
    gpio_set(GPIOB, GPIO5);

     // VCC_A enable
    gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO14);
    gpio_set(GPIOC, GPIO14);

    delay(1000000);

    mpu_spi_init();

    if (mpu60X0_ping(&mpu)) {
        gpio_set(GPIOB, GPIO14);
    } else {
        gpio_set(GPIOA, GPIO10);
    }

    while (1) {
        gpio_toggle(GPIOA, GPIO8);
        delay(1000000);
    }
    return 0;
}
