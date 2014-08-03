
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/f4/pwr.h>
#include <libopencm3/stm32/f4/flash.h>

void delay(unsigned int n)
{
    while (n-- > 0) {
        __asm__ volatile ("nop":::);
    }
}

int main(void)
{
    rcc_clock_setup_hse_3v3(&hse_16mhz_3v3[CLOCK_3V3_168MHZ]);

    rcc_periph_clock_enable(RCC_GPIOA);
    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO8 | GPIO10);

    rcc_periph_clock_enable(RCC_GPIOB);
    gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO14 | GPIO15);

    gpio_set(GPIOB, GPIO15);


    // IO enable
    gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO5);
    gpio_set(GPIOB, GPIO5);

     // VCC_A enable
    rcc_periph_clock_enable(RCC_GPIOC);
    gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO14);
    gpio_set(GPIOC, GPIO14);

    while (1) {
        gpio_toggle(GPIOA, GPIO8);
        delay(1000000);
        gpio_toggle(GPIOA, GPIO8);
        gpio_toggle(GPIOB, GPIO14);
        delay(1000000);
        gpio_toggle(GPIOB, GPIO14);
        gpio_toggle(GPIOA, GPIO10);
        delay(1000000);
        gpio_toggle(GPIOA, GPIO10);
    }
    return 0;
}
