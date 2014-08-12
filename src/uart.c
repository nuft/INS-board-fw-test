#include "uart.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>


/* INS board uarts:
 * UART CONN 1: UART1, PB6(TX) PB7(RX)
 * UART CONN 2: UART4, PA0(TX) PA1(RX)
 * UART CONN 3: UART2, PA2(TX) PA3(RX)
 * UART CONN 4: UART6, PC6(TX) PC7(RX)
 * I2C CONN (UART mode): UART3, PB10(TX) PB11(RX)
 */


void uart_conn1_write(const char *str)
{
    while (*str != '\0') {
        usart_send_blocking(USART1, *str);
        str++;
    }
}

void uart_conn1_init(int baud)
{
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_USART1);

    gpio_set_af(GPIOB, GPIO_AF7, GPIO6); // TX
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO6);
    gpio_set_af(GPIOB, GPIO_AF7, GPIO7); // RX
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO7);

    usart_set_baudrate(USART1, baud);
    usart_set_databits(USART1, 8);
    usart_set_stopbits(USART1, USART_STOPBITS_1);
    usart_set_mode(USART1, USART_MODE_TX_RX);
    usart_set_parity(USART1, USART_PARITY_NONE);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
    usart_enable(USART1);
}