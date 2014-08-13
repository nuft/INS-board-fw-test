#include <libopencm3/stm32/can.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include "can.h"
#include "uart.h"
#include "ins-board.h"

void delay(unsigned int n); // in main.c (TODO)

// CAN2 PB13(TX) PB12(RX) PC5(EN low=on,high=standby)

void can_test(void)
{
    uart_conn1_write("can test started\n");

    // enable CAN transceiver
    gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO5);
    gpio_clear(GPIOC, GPIO5);

    rcc_periph_clock_enable(RCC_CAN2);

    gpio_set_af(GPIOB, GPIO_AF9, GPIO12 | GPIO13);
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO12 | GPIO13);

    delay(10000000);

    /* CAN cell init.
     * Setting the bitrate to 1MBit. APB1 = 42MHz,
     * prescaler = 2 -> 21MHz time quanta frequency.
     * 1tq sync + 12tq bit segment1 (TS1) + 8tq bit segment2 (TS2) =
     * 21time quanta per bit period, therefor 21MHz/21 = 1MHz
     */
    if (can_init(CAN2,          // Interface
             false,             // Time triggered communication mode.
             true,              // Automatic bus-off management.
             false,             // Automatic wakeup mode.
             true,              // No automatic retransmission.
             false,             // Receive FIFO locked mode.
             true,              // Transmit FIFO priority.
             CAN_BTR_SJW_1TQ,   // Resynchronization time quanta jump width
             CAN_BTR_TS1_12TQ,  // Time segment 1 time quanta width
             CAN_BTR_TS2_8TQ,   // Time segment 2 time quanta width
             2,                 // Prescaler
             true,              // Loopback
             false)) {          // Silent
        uart_conn1_write("can init failed\n");
        ERROR_LED_ON();
    }

    delay(10000000);

    uint32_t id = 0x2a;
    uint8_t data[] = {0xaa, 0x55, 0xaa, 0x55};
    STATUS_LED_ON();
    if (can_transmit(CAN2, id, false, false, 2, data) != 0) {
        uart_conn1_write("not using mailbox 0, tests will fail\n");
        ERROR_LED_ON();
    }
    while((CAN_TSR(CAN2) & CAN_TSR_RQCP0) == 0);
    STATUS_LED_OFF();

    if ((CAN_TSR(CAN2) & CAN_TSR_TXOK0)) {
        uart_conn1_write("can ok\n");
    } else {
        ERROR_LED_ON();
        uart_conn1_write("can not ok\n");
    }
    if ((CAN_TSR(CAN2) & CAN_TSR_TERR0)) {
        ERROR_LED_ON();
        uart_conn1_write("can transmit error\n");
    }
    if ((CAN_TSR(CAN2) & CAN_TSR_ALST0)) {
        ERROR_LED_ON();
        uart_conn1_write("can arbitration lost\n");
    }
}
