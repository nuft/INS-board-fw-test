#include <libopencm3/stm32/can.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include "can.h"
#include "uart.h"
#include "ins-board.h"
#include <stdio.h>

void delay(unsigned int n); // in main.c (TODO)

// CAN2 PB13(TX) PB12(RX) PC5(EN low=on,high=standby)

void can_monitor(void)
{
    uart_conn1_puts("can monitor started\n");

    // enable CAN transceiver
    gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO5);
    gpio_clear(GPIOC, GPIO5);

    // CAN1 clock must be enabled to use CAN2.
    rcc_periph_clock_enable(RCC_CAN1);
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
             false,             // No automatic retransmission.
             false,             // Receive FIFO locked mode.
             true,              // Transmit FIFO priority.
             CAN_BTR_SJW_1TQ,   // Resynchronization time quanta jump width
             CAN_BTR_TS1_12TQ,  // Time segment 1 time quanta width
             CAN_BTR_TS2_8TQ,   // Time segment 2 time quanta width
             2,                 // Prescaler
             true,              // Loopback
             false)) {          // Silent
        uart_conn1_puts("can init failed\n");
        ERROR_LED_ON();
    }

    // All filter configuration must be done with CAN1!
    CAN_FMR(CAN1) &= ~(int32_t)0x3F00; // assign all filters to CAN2
    can_filter_id_mask_32bit_init(
        CAN1,
        0,      // nr
        0,      // id
        0,      // mask
        0,      // fifo
        true    // enable
    ); // match any id

    while (1) {
        STATUS_LED_ON();
        // wait for packet in fifio
        while((CAN_RF0R(CAN2) & CAN_RF0R_FMP0_MASK) == 0);
        STATUS_LED_OFF();

        uint32_t id, fid;
        uint8_t data[9];
        uint8_t len;
        bool ext, rtr;

        int i;
        for (i = 0; i < 8; i++) {
            data[i] = 0;
        }

        // read packet
        can_receive(
            CAN2,   // canport
            0,      // fifo
            true,   // release
            &id,    // id
            &ext,   // extended id
            &rtr,   // transmission request
            &fid,   // filter id
            &len,   // length
            data
        );

        data[8] = '\0';

        uint32_t dl = data[4]<<0 | data[5]<<8 | data[6]<<16 | data[7]<<24;
        uint32_t dh = data[0]<<0 | data[1]<<8 | data[2]<<16 | data[3]<<24;

        printf("id: %u, packet: %8x%8x, len %u, %s\n", id, dh, dl, len, data);
    }
}

void can_test(void)
{
    uart_conn1_puts("can test started\n");

    // enable CAN transceiver
    gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO5);
    gpio_clear(GPIOC, GPIO5);

    // CAN1 clock must be enabled to use CAN2.
    rcc_periph_clock_enable(RCC_CAN1);
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
             false,             // No automatic retransmission.
             false,             // Receive FIFO locked mode.
             true,              // Transmit FIFO priority.
             CAN_BTR_SJW_1TQ,   // Resynchronization time quanta jump width
             CAN_BTR_TS1_12TQ,  // Time segment 1 time quanta width
             CAN_BTR_TS2_8TQ,   // Time segment 2 time quanta width
             2,                 // Prescaler
             true,              // Loopback
             false)) {          // Silent
        uart_conn1_puts("can init failed\n");
        ERROR_LED_ON();
    }

    // All filter configuration must be done with CAN1!
    CAN_FMR(CAN1) &= ~(int32_t)0x3F00; // assign all filters to CAN2
    can_filter_id_mask_16bit_init(CAN1, 0, 0, 0, 0, 0, 0, true); // match any id

    // CAN has a maximum of 8 chars payload... this can't be a coincidence
    uint8_t data[] = {'S', 'i', 'e', 'g', 'H', 'e', 'i', 'l'};
    uint32_t id = 0x00;
    STATUS_LED_ON();
    if (can_transmit(CAN2, id, false, false, sizeof(data), data) != 0) {
        uart_conn1_puts("not using mailbox 0, tests will fail\n");
        ERROR_LED_ON();
    }
    while((CAN_TSR(CAN2) & CAN_TSR_RQCP0) == 0);
    STATUS_LED_OFF();

    if ((CAN_TSR(CAN2) & CAN_TSR_TXOK0)) {
        uart_conn1_puts("can ok\n");
    } else {
        ERROR_LED_ON();
        uart_conn1_puts("can not ok\n");
    }
    if ((CAN_TSR(CAN2) & CAN_TSR_TERR0)) {
        ERROR_LED_ON();
        uart_conn1_puts("can transmit error\n");
    }
    if ((CAN_TSR(CAN2) & CAN_TSR_ALST0)) {
        ERROR_LED_ON();
        uart_conn1_puts("can arbitration lost\n");
    }
    if (CAN_RF0R(CAN2) & CAN_RF0R_FMP0_MASK) {
        uart_conn1_puts("can message received! :)\n");
        uint32_t id, fmi;
        bool ext, rtr;
        uint8_t length;
        uint8_t data[9];
        can_receive(CAN2, 0, true, &id, &ext, &rtr, &fmi, &length, data);
        data[length] = '\0';
        uart_conn1_puts("received: \"");
        uart_conn1_puts((char*)data);
        uart_conn1_puts("\"\n");
    } else {
        uart_conn1_puts("nothing received :(\n");
    }
}
