#ifndef UART_H
#define UART_H

#include <stdint.h>
#include <stdlib.h>

uint32_t uart_conn_write(uint32_t conn_nb, const char *buf, size_t len);

void uart_conn1_puts(const char *str);
void uart_conn1_init(int baud);

#endif // UART_H
