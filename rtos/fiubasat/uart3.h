#ifndef UART3_H
#define UART3_H

#include "task.h"
#include "uart1.h"
#include "uart2.h"
#include <queue.h>
#include "semphr.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>

extern SemaphoreHandle_t uart3_mutex; // Mutex for UART

// TASKUART3_TRANSMIT
// Tarea que transmite datos a través de UART3
void taskUART3_transmit(void *args __attribute__((unused)));

// TASKUART3_RECEIVE
// Tarea que recibe datos de la cola uart3_rxq, los almacena en el buffer y los envía a través de UART3
void taskUART3_receive(void *args __attribute__((unused)));

// OBS: el encolamiento de datos en la cola de RX se realiza en la interrupción USART3_ISR

// UART3_GET_BUFFER
// Devuelve el buffer de recepción de UART3
uint16_t *UART3_get_buffer(void);

// UART3_PUTS
// Encola un string en uart3_txq, bloqueando la tarea si la cola está llena
uint16_t UART3_puts(const char *s);

// UART3_RECEIVE
// Recibe un byte de datos desde la cola de recepción de UART3, que fue llenada por USART3_ISR
int UART3_receive(void);

// UART3_PUTCHAR
// Encola el dato en uart3_txq, bloqueando la tarea si la cola está llena
void UART3_putchar(uint16_t ch);

// UART3_SETUP
// Configura el periférico USART3
void UART3_setup(void);

bool UART3_buffer_read(uint16_t *data);

void UART3_print_buffer(void);

#endif /* ifndef UART3_H */