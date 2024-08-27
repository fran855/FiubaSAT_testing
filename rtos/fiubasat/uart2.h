#ifndef UART2_H
#define UART2_H

#include "task.h"
#include "uart1.h"
#include "uart3.h"
#include <queue.h>
#include "semphr.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>

extern SemaphoreHandle_t uart2_mutex; // Mutex for UART

// TASKUART2_TRANSMIT
// Tarea que transmite datos a través de UART2
void taskUART2_transmit(void *args __attribute__((unused)));

// TASKUART2_RECEIVE
// Tarea que recibe datos de la cola uart2_rxq, los almacena en el buffer y los envía a través de UART2
void taskUART2_receive(void *args __attribute__((unused)));

// OBS: el encolamiento de datos en la cola de RX se realiza en la interrupción USART2_ISR

// UART2_GET_BUFFER
// Devuelve el buffer de recepción de UART2
uint16_t *UART2_get_buffer(void);

// UART2_PUTS
// Encola un string en uart2_txq, bloqueando la tarea si la cola está llena
uint16_t UART2_puts(const char *s);

// UART2_RECEIVE
// Recibe un byte de datos desde la cola de recepción de UART2, que fue llenada por USART2_ISR
int UART2_receive(void);

// UART2_PUTCHAR
// Encola el dato en uart2_txq, bloqueando la tarea si la cola está llena
void UART2_putchar(uint16_t ch);

// UART2_SETUP
// Configura el periférico USART2
void UART2_setup(void);

bool UART2_buffer_read(uint16_t *data);

void UART2_print_buffer(void);

#endif /* ifndef UART2_H */