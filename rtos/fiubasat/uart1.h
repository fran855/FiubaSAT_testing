#ifndef UART1_H
#define UART1_H

#include "task.h"
#include "uart2.h"
#include "uart3.h"
#include <queue.h>
#include "semphr.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>

extern SemaphoreHandle_t uart1_mutex; // Mutex for UART

// TASKUART1_TRANSMIT
// Tarea que transmite datos a través de UART1
void taskUART1_transmit(void *args __attribute__((unused)));

// TASKUART1_RECEIVE
// Tarea que recibe datos de la cola uart1_rxq, los almacena en el buffer y los envía a través de UART1
void taskUART1_receive(void *args __attribute__((unused)));

// OBS: el encolamiento de datos en la cola de RX se realiza en la interrupción USART1_ISR

// UART1_GET_BUFFER
// Devuelve el buffer de recepción de UART1
uint16_t *UART1_get_buffer(void);

// UART1_PUTS
// Encola un string en uart1_txq, bloqueando la tarea si la cola está llena
uint16_t UART1_puts(const char *s);

// UART1_RECEIVE
// Recibe un byte de datos desde la cola de recepción de UART1, que fue llenada por USART1_ISR
int UART1_receive(void);

// UART1_PUTCHAR
// Encola el dato en uart1_txq, bloqueando la tarea si la cola está llena
void UART1_putchar(uint16_t ch);

// UART1_SETUP
// Configura el periférico USART1
void UART1_setup(void);

bool UART1_buffer_read(uint16_t *data);

void UART1_print_buffer(void);

#endif /* ifndef UART1_H */