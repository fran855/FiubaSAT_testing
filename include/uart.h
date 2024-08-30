#ifndef UART_H
#define UART_H

#include "task.h"
#include <queue.h>
#include "semphr.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>

#define SIZE_BUFFER 256  // Define el tamaño del buffer

// Identificadores para los UARTs
#define UART1 USART1
#define UART2 USART2
#define UART3 USART3

// Configura el periférico USART
void UART_setup(uint32_t usart, uint32_t baudrate);

// Tarea que transmite datos a través de UART1
void taskUART_transmit(uint32_t usart_id);

// Tarea que recibe datos de la cola uart1_rxq, los almacena en el buffer y los envía a través de UART1
void taskUART_receive(uint32_t usart_id);

// OBS: el encolamiento de datos en la cola de RX se realiza en la interrupción USART_ISR

// Recibe un dato desde la cola de recepción de UART, que fue llenada por USART_ISR
int UART_receive(uint32_t usart_id);

// Devuelve el buffer de recepción de UART
uint16_t *UART_get_buffer(uint32_t usart_id);

// Encola un string en uart_txq, bloqueando la tarea si la cola está llena
uint16_t UART_puts(uint32_t usart_id, const char *s);

// Encola el dato en uart_txq, bloqueando la tarea si la cola está llena
void UART_putchar(uint32_t usart_id, uint16_t ch);

// Manejadores de interrupciones de UART
void usart_generic_isr(uint32_t usart_id);

// Lee un dato del buffer de UART
bool UART_buffer_read(uint32_t usart_id, uint16_t *data);

// Imprime el contenido del buffer de UART
void UART_print_buffer(uint32_t usart_id);

#endif