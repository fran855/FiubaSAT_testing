#include "FreeRTOS.h"
#include "uart.h"
#include <stdint.h>
#include <stdbool.h>
#include "stdio.h"

typedef struct {
    uint32_t usart;  // Identificador del USART
    uint16_t buffer[SIZE_BUFFER];  // Buffer de datos
    uint16_t head;  // Índice de escritura
    uint16_t tail;  // Índice de lectura
    bool buffer_full;  // Indica si el buffer está lleno
    QueueHandle_t txq;  // Cola de transmisión
    QueueHandle_t rxq;  // Cola de recepción
    SemaphoreHandle_t mutex;  // Mutex para protección de acceso
    int interrupciones;  // Contador de interrupciones
} uart_t;

// Definición de estructuras UART
uart_t uart1;
uart_t uart2;
uart_t uart3;

static void uart_init(uart_t *uart, uint32_t usart);
static void UART_process_data(uint32_t usart_id, uint16_t data);
static void buffer_write(uint32_t usart_id, uint16_t data);

void UART_setup(uint32_t usart, uint32_t baudrate) {
    // Configuración del reloj y pines según el USART
    if (usart == USART1) {
        // Habilitar el clock para GPIOA (donde están conectados los pines TX y RX de UART1)
        rcc_periph_clock_enable(RCC_GPIOA); 
        // Habilitar el clock para USART1
        rcc_periph_clock_enable(RCC_USART1); 

        // Configurar los pines de UART1 (PA9 TX y PA10 RX)
        // gpio_set_mode(puerto GPIO afectado, input/output y velocidad de cambio, modo de salida, pin afectado)
        gpio_set_mode(GPIO_BANK_USART1_TX, 
            GPIO_MODE_OUTPUT_50_MHZ,
            GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, 
            GPIO_USART1_TX);

        gpio_set_mode(GPIO_BANK_USART1_RX, 
            GPIO_MODE_INPUT, 
            GPIO_CNF_INPUT_FLOAT, 
            GPIO_USART1_RX);
        
        // Habilitar la interrupción de UART1 en el NVIC (a nivel sistema para que el controlador de interrupciones pueda manejarla)
        nvic_enable_irq(NVIC_USART1_IRQ);
        uart_init(&uart1, USART1);

    } else if (usart == USART2) {
        // Habilitar el clock para GPIOA (donde están conectados los pines TX y RX de UART2)
        rcc_periph_clock_enable(RCC_GPIOA);
        // Habilitar el clock para USART2
        rcc_periph_clock_enable(RCC_USART2);

        // Configurar los pines de UART2 (PA2 TX y PA3 RX)
        // gpio_set_mode(puerto GPIO afectado, input/output y velocidad de cambio, modo de salida, pin afectado)
        gpio_set_mode(GPIO_BANK_USART2_TX, 
            GPIO_MODE_OUTPUT_50_MHZ, 
            GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, 
            GPIO_USART2_TX);

        gpio_set_mode(GPIO_BANK_USART2_RX, 
            GPIO_MODE_INPUT, 
            GPIO_CNF_INPUT_FLOAT, 
            GPIO_USART2_RX);

        // Habilitar la interrupción de UART2 en el NVIC (a nivel sistema para que el controlador de interrupciones pueda manejarla)
        nvic_enable_irq(NVIC_USART2_IRQ);
        uart_init(&uart2, USART2);

    } else if (usart == USART3) {
        // Habilitar el clock para GPIOA (donde están conectados los pines TX y RX de UART3)
        rcc_periph_clock_enable(RCC_GPIOB);
        // Habilitar el clock para USART3
        rcc_periph_clock_enable(RCC_USART3);

        // Configurar los pines de UART1 (PB10 TX y PB11 RX)
        // gpio_set_mode(puerto GPIO afectado, input/output y velocidad de cambio, modo de salida, pin afectado)
        gpio_set_mode(GPIO_BANK_USART3_TX, 
            GPIO_MODE_OUTPUT_50_MHZ, 
            GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, 
            GPIO_USART3_TX);

        gpio_set_mode(GPIO_BANK_USART3_RX, 
            GPIO_MODE_INPUT, 
            GPIO_CNF_INPUT_FLOAT, 
            GPIO_USART3_RX);
        
        // Habilitar la interrupción de UART3 en el NVIC (a nivel sistema para que el controlador de interrupciones pueda manejarla)
        nvic_enable_irq(NVIC_USART3_IRQ);
        uart_init(&uart3, USART3);
    }

    // Configuración de USART
    usart_set_baudrate(usart, baudrate);
    usart_set_databits(usart, 8);
    usart_set_stopbits(usart, USART_STOPBITS_1);
    usart_set_mode(usart, USART_MODE_TX_RX);
    usart_set_parity(usart, USART_PARITY_NONE);
    usart_set_flow_control(usart, USART_FLOWCONTROL_NONE);

    usart_enable(usart);
    // Dentro de las fuentes de interrupción de UART, habilitar la interrupción de recepción
    usart_enable_rx_interrupt(usart);
}

static void uart_init(uart_t *uart, uint32_t usart) {
    uart->usart = usart;  // Asigna el USART correspondiente
    uart->head = 0;
    uart->tail = 0;
    uart->buffer_full = false;
    uart->txq = xQueueCreate(SIZE_BUFFER, sizeof(uint16_t));
    uart->rxq = xQueueCreate(SIZE_BUFFER, sizeof(uint16_t));
    uart->mutex = xSemaphoreCreateBinary();
    uart->interrupciones = 0;
}

// Manejadores de UARTs
static uart_t *get_uart(uint32_t usart_id) {
    switch (usart_id) {
        case UART1: return &uart1;
        case UART2: return &uart2;
        case UART3: return &uart3;
        default: return NULL;
    }
}

void taskUART_transmit(uint32_t usart_id) {
    uart_t *uart = get_uart(usart_id);
    if (uart == NULL) return;

    uint16_t ch;
    for (;;) {
        // Recibir datos de la cola de transmisión
        while (xQueueReceive(uart->txq, &ch, pdMS_TO_TICKS(500)) == pdPASS) {
            // Esperar hasta que el registro de transmisión esté vacío
            while (!usart_get_flag(uart->usart, USART_SR_TXE))
                taskYIELD(); // Ceder la CPU hasta que esté listo

            // Enviar el byte a través de USART
            usart_send_blocking(uart->usart, ch);
        }
        // Esperar 50 ms antes de la siguiente iteración
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void taskUART_receive(uint32_t usart_id) {
    uart_t *uart = get_uart(usart_id);
    if (uart == NULL) return;

    int data;
    for(;;) {
        while((data = UART_receive(usart_id)) != -1) {
            UART_process_data(usart_id, data);
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

int UART_receive(uint32_t usart_id) {
    uart_t *uart = get_uart(usart_id);
    if (uart == NULL) return -1;

    int data;
    // Intenta recibir un dato de la cola uart1_rxq. Si no hay datos, se bloquea durante 500 ms; si hay, lo devuelve
    if (xQueueReceive(uart->rxq, &data, pdMS_TO_TICKS(500)) == pdPASS) {
        return data;
    }
    return -1;
}

// UART_PROCESS_DATA
// Envia un byte de datos a través de UART y lo almacena en el buffer
static void UART_process_data(uint32_t usart_id, uint16_t data) {
    uart_t *uart = get_uart(usart_id);
    if (uart == NULL) return;

    buffer_write(usart_id, data);
}

uint16_t *UART_get_buffer(uint32_t usart_id) {
    uart_t *uart = get_uart(usart_id);
    if (uart == NULL) return NULL;

    return uart->buffer;
}

uint16_t UART_puts(uint32_t usart_id, const char *s) {
    uart_t *uart = get_uart(usart_id);
    if (uart == NULL) return 0;

    uint16_t nsent = 0;
    // Recorre el string s hasta encontrar el caracter nulo
    for ( ; *s; s++) {
        // Añade el caracter a la cola uart1_txq. portMAX_DELAY indica que la tarea se bloqueará indefinidamente si la cola está llena
        if(xQueueSend(uart->txq, s, portMAX_DELAY) != pdTRUE) {
            // Si falla, se resetea la cola y se devuelve la cantidad de caracteres enviados
            xQueueReset(uart->txq);
            return nsent; // Queue full
        }
        nsent++;
    }
    return nsent;
}

void UART_putchar(uint32_t usart_id, uint16_t ch) {
    uart_t *uart = get_uart(usart_id);
    if (uart == NULL) return;

    xQueueSend(uart->txq, &ch, portMAX_DELAY);
}

void usart1_isr(void) {
    usart_generic_isr(USART1);
}

void usart2_isr(void) {
    usart_generic_isr(USART2);
}

void usart3_isr(void) {
    usart_generic_isr(USART3);
}

void usart_generic_isr(uint32_t usart_id) {
    uart_t *uart = get_uart(usart_id);
    if (uart == NULL) return;

    uart->interrupciones++;

    // flag USART_SR_RXNE: Receive Data Register Not Empty
    while (usart_get_flag(uart->usart, USART_SR_RXNE)) {
        // Leer el byte de datos recibido del registro de datos del USART correspondiente
        uint16_t data = usart_recv_blocking(uart->usart);
        // Añade el byte de datos a la cola de recepción desde la rutina de interrupción, sin prioridad de interrupción (NULL)
        if (xQueueSendToBackFromISR(uart->rxq, &data, NULL) != pdTRUE) { 
            // Si falla, se resetea la cola
            xQueueReset(uart->rxq);
        }
    }
}

bool UART_buffer_read(uint32_t usart_id, uint16_t *data) {
    uart_t *uart = get_uart(usart_id);
    if (uart == NULL) return false;

    if (uart->head == uart->tail && !uart->buffer_full) {
        // Buffer vacío
        return false;
    }

    *data = uart->buffer[uart->tail];
    uart->tail = (uart->tail + 1) % SIZE_BUFFER;
    uart->buffer_full = false;  // Después de leer, el buffer ya no puede estar lleno
    return true;
}

static void buffer_write(uint32_t usart_id, uint16_t data) {
    uart_t *uart = get_uart(usart_id);
    if (uart == NULL) return;

    if (uart->buffer_full) {
        // Opcional: manejar el caso de buffer lleno, como sobrescribir el dato más antiguo.
        uart->tail = (uart->tail + 1) % SIZE_BUFFER;
    }

    uart->buffer[uart->head] = data;  // Escribe el dato en la posición de head
    uart->head = (uart->head + 1) % SIZE_BUFFER;

    // Verifica si el buffer está lleno
    if (uart->head == uart->tail) {
        uart->buffer_full = true;
    } else {
        uart->buffer_full = false;
    }
}

void UART_print_buffer(uint32_t usart_id) {
    uart_t *uart = get_uart(usart_id);
    if (uart == NULL) return;

    uint16_t i = uart->tail;
    UART_puts(USART3, "Contenido del buffer:\r\n");

    // Si el buffer no está lleno, imprimir desde tail hasta head
    while (i != uart->head || (i == uart->head && uart->buffer_full)) {
        UART_putchar(USART3, uart->buffer[i]);
        i = (i + 1) % SIZE_BUFFER;

        // Si el buffer estaba lleno, necesitamos asegurarnos de que
        // no imprima dos veces al dar una vuelta completa
        if (uart->buffer_full && i == uart->head) {
            uart->buffer_full = false; // Resetear flag de buffer lleno
            break;
        }
    }
    UART_putchar(USART3, '\r');
    UART_putchar(USART3, '\n');

    UART_puts(USART3, "Interrupciones: ");
    // Convertir el número de interrupciones a una cadena
    char buffer[10];  // Asegúrate de que el buffer sea lo suficientemente grande
    snprintf(buffer, sizeof(buffer), "%u", uart->interrupciones);
    // Enviar la cadena a través del UART
    UART_puts(USART3, buffer);

    UART_putchar(USART3, '\r');
    UART_putchar(USART3, '\n');
}