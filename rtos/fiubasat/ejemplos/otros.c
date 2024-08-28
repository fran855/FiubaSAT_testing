//static TimerHandle_t auto_reload_timer;
//static void taskPeriodic(void *pvParameters);
//static void autoReloadCallback(TimerHandle_t xTimer);

/*
static void taskPeriodic(void *pvParameters) {
    TaskHandle_t xHandle = (TaskHandle_t) pvParameters;
    int i = 0;
    for (;;) {
        if (i == 0) {
            vTaskResume(xHandle);
            i = 1;
        } else {
            vTaskSuspend(xHandle);
            i = 0;
        }
        if(xSemaphoreTake(uart1_mutex, portMAX_DELAY) == pdTRUE) {
            UART1_puts("taskPeriodic is running\r\n");
            UART2_puts("Transmiting through UART2\r\n");
            xSemaphoreGive(uart1_mutex);
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
*/

/*
static void autoReloadCallback(TimerHandle_t xTimer) {
    char *autoReloadMessage = "Timer expired, auto-reloading\r\n";
    if(xSemaphoreTake(uart1_mutex, portMAX_DELAY) == pdTRUE) {
        UART1_puts(autoReloadMessage);
        xSemaphoreGive(uart1_mutex);
    }

    char message[50];
    int expire_time = xTimerGetExpiryTime(xTimer) - xTaskGetTickCount();
    sprintf(message, "Timer will expire again in %d ms\r\n", expire_time);
    
    if(xSemaphoreTake(uart1_mutex, portMAX_DELAY) == pdTRUE) {
        UART1_puts(message);
        xSemaphoreGive(uart1_mutex);
    }
}
*/