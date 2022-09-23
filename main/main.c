#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_log.h"

#include "Connect.h"
#include "MQTT.h"
// #include "PN532_SPI.h"
#include "ADXL345_SPI.h"

TaskHandle_t Main_Task_Handle = NULL;

// EventGroupHandle_t EventGroup_Handle;

static const char* TAG1 = "APP";

void main_Task(void *pvParameter){
    // EventBits_t EventGroupFlags;

    while(1){
        printf("Running main task\n");
        // EventGroupFlags = xEventGroupWaitBits(EventGroup_Handle, LWIP_INIT|DHCP_START|IPV4_ADQUIRED, pdFALSE, pdTRUE, portMAX_DELAY);
        // printf("Event group value: %d\n", EventGroupFlags);
        // if(EventGroupFlags == 0x7){
            ESP_LOGI(TAG1,"Ready to start application code");    //IP_EVENT_STA_GOT_IP
        // }
    }
    vTaskDelete(NULL);
}

void app_main(void){
    // EventGroup_Handle = xEventGroupCreate();

    // (void) WiFi_lwIP_Connection();
    // (void) MQTT_Config();
    (void) ADXL345_SPI_Config();
    // (void) PN532_SPI_Config();
    // xTaskCreate(&main_Task   ,"Main"     ,1024      ,NULL     ,1        ,&Main_Task_Handle);
}