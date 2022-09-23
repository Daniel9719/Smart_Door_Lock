#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
// #include "freertos/event_groups.h"

#include "mqtt_client.h"
#include "esp_log.h"

#define SHIFTR_ID "ESP32"
#define SHIFTR_USERNAME "danielortiz"
#define SHIFTR_PASSWORD "Test_MQTT"

static const char* TAG = "MQTT";
static TaskHandle_t MQTT_App_Handle = NULL;
static esp_mqtt_client_handle_t MQTT_Client; 

static void MQTT_Event_Handler(void* event_handler_arg, esp_event_base_t event_base,
                             int32_t event_id, void* event_data){
    esp_mqtt_event_handle_t event = event_data;
    // esp_mqtt_client_handle_t client = event->client;
    // MQTT_Callback(event_data);
    ESP_LOGI("TERMINAL", "Event dispatched from event loop base=%s, event_id=%d", event_base, event_id);
    // int msg_id;
    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            xTaskNotifyGive(MQTT_App_Handle);
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
            break;
        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "MQTT_EVENT_DATA");
            printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
            printf("DATA=%.*s\r\n", event->data_len, event->data);
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
            break;
        default:
            ESP_LOGI(TAG, "Other event id:%d", event->event_id);
            break;
    }
}

static void MQTT_App(void *pvParameter){
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    int msg_id = esp_mqtt_client_publish(MQTT_Client, "hola/yo", "9", 0, 1, 0);
    ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
    msg_id = esp_mqtt_client_subscribe(MQTT_Client, "hola/yo", 0);
    ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
    while(1){
        vTaskDelay(pdMS_TO_TICKS(10000));
        ESP_LOGI(TAG,"Inside task");
    };
    vTaskDelete(NULL);
}

void MQTT_Config(void){
    esp_mqtt_client_config_t MQTT_Config = {
        .client_id = SHIFTR_ID,
        .username = SHIFTR_USERNAME,
        .password = SHIFTR_PASSWORD,
        .uri = "mqtt://danielortiz.cloud.shiftr.io"
        // .disable_clean_session = false  //Enable Persisten session
        // .keep_alive = 60,
    };
    MQTT_Client = esp_mqtt_client_init(&MQTT_Config);
    esp_mqtt_client_register_event(MQTT_Client, ESP_EVENT_ANY_ID, &MQTT_Event_Handler, MQTT_Client);
    esp_mqtt_client_start(MQTT_Client);

              /* Func_name     Nickname   StackSize   Params   Priority   Handle */
    xTaskCreate(&MQTT_App   ,"MQTT_App"  ,2048      ,NULL     ,1        ,&MQTT_App_Handle);
}
