#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/event_groups.h"

#include "esp_wifi.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "Connect.h"

static const char* WIFI_TAG = "WIFI";
static const char* IP_TAG = "IP";

static EventGroupHandle_t EventGroup_Handle;

static void WiFi_Event_Handler(void* event_handler_arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data){
    static uint16_t First_connect_trials = 0;
    static uint16_t Second_connect_trials = 0;

    switch (event_id){
    case WIFI_EVENT_STA_START:
        ESP_LOGI(WIFI_TAG,"LwIP network initialized\n");
        xEventGroupSetBits(EventGroup_Handle,LWIP_INIT);
        ESP_ERROR_CHECK(esp_wifi_connect());  //WIFI_EVENT_STA_CONNECTED
        break;
    case WIFI_EVENT_STA_CONNECTED:
        ESP_LOGI(WIFI_TAG,"DHCP client started and begginning DCHP process\n");
        xEventGroupSetBits(EventGroup_Handle,DHCP_START);
        break;
    case IP_EVENT_STA_GOT_IP:
        ESP_LOGI(IP_TAG,"Got IPV4 address from DHCP server \n");
        xEventGroupSetBits(EventGroup_Handle,IPV4_ADQUIRED);
        break;
    case WIFI_EVENT_STA_DISCONNECTED: 
        ESP_LOGI(WIFI_TAG,"Wi-Fi driver fails to set up a connection with the AP due to certain reasons \n");
        if(First_connect_trials<10){
            ESP_ERROR_CHECK(esp_wifi_connect());
            First_connect_trials++;
            ESP_LOGI(WIFI_TAG,"Retry No. %d to connect to DHCP server\n",First_connect_trials);
        }
        else{
            ESP_LOGI(WIFI_TAG,"Connection not established, retrying after 10 sec\n");
            vTaskDelay(pdMS_TO_TICKS(10000));
            First_connect_trials = 0;
            Second_connect_trials++;
            if(Second_connect_trials>4){
                ESP_LOGI(WIFI_TAG,"Connection never established, sending mail to owner...\n");
                // TODO: Implementation SMTP or something that notifies lost of connection, maybe buffer or BLE
            }
        }
        break;
    default:
        break;
    }
}

void WiFi_lwIP_Connection(void){
    EventBits_t EventGroupFlags;
    
    EventGroup_Handle = xEventGroupCreate();

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());                  //Create LwIP core task and initialize LwIP-related work
    ESP_ERROR_CHECK(esp_event_loop_create_default());   //Create Event task and initialize application event’s callback function
    esp_netif_create_default_wifi_sta();                //Create default network interface instance binding station with TCP/IP stack.
    wifi_init_config_t WiFi_Init_Config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&WiFi_Init_Config));  //Create the Wi-Fi driver task and initialize the Wi-Fi driver.

    //Registering different events to handler function
    esp_event_handler_instance_register(WIFI_EVENT,WIFI_EVENT_STA_START,&WiFi_Event_Handler,NULL,NULL);
    esp_event_handler_instance_register(WIFI_EVENT,WIFI_EVENT_STA_CONNECTED,&WiFi_Event_Handler,NULL,NULL);
    esp_event_handler_instance_register(WIFI_EVENT,WIFI_EVENT_STA_DISCONNECTED,&WiFi_Event_Handler,NULL,NULL);
    esp_event_handler_instance_register(IP_EVENT,IP_EVENT_STA_GOT_IP,&WiFi_Event_Handler,NULL,NULL);

    wifi_config_t WiFi_Config = {
        .sta = {
            .ssid = CONFIG_WIFI_SSID,
            .password = CONFIG_WIFI_PASSWORD
        }
    };

    esp_wifi_set_mode(WIFI_MODE_STA);                   //Configure the Wi-Fi mode as Station
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA,&WiFi_Config));
    ESP_ERROR_CHECK(esp_wifi_start());  //Start the Wi-Fi driver.
                                        //WIFI_EVENT_STA_START
    printf("Starting wifi\n");
    EventGroupFlags = xEventGroupWaitBits(EventGroup_Handle, LWIP_INIT|DHCP_START|IPV4_ADQUIRED, pdFALSE, pdTRUE, portMAX_DELAY);
}



