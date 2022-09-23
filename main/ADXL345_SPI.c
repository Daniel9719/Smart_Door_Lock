
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/spi_master.h"
#include "esp_log.h"

#include "ADXL345_SPI.h"

#define PIN_NUM_MISO 19   
#define PIN_NUM_MOSI 23   
#define PIN_NUM_CLK  18   
#define PIN_NUM_CS   5    

typedef union
{
    struct {
        uint16_t HSB : 8;
        uint16_t LSB : 8;
    } Bits;
    uint16_t Data;
} Axe;

static spi_device_handle_t SPI_Handle;
static const char* SPI_TAG = "SPI";

static QueueHandle_t AcelQi_QueHdl;
static QueueHandle_t AcelF_QueHdl;

static void ADXL345_Serial(void *pvParameter){
    uint16_t Acel_Qi;
    float Acel[3];

    while (1)
    {
        xQueueReceive(AcelQi_QueHdl,&Acel_Qi, portMAX_DELAY);
        ESP_LOGI(SPI_TAG, "Data received on Axe X= 0x%4x", Acel_Qi);
        xQueueReceive(AcelQi_QueHdl,&Acel_Qi, portMAX_DELAY);
        ESP_LOGI(SPI_TAG, "Data received on Axe Y= 0x%4x", Acel_Qi);
        xQueueReceive(AcelQi_QueHdl,&Acel_Qi, portMAX_DELAY);
        ESP_LOGI(SPI_TAG, "Data received on Axe Z= 0x%4x", Acel_Qi);
        xQueueReceive(AcelF_QueHdl,&Acel[0], portMAX_DELAY);
        xQueueReceive(AcelF_QueHdl,&Acel[1], portMAX_DELAY);
        xQueueReceive(AcelF_QueHdl,&Acel[2], portMAX_DELAY);
        ESP_LOGI(SPI_TAG, "Data received on Axe X= %f, Axe Y= %f & Axe Z= %f", Acel[0], Acel[1], Acel[2]);
        // vTaskDelay(pdMS_TO_TICKS(1000));
    }
    vTaskDelete(NULL);
}

static void ADXL345_Read(void *pvParameter){
    // QueueHandle_t Acel_QHandle = (QueueHandle_t) pvParameter;
    spi_transaction_t* SPI_Trans_RxData;
    spi_transaction_t SPI_Trans_Config = {
        .flags = SPI_TRANS_USE_RXDATA|SPI_TRANS_USE_TXDATA, // Normal SPI
        .rxlength = 4,
        .length = 8
    };
    struct {
        Axe AxeX;
        Axe AxeY;
        Axe AxeZ;
    } Acel_Qi;
    float Acel[3] = {0};

    while(1){
        // ulTaskNotifyTake(true, portMAX_DELAY);
        SPI_Trans_Config.cmd = READ;
        SPI_Trans_Config.addr = ACE_X_LSB;
        SPI_Trans_Config.tx_data[0] = 0x00;
        // ESP_ERROR_CHECK(spi_device_queue_trans(SPI_Handle, &SPI_Trans_Config, portMAX_DELAY));
        // err = spi_device_transmit(SPI_Handle, &SPI_Trans_Config);
        ESP_ERROR_CHECK(spi_device_transmit(SPI_Handle, &SPI_Trans_Config));
        // ESP_ERROR_CHECK(spi_device_get_trans_result(SPI_Handle, &SPI_Trans_RxData, portMAX_DELAY));
        // printf("Error: %d", err);
        // Acel_Qi.AxeX.Bits.LSB = SPI_Trans_RxData->rx_data[0];
        Acel_Qi.AxeX.Bits.LSB = SPI_Trans_Config.rx_data[0];
        // ESP_LOGI(SPI_TAG, "Data received on Axe X= 0x%2x", Acel_Qi.AxeX.Data);

        SPI_Trans_Config.addr = ACE_X_MSB;
        // // ESP_ERROR_CHECK(spi_device_queue_trans(SPI_Handle, &SPI_Trans_Config, portMAX_DELAY));
        // // ESP_ERROR_CHECK(spi_device_transmit(SPI_Handle, &SPI_Trans_Config));
        ESP_ERROR_CHECK(spi_device_transmit(SPI_Handle, &SPI_Trans_Config));
        // // ESP_ERROR_CHECK(spi_device_get_trans_result(SPI_Handle, &SPI_Trans_RxData, portMAX_DELAY));
        // // Acel_Qi.AxeX.Bits.HSB = SPI_Trans_RxData->rx_data[0];
        Acel_Qi.AxeX.Bits.HSB = SPI_Trans_Config.rx_data[0];

        //Axe Y
        SPI_Trans_Config.addr = ACE_Y_LSB;
        // // ESP_ERROR_CHECK(spi_device_queue_trans(SPI_Handle, &SPI_Trans_Config, portMAX_DELAY));
        // // // ESP_ERROR_CHECK(spi_device_transmit(SPI_Handle, &SPI_Trans_Config));
        // // ESP_ERROR_CHECK(spi_device_get_trans_result(SPI_Handle, &SPI_Trans_RxData, portMAX_DELAY));
        ESP_ERROR_CHECK(spi_device_transmit(SPI_Handle, &SPI_Trans_Config));
        // // Acel_Qi.AxeY.Bits.LSB = SPI_Trans_RxData->rx_data[0];
        Acel_Qi.AxeY.Bits.LSB = SPI_Trans_Config.rx_data[0];
        // ESP_LOGI(SPI_TAG, "Data received on Axe Y= 0x%2x", Acel_Qi.AxeY.Data);

        SPI_Trans_Config.addr = ACE_Y_MSB;
        // ESP_ERROR_CHECK(spi_device_queue_trans(SPI_Handle, &SPI_Trans_Config, portMAX_DELAY));
        ESP_ERROR_CHECK(spi_device_transmit(SPI_Handle, &SPI_Trans_Config));
        // ESP_ERROR_CHECK(spi_device_get_trans_result(SPI_Handle, &SPI_Trans_RxData, portMAX_DELAY));
        // Acel_Qi.AxeY.Bits.HSB = SPI_Trans_RxData->rx_data[0];
        Acel_Qi.AxeY.Bits.HSB = SPI_Trans_Config.rx_data[0];

        //Axe Z
        SPI_Trans_Config.addr = ACE_Z_LSB;
        // ESP_ERROR_CHECK(spi_device_queue_trans(SPI_Handle, &SPI_Trans_Config, portMAX_DELAY));
        ESP_ERROR_CHECK(spi_device_transmit(SPI_Handle, &SPI_Trans_Config));
        // ESP_ERROR_CHECK(spi_device_get_trans_result(SPI_Handle, &SPI_Trans_RxData, portMAX_DELAY));
        // Acel_Qi.AxeZ.Bits.LSB = SPI_Trans_RxData->rx_data[0];
        Acel_Qi.AxeZ.Bits.LSB = SPI_Trans_Config.rx_data[0];

        SPI_Trans_Config.addr = ACE_Z_MSB;
        // ESP_ERROR_CHECK(spi_device_queue_trans(SPI_Handle, &SPI_Trans_Config, portMAX_DELAY));
        ESP_ERROR_CHECK(spi_device_transmit(SPI_Handle, &SPI_Trans_Config));
        // ESP_ERROR_CHECK(spi_device_get_trans_result(SPI_Handle, &SPI_Trans_RxData, portMAX_DELAY));
        // Acel_Qi.AxeZ.Bits.HSB = SPI_Trans_RxData->rx_data[0];
        Acel_Qi.AxeZ.Bits.HSB = SPI_Trans_Config.rx_data[0];

        // ESP_LOGI(SPI_TAG, "Data received on Axe X= 0x%2x, Axe Y= 0x%2x & Axe Z= 0x%2x", Acel_Qi.AxeX.Data, Acel_Qi.AxeY.Data, Acel_Qi.AxeZ.Data);

        if(Acel_Qi.AxeX.Data>=512){
            Acel_Qi.AxeX.Data=-((Acel_Qi.AxeX.Data^0x3FF)+1);       //Conversi�n a Complem 2
        }
        Acel[0]=Acel_Qi.AxeX.Data*0.003906;                    //Factor de escala 3.9 mg/LSB de hoja de datos para +-2g
        if(Acel_Qi.AxeY.Data>=512){
            Acel_Qi.AxeY.Data=-((Acel_Qi.AxeY.Data^0x3FF)+1);       //Conversi�n a Complem 2
        }
        Acel[1]=Acel_Qi.AxeY.Data*0.003906;                    //Factor de escala 3.9 mg/LSB de hoja de datos para +-2g
        if(Acel_Qi.AxeZ.Data>=512){
            Acel_Qi.AxeZ.Data=-((Acel_Qi.AxeZ.Data^0x3FF)+1);       //Conversi�n a Complem 2
        }
        Acel[2]=Acel_Qi.AxeZ.Data*0.003906;                    //Factor de escala 3.9 mg/LSB de hoja de datos para +-2g
        //                                                 //Dato en Q8 Punto fijo
        xQueueSend(AcelQi_QueHdl, &Acel_Qi.AxeX.Data, portMAX_DELAY);
        xQueueSend(AcelQi_QueHdl, &Acel_Qi.AxeY.Data, portMAX_DELAY);
        xQueueSend(AcelQi_QueHdl, &Acel_Qi.AxeZ.Data, portMAX_DELAY);
        xQueueSend(AcelF_QueHdl, &Acel[0], portMAX_DELAY);
        xQueueSend(AcelF_QueHdl, &Acel[1], portMAX_DELAY);
        xQueueSend(AcelF_QueHdl, &Acel[2], portMAX_DELAY);
        // printf("Data received on Axe X= %f, Axe Y= %f & Axe Z= %f", Acel[0], Acel[1], Acel[2]);
    }
    vTaskDelete(NULL);
}

static void ADXL345_Config(spi_device_handle_t SPI_Handle){
    spi_transaction_t SPI_Trans_Config = {
        .flags = SPI_TRANS_USE_TXDATA, 
        .length = 8
        // .rxlength = 0
    };

    // while(1){
        //Measure mode OFF
        SPI_Trans_Config.cmd = WRITE;
        SPI_Trans_Config.addr = ACE_POWER_CTL;
        SPI_Trans_Config.tx_data[0] = 0x00;
        ESP_ERROR_CHECK(spi_device_transmit(SPI_Handle, &SPI_Trans_Config));

        //Baud Rate at 400 [Hz]
        SPI_Trans_Config.addr = ACE_DATA_FORMAT;
        SPI_Trans_Config.tx_data[0] = 0x00;
        ESP_ERROR_CHECK(spi_device_transmit(SPI_Handle, &SPI_Trans_Config));

        //Use of FIFO Stream mode
        SPI_Trans_Config.addr = ACE_BW_RATE;
        SPI_Trans_Config.tx_data[0] = 0x0F;
        ESP_ERROR_CHECK(spi_device_transmit(SPI_Handle, &SPI_Trans_Config));

        //Use of +-2g 10 bits
        SPI_Trans_Config.addr = ACE_FIFO_CTL;
        SPI_Trans_Config.tx_data[0] = 0x80;
        ESP_ERROR_CHECK(spi_device_transmit(SPI_Handle, &SPI_Trans_Config));

        //Measure mode ON
        SPI_Trans_Config.addr = ACE_POWER_CTL;
        SPI_Trans_Config.tx_data[0] = 0x08;
        ESP_ERROR_CHECK(spi_device_transmit(SPI_Handle, &SPI_Trans_Config));
        vTaskDelay(pdMS_TO_TICKS(100));
    // }
}

/*******************************************************************
************************    MAIN PROGRAM    ************************
* @param 
* @return None
*******************************************************************/

void ADXL345_SPI_Config(void){
    spi_bus_config_t SPI_bus_Config = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1
    };
    spi_device_interface_config_t ADXL345_IfConfig = {
        .command_bits = 2,
        .address_bits = 6,
        .mode = 3, // (CPOL, CPHA) (1, 1)
        .clock_speed_hz = 1200000 ,
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 1
    };

    ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &SPI_bus_Config, SPI_DMA_DISABLED));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI3_HOST, &ADXL345_IfConfig, &SPI_Handle));
    ADXL345_Config(SPI_Handle);
    ESP_LOGI(SPI_TAG, "ADXL345 SPI Module configured...");

    AcelQi_QueHdl = xQueueCreate(3,sizeof(Axe));
    AcelF_QueHdl = xQueueCreate(3,sizeof(float));
    
            /*   Func_name        Nickname         StackSize  Params    Priority   Handle */
    xTaskCreate(&ADXL345_Read,   "ADXL345 Read"   ,4096      ,NULL      ,2        ,NULL     );
    xTaskCreate(&ADXL345_Serial, "ADXL345 Serial" ,4096      ,NULL      ,1        ,NULL     );
}
