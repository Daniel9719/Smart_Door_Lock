
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/spi_master.h"
#include "esp_log.h"

#include "PN532_SPI.h"

#define PIN_NUM_MISO 19   
#define PIN_NUM_MOSI 23   
#define PIN_NUM_CLK  18   
#define PIN_NUM_CS   5    

/* GetFirmwareVersion
GetGeneralStatus
SAMConfiguration
InListPassiveTarget

InDataExchange
InDeselect
InRelease
InSelect
 */

typedef struct 
{
    union
    {
        uint32_t Data;
        struct
        {
            uint32_t Preamble :28;  // 0x0F 0x00 0x00 0x00 0x00 0x00 0x00 
            uint16_t START_CODE1 : 8; // 0x00
        } Byte;
    } Sync;

    union
    {
        uint32_t Data;
        struct 
        {
            uint16_t START_CODE2 : 8;  //0xFF
            uint16_t LEN : 8;
            uint16_t LCS : 8;  // LCS = uint8 (0x100 - [LEN])
            uint16_t TFI : 8;  // 0xD4 host -> PN532   or   0xD5 PN532 -> host
        } Byte;
    } Start;

    union 
    {
        uint16_t Data;
        struct 
        {
            uint16_t DCS : 8;   //  DCS = uint8 (0x100 - [TFI + PD0 + PD1 + … + PDn])
            uint16_t POSTAMBLE : 8;  // 0x00
        } Byte;
    } End;

    uint8_t (*LCS_Cal) (uint8_t);
    uint8_t (*DCS_Cal) (uint8_t, uint8_t*, uint8_t);
} PN532_RegFrame;

static spi_device_handle_t SPI_Handle;
static const char* SPI_TAG = "SPI";

static uint8_t LCS_Cal (uint8_t LEN){
    return (uint8_t) (0x100u - LEN);
}

static uint8_t DCS_Cal (uint8_t TFI, uint8_t* Data, uint8_t DataSize){
    uint8_t Ret =0;
    for(int i=0; i<DataSize;i++){
        Ret += Data[i];
    }
    Ret += TFI;
    return (uint8_t) (0x100u - Ret);
}

static void PN532_Read(void *pvParameter){
    spi_transaction_t* SPI_Trans_RxData;
    spi_transaction_t SPI_Trans_Config = {
        .flags = SPI_TRANS_USE_RXDATA, // Normal SPI
        .length = 8,
        .rxlength = 8
    };
    PN532_RegFrame Frame ={
        .Sync.Byte = {
            .Preamble = 0x0F000000000000u,
            .START_CODE1 = 0x00u
        },
        .Start.Byte = {
            .START_CODE2 = 0xFFu,
            .TFI = 0xD4u
        },
        .End.Byte.POSTAMBLE = 0x00u
    };
    esp_err_t err;

    while(1){
        // ulTaskNotifyTake(true, portMAX_DELAY);
        // SPI_Trans_Config.cmd = READ|MULTIPLE_BIT;
        // SPI_Trans_Config.addr = ACE_X_LSB;
        // ESP_ERROR_CHECK(spi_device_queue_trans(SPI_Handle, &SPI_Trans_Config, portMAX_DELAY));
        // err = spi_device_transmit(SPI_Handle, &SPI_Trans_Config);
        // ESP_ERROR_CHECK(spi_device_transmit(SPI_Handle, &SPI_Trans_Config));
        // ESP_ERROR_CHECK(spi_device_get_trans_result(SPI_Handle, &SPI_Trans_RxData, portMAX_DELAY));
        // printf("Error: %d", err);
        // Acel_Qi.AxeX.Bits.LSB = SPI_Trans_RxData->rx_data[0];
        // Acel_Qi.AxeX.Bits.LSB = SPI_Trans_Config.rx_data[0];
        // ESP_LOGI(SPI_TAG, "Data received on Axe X= 0x%2x", Acel_Qi.AxeX.Data);

        // SPI_Trans_Config.cmd = READ|MULTIPLE_BIT;
        // SPI_Trans_Config.addr = ACE_X_MSB;
        // ESP_ERROR_CHECK(spi_device_queue_trans(SPI_Handle, &SPI_Trans_Config, portMAX_DELAY));
        // // ESP_ERROR_CHECK(spi_device_transmit(SPI_Handle, &SPI_Trans_Config));
        // ESP_ERROR_CHECK(spi_device_get_trans_result(SPI_Handle, &SPI_Trans_RxData, portMAX_DELAY));
        // Acel_Qi.AxeX.Bits.HSB = SPI_Trans_RxData->rx_data[0];

        // //Axe Y
        // SPI_Trans_Config.cmd = READ|MULTIPLE_BIT;
        // SPI_Trans_Config.addr = ACE_Y_LSB;
        // ESP_ERROR_CHECK(spi_device_queue_trans(SPI_Handle, &SPI_Trans_Config, portMAX_DELAY));
        // // ESP_ERROR_CHECK(spi_device_transmit(SPI_Handle, &SPI_Trans_Config));
        // ESP_ERROR_CHECK(spi_device_get_trans_result(SPI_Handle, &SPI_Trans_RxData, portMAX_DELAY));
        // Acel_Qi.AxeY.Bits.LSB = SPI_Trans_RxData->rx_data[0];

        // SPI_Trans_Config.cmd = READ|MULTIPLE_BIT;
        // SPI_Trans_Config.addr = ACE_Y_MSB;
        // ESP_ERROR_CHECK(spi_device_queue_trans(SPI_Handle, &SPI_Trans_Config, portMAX_DELAY));
        // // ESP_ERROR_CHECK(spi_device_transmit(SPI_Handle, &SPI_Trans_Config));
        // ESP_ERROR_CHECK(spi_device_get_trans_result(SPI_Handle, &SPI_Trans_RxData, portMAX_DELAY));
        // Acel_Qi.AxeY.Bits.HSB = SPI_Trans_RxData->rx_data[0];

        // //Axe Z
        // SPI_Trans_Config.addr = ACE_Z_LSB;
        // ESP_ERROR_CHECK(spi_device_queue_trans(SPI_Handle, &SPI_Trans_Config, portMAX_DELAY));
        // // ESP_ERROR_CHECK(spi_device_transmit(SPI_Handle, &SPI_Trans_Config));
        // ESP_ERROR_CHECK(spi_device_get_trans_result(SPI_Handle, &SPI_Trans_RxData, portMAX_DELAY));
        // Acel_Qi.AxeZ.Bits.LSB = SPI_Trans_RxData->rx_data[0];

        // SPI_Trans_Config.cmd = READ|MULTIPLE_BIT;
        // SPI_Trans_Config.addr = ACE_Z_MSB;
        // ESP_ERROR_CHECK(spi_device_queue_trans(SPI_Handle, &SPI_Trans_Config, portMAX_DELAY));
        // // ESP_ERROR_CHECK(spi_device_transmit(SPI_Handle, &SPI_Trans_Config));
        // ESP_ERROR_CHECK(spi_device_get_trans_result(SPI_Handle, &SPI_Trans_RxData, portMAX_DELAY));
        // Acel_Qi.AxeZ.Bits.HSB = SPI_Trans_RxData->rx_data[0];

        // ESP_LOGI(SPI_TAG, "Data received on Axe X= 0x%2x, Axe Y= 0x%2x & Axe Z= 0x%2x", Acel_Qi.AxeX.Data, Acel_Qi.AxeY.Data, Acel_Qi.AxeZ.Data);

        // if(Acel_Qi.AxeX.Data>=512){
        //     Acel_Qi.AxeX.Data=-((Acel_Qi.AxeX.Data^0x3FF)+1);       //Conversi�n a Complem 2
        // }
        // Acel[0]=Acel_Qi.AxeX.Data*0.003906;                    //Factor de escala 3.9 mg/LSB de hoja de datos para +-2g
        // if(Acel_Qi.AxeY.Data>=512){
        //     Acel_Qi.AxeY.Data=-((Acel_Qi.AxeY.Data^0x3FF)+1);       //Conversi�n a Complem 2
        // }
        // Acel[1]=Acel_Qi.AxeY.Data*0.003906;                    //Factor de escala 3.9 mg/LSB de hoja de datos para +-2g
        // if(Acel_Qi.AxeZ.Data>=512){
        //     Acel_Qi.AxeZ.Data=-((Acel_Qi.AxeZ.Data^0x3FF)+1);       //Conversi�n a Complem 2
        // }
        // Acel[2]=Acel_Qi.AxeZ.Data*0.003906;                    //Factor de escala 3.9 mg/LSB de hoja de datos para +-2g
        //                                                 //Dato en Q8 Punto fijo

        // ESP_LOGI(SPI_TAG, "Data received on Axe X= %f, Axe Y= %f & Axe Z= %f", Acel[0], Acel[1], Acel[2]);
        // vTaskDelay(pdMS_TO_TICKS(1000));
    }
    vTaskDelete(NULL);
}

static void PN532_Config(spi_device_handle_t SPI_Handle){
    spi_transaction_t SPI_Trans_Config = {
        .flags = SPI_TRANS_USE_TXDATA, 
        .length = 8,
        .rxlength = 0
    };
    esp_err_t err;
    // PN532_RegFrame Frame ={
    //     .Start.Byte.START_CODE = 0x00FF,
    //     .End.Byte.POSTAMBLE = 0x00
    // };

    // while(1){
        // SPI_Trans_Config.cmd = PREAMBLE;
        // Frame.Start.Byte.LEN = 0x3;
        // Frame.Start.Byte.LCS = (uint8_t)(0x100 - Frame.Start.Byte.LEN);
        // SPI_Trans_Config.addr = (Frame.Start.Data<<8) | SAMConfiguration;
        // SPI_Trans_Config.tx_data[0] = 0x00;
        // // ESP_ERROR_CHECK(spi_device_queue_trans(SPI_Handle, &SPI_Trans_Config, portMAX_DELAY));
        // // ESP_ERROR_CHECK(spi_device_transmit(SPI_Handle, &SPI_Trans_Config));
        // err = spi_device_transmit(SPI_Handle, &SPI_Trans_Config);
        // printf("Error: %d", err);

        // SPI_Trans_Config.addr = ACE_DATA_FORMAT;
        // SPI_Trans_Config.tx_data[0] = 0x00;
        // // ESP_ERROR_CHECK(spi_device_queue_trans(SPI_Handle, &SPI_Trans_Config, portMAX_DELAY));
        // // ESP_ERROR_CHECK(spi_device_transmit(SPI_Handle, &SPI_Trans_Config));
        // err = spi_device_transmit(SPI_Handle, &SPI_Trans_Config);
        // printf("Error: %d", err);

        // // vTaskDelay(pdMS_TO_TICKS(10));
        // SPI_Trans_Config.addr = ACE_BW_RATE;
        // SPI_Trans_Config.tx_data[0] = 0x0F;
        // // ESP_ERROR_CHECK(spi_device_queue_trans(SPI_Handle, &SPI_Trans_Config, portMAX_DELAY));
        // // ESP_ERROR_CHECK(spi_device_transmit(SPI_Handle, &SPI_Trans_Config));
        // err = spi_device_transmit(SPI_Handle, &SPI_Trans_Config);
        // printf("Error: %d", err);

        // SPI_Trans_Config.addr = ACE_FIFO_CTL;
        // SPI_Trans_Config.tx_data[0] = 0x80;
        // // ESP_ERROR_CHECK(spi_device_queue_trans(SPI_Handle, &SPI_Trans_Config, portMAX_DELAY));
        // // ESP_ERROR_CHECK(spi_device_transmit(SPI_Handle, &SPI_Trans_Config));
        // err = spi_device_transmit(SPI_Handle, &SPI_Trans_Config);
        // printf("Error: %d", err);

        // SPI_Trans_Config.addr = ACE_POWER_CTL;
        // SPI_Trans_Config.tx_data[0] = 0x08;
        // // ESP_ERROR_CHECK(spi_device_queue_trans(SPI_Handle, &SPI_Trans_Config, portMAX_DELAY));
        // // ESP_ERROR_CHECK(spi_device_transmit(SPI_Handle, &SPI_Trans_Config));
        // err = spi_device_transmit(SPI_Handle, &SPI_Trans_Config);
        // printf("Error: %d", err);
        printf("Sending SPI frame\n");

    // }
}

/*******************************************************************
************************    MAIN PROGRAM    ************************
* @param 
* @return None
*******************************************************************/

void PN532_SPI_Config(void){
    spi_bus_config_t SPI_bus_Config = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK
    };
    spi_device_interface_config_t PN532_IfConfig = {
        .command_bits = 8,
        .address_bits = 40,
        .mode = 3, // (CPOL, CPHA) (1, 1)
        .clock_speed_hz = 3000000 ,
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 8
    };

    ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &SPI_bus_Config, SPI_DMA_DISABLED));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI3_HOST, &PN532_IfConfig, &SPI_Handle));
    ESP_LOGI(SPI_TAG, "PN532 SPI Module configured...");
    PN532_Config(SPI_Handle);
    
            /*   Func_name      Nickname       StackSize  Params    Priority   Handle */
    xTaskCreate(&PN532_Read, "PN532 Read" ,4096      ,NULL      ,1        ,NULL     );
}
