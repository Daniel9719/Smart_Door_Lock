#ifndef PN532_SPI_H_
#define PN532_SPI_H_

#define PREAMBLE        0x0500u
// #define START_CODE      0x00FFu
// #define POSTAMBLE       0x00u
#define MULTIPLE_BIT    0x1u 
#define ACK             0x0000FF00FF00u 
#define NACK            0x0000FFFF0000u 
#define ERROR           0x0000FF01FF7F8100u 

#define SPI_StatusRead          0x02u // Status Reading  = xxxx xx10b
#define SPI_DataWrite           0x01u // Data Write      = xxxx xx01b
#define SPI_DataRead            0x03u // Data Reading    = xxxx xx11b

/*
    COMMANDS
*/

/* Miscellaneous */

#define Diagnose                  0x00u  
#define GetFirmwareVersion        0x02u  
#define GetGeneralStatus          0x04u  
#define ReadRegister              0x06u  
#define WriteRegister             0x08u  
#define ReadGPIO                  0x0Cu  
#define WriteGPIO                 0x0Eu  
#define SetSerialBaudRate         0x10u  
#define SetParameters             0x12u  
#define SAMConfiguration          0x14u  
#define PowerDown                 0x16u  

/* RF Communication */
#define RFConfiguration           0x32u
#define RFRegulationTest          0x58u  

/* Initiator */
#define InJumpForDEP              0x56u
#define InJumpForPSL              0x46u
#define InListPassiveTarget       0x4Au
#define InATR                     0x50u
#define InPSL                     0x4Eu
#define InDataExchange            0x40u
#define InCommmunicateThru        0x42u
#define InDeselect                0x44u
#define InRelease                 0x52u
#define InSelect                  0x54u
#define InAutoPoll                0x60u

void PN532_SPI_Config(void);

#endif