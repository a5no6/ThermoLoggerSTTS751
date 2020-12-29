/**
  Generated Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This is the main file generated using PIC10 / PIC12 / PIC16 / PIC18 MCUs

  Description:
    This header file provides implementations for driver APIs for all modules selected in the GUI.
    Generation Information :
        Product Revision  :  PIC10 / PIC12 / PIC16 / PIC18 MCUs - 1.81.5
        Device            :  PIC16F1705
        Driver Version    :  2.00
*/

/*
    (c) 2018 Microchip Technology Inc. and its subsidiaries. 
    
    Subject to your compliance with these terms, you may use Microchip software and any 
    derivatives exclusively with Microchip products. It is your responsibility to comply with third party 
    license terms applicable to your use of third party software (including open source software) that 
    may accompany Microchip software.
    
    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER 
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY 
    IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS 
    FOR A PARTICULAR PURPOSE.
    
    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP 
    HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO 
    THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL 
    CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT 
    OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS 
    SOFTWARE.
*/

#include "mcc_generated_files/mcc.h"
//#include "non_mcc/i2c_eeprom.h"
#include "uart_print.h"

/*
                         Main application
 */
//void I2C_ReadNBytes(i2c_address_t address, uint8_t *data, size_t len);
void I2C_EEPROM_ReadDataBlock(unsigned long mem_address, uint8_t *data, size_t len);
void I2C_EEPROM_WriteDataBlock(unsigned long mem_address, uint8_t *data, size_t len);

const unsigned char test_string[64] = "Hello EEPROM 512.\n";
unsigned char buf[64];
void logger_main(void);

void main(void)
{
    // initialize the device
    SYSTEM_Initialize();
    TXIE = 0;

    // When using interrupts, you need to set the Global and Peripheral Interrupt Enable bits
    // Use the following macros to:

    // Enable the Global Interrupts
    INTERRUPT_GlobalInterruptEnable();

    // Enable the Peripheral Interrupts
    INTERRUPT_PeripheralInterruptEnable();

    // Disable the Global Interrupts
    //INTERRUPT_GlobalInterruptDisable();

    // Disable the Peripheral Interrupts
    //INTERRUPT_PeripheralInterruptDisable();


//   enum i2c_eeprom_write_state  state = I2C_EEPROM_WRITE_INIT;
LATCbits.LATC3 = 1;
__delay_ms(100);
    UART_puts("Hello.\n");
//   while(state!=I2C_EEPROM_WRITE_FINISH)
    UART_puts("s1\n");
        I2C_EEPROM_WriteDataBlock(0,test_string ,32);
    UART_puts("s2\n");
       I2C_EEPROM_ReadDataBlock(0,buf ,32);
    UART_puts("s3\n");
        
    UART_puts(buf);
            
    logger_main();

    while (1)
    {
//    UART_puts("Hello.\n");
        // Add your application code
    }
}
/**
 End of File
*/