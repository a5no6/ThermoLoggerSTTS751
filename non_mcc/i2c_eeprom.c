#include <stdint.h>
#include <stdio.h>

#include "../mcc_generated_files/i2c_master.h"
#include "../non_mcc/i2c_eeprom.h"

#define EEPROM_DEVICE_CODE  0b1010 // All I2C devices have a control code assigned to them.
unsigned char address_shift_for_chip_select = 16; /* usually 16. set byte address bits for multiple eeprom device use. */

uint8_t make_control_byte(uint32_t  address)
{
    uint8_t upper_3bit = (address >>address_shift_for_chip_select)&0xff;
    upper_3bit &= 0b111;
#ifdef USE_EEPROM_TYPE_1025
        //#if EEPROM_ADDRESS_N_BITS == 17
            if(address_shift_for_chip_select==17){
                upper_3bit &= 0b011;
                if( Address&0x10000)
                    upper_3bit |= 0b100;
                else
                    upper_3bit &= 0b011;
            }else{
                upper_3bit &= 0b111;
            }
#endif
    return((EEPROM_DEVICE_CODE << 3) | (upper_3bit ));
}


enum i2c_eeprom_write_state I2C_EEPROM_WriteNBytes(unsigned long Address,uint8_t *data,size_t len,enum i2c_eeprom_write_state state)
{
    uint8_t address16[2];
    
    switch(state){
        case I2C_EEPROM_WRITE_INIT:
            address16[0] = (Address>>8)&0xff;
            address16[1] = (Address       )&0xff;
            return(I2C_EEPROM_WRITE_OPEN);
        case I2C_EEPROM_WRITE_OPEN:
            if(I2C_Open(make_control_byte(Address)));
                return(I2C_EEPROM_WRITE_ADDRESS_WRITE);
            return(state);
        case I2C_EEPROM_WRITE_ADDRESS_WRITE:
            I2C_SetBuffer(data,len);
            I2C_SetAddressNackCallback(NULL,NULL); //NACK polling?
            I2C_MasterWrite();
            return(I2C_EEPROM_WRITE_CLOSE);
        case I2C_EEPROM_WRITE_CLOSE:
            if(I2C_BUSY == I2C_Close())
                return(I2C_EEPROM_WRITE_CLOSE);
            return(I2C_EEPROM_WRITE_FINISH);
        default:
            break;
    };
            return(I2C_EEPROM_WRITE_FINISH);

}

enum i2c_eeprom_read_state I2C_EEPROM_ReadNBytes(unsigned long Address,uint8_t *data,size_t len,enum i2c_eeprom_read_state state)
{
    uint8_t address16[2];
    
    switch(state){
        case I2C_EEPROM_READ_INIT:
            address16[0] = (Address>>8)&0xff;
            address16[1] = (Address       )&0xff;
            return(I2C_EEPROM_READ_OPEN);
        case I2C_EEPROM_READ_OPEN:
            if(I2C_Open(make_control_byte(Address)));
                return(I2C_EEPROM_READ_ADDRESS_WRITE);
            return(state);
        case I2C_EEPROM_READ_ADDRESS_WRITE:
            I2C_SetBuffer(address16,2);
            I2C_SetAddressNackCallback(NULL,NULL); //NACK polling?
            I2C_MasterWrite();
            return(I2C_EEPROM_READ_CLOSE);
        case I2C_EEPROM_READ_DATA_READ:
            I2C_SetBuffer(data,len);
            I2C_SetAddressNackCallback(NULL,NULL); //NACK polling?
            I2C_MasterWrite();
            return(I2C_EEPROM_READ_CLOSE);
        case I2C_EEPROM_READ_CLOSE:
            if(I2C_BUSY == I2C_Close())
                return(I2C_EEPROM_READ_CLOSE);
            return(I2C_EEPROM_READ_FINISH);
        default:
            break;
    };
            return(I2C_EEPROM_READ_FINISH);

}

typedef struct
{
    size_t len;
    uint8_t *data;
}i2c_buffer_t;

static i2c_operations_t eeprom_read_write_address_CompleteHandler(void *ptr)
{
    I2C_SetBuffer(((i2c_buffer_t *)ptr)->data,((i2c_buffer_t*)ptr)->len);
    I2C_SetDataCompleteCallback(NULL,NULL);
    return I2C_RESTART_READ;
}
