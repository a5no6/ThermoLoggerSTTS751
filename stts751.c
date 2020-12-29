#include <xc.h> // include standard header file
#include <string.h>
#include <stdio.h>
#include "system.h"
#include "stts751.h"
#include "uart_print.h"
#include "mcc_generated_files/examples/i2c_master_example.h"


void STTS751_read_regsiter(unsigned char reg,unsigned short *d)
{
    *d = I2C_Read1ByteRegister(STTS751_I2C_ADDRESS,reg);
}

#ifdef ENABLE_MODULE_TEST
void STTS751_test_sync(void)
{
	unsigned short hi, lo;
	while (1) {
		UART_puts("status ");UART_flush();
		UART_put_HEX16(STTS751_read_regsiter_sync(0x01));UART_flush();
		UART_puts("\n");UART_flush();

//		UART_puts("temp Hi8 %04x\n",hi=STTS751_read_regsiter_sync(0x00));UART_flush();
		UART_puts("temp Hi8  ");UART_flush();
		UART_put_HEX16(hi=STTS751_read_regsiter_sync(0x00));UART_flush();
		UART_puts("\n");UART_flush();



//		UART_puts("temp Lo8 %04x\n",lo=STTS751_read_regsiter_sync(0x02));UART_flush();
		UART_puts("temp Lo8  ");UART_flush();
		UART_put_HEX16(hi=STTS751_read_regsiter_sync(0x02));UART_flush();
		UART_puts("\n");UART_flush();

		short s1 = (short )((hi&0xff) * 256 + (lo&0xff));
        long s2 = s1;
        s2 = (s2 * 1000)/256;
		UART_put_HEX16(s1);UART_flush();
		UART_put_uint32(s2/1000);UART_flush();
		UART_put_uint32(s2%1000);UART_flush();
		UART_puts("\n");UART_flush();
	}
}

void STTS751_test(void)
{
	unsigned short hi, lo   ,status;
	while (1) {
//        i2c_state = 1;while(i2c_state)STTS751_read_regsiter(STTS751_REGISTER_ADDRESS_STATUS,&status);
//        i2c_state = 1;while(i2c_state)STTS751_read_regsiter(STTS751_REGISTER_ADDRESS_LO,&lo);
//        i2c_state = 1;while(i2c_state)STTS751_read_regsiter(STTS751_REGISTER_ADDRESS_HI,&hi);
        
//		UART_puts("status %04x\n", status);UART_flush();
//		UART_puts("temp Hi8 %04x\n",hi);UART_flush();
//		UART_puts("temp Lo8 %04x\n",lo);UART_flush();
		short s1 = (short )((hi&0xff) * 256 + (lo&0xff));
        long s2 = s1;
        s2 = (s2 * 1000)/256;
//		UART_puts("%04x %ld.%ld\n", s1,s2/1000,(s2%1000));UART_flush();
		UART_puts("\n");UART_flush();
	}
}

#endif //ENABLE_MODULE_TEST


