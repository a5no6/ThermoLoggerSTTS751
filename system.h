/*
 MIT License

Copyright (c) 2020,2021 Konomu Abe

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/


#ifndef SYSTEM_H
#define	SYSTEM_H

#ifdef	__cplusplus
extern "C" {
#endif

#define LOG_DEBUG(x)    {x}
//#define LOG_DEBUG(x)    

#define LOG_INFO(x)    {x}
//#define ASSERT(cond,not_satisfy)    if(!(cond)){not_satisfy}
#define ASSERT(a,msg) if(!(a)){UART_puts("\aASSERT ");UART_puts(msg);UART_puts(" ");UART_puts(__FILE__);UART_puts(":");UART_put_uint16(__LINE__);UART_puts("\n");}
  
#define ENABLE_LOG_PWM
    
#define I2C_FREQ    (100000)
#define UART_BPS    (310*8)
#define _XTAL_FREQ  (1000000)
#define EEPROM_SIZE_BYTE	(0x20000)
#define DEFAULT_LOG_INTERVAL_MINUTES    (10)
    
#define USE_DEBUG_LONG_TYPE
    
        /* UART Logger */
#define BK_UART_TX   1
#define UART_TX_INTERRUPT   1  /* not tested for XC8 */
#define UART_TX_BK_BUF_LEN  (256)
#ifdef __XC8
#define UART_FIFO_DEPTH (0) /* INT assumption is all the bits are transmitted. but pic16 int is TXREG is empty */
#define TX_BUF_FULL  (!TXIF)
#define TRMT    TX1STAbits.TRMT
#define CLEAR_WDT   asm("CLRWDT")
#else
#define CLEAR_WDT  
#define UART_FIFO_DEPTH (8)
#define TXIF    IFS0bits.U1TXIF
#define TXIE    IEC0bits.U1TXIE
#define TXREG   U1TXREG
#define UTXBF    U1STAbits.UTXBF
#define TRMT    U1STAbits.TRMT
#define TX_BUF_FULL  (UTXBF)
#endif

#ifdef ENABLE_LOG_PWM
#define LOG_PWM(x)  x
#else
#define LOG_PWM(x)
#endif

    
//typedef  unsigned char bool;

//#define false (0)
//#define true (1)


//#define ENABLE_MODULE_TEST

    
typedef struct
{
    unsigned i2c_no_nack:1;
} system_error_t;

extern system_error_t system_error;

#define MAX_I2C_NO_NACK_RETRY   (3)
    
#ifdef	__cplusplus
}
#endif

#endif	/* SYSTEM_H */

