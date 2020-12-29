/* 
 * File:   system.h
 * Author: konomu
 *
 * Created on 2019/04/14, 20:54
 */

#ifndef SYSTEM_H
#define	SYSTEM_H

#ifdef	__cplusplus
extern "C" {
#endif

//#define LOG_DEBUG(x)    {x}
#define LOG_DEBUG(x)    

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

#define false (0)
#define true (1)


//#define ENABLE_MODULE_TEST

#ifdef	__cplusplus
}
#endif

#endif	/* SYSTEM_H */

