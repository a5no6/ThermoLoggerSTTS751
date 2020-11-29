/* 
 * File:   uart_print.h
 * Author: konomu
 *
 * Created on 2019/04/14, 19:13
 */

#if 0
/********* Example of system(_config).h ******/
//#define USE_WITH_MCC_UART
/*
  if do not use USE_WITH_MCC_UART Make sure to have 
 * if(xxIE&&xxIF) xx_handler(); 
 * if(TXIE&&TXIF) UART_TX_Interrupt_Handler(); 
 * in PIC16/18 family
 */
#define LOG_DEBUG(x)    {x}
//#define LOG_DEBUG(x)    

#define LOG_INFO(x)    {x}
//#define ASSERT(cond,not_satisfy)    if(!(cond)){not_satisfy}
#define ASSERT(a,msg) if(!(a)){UART_puts("\aASSERT ");UART_puts(msg);UART_puts(" ");UART_puts(__FILE__);UART_puts(":");UART_put_uint16(__LINE__);UART_puts("\n");}
  
//#define ENABLE_LOG_PWM
#define USE_DEBUG_LONG_TYPE

#ifndef USE_WITH_MCC_UART
#define UART_BPS    (38400)
#define _XTAL_FREQ  (16000000)

//#define BK_UART_TX   1
#define UART_TX_INTERRUPT   1  
#define UART_TX_BK_BUF_LEN  (256)
#ifdef __XC8
#define UART_FIFO_DEPTH (0) /* INT assumption is all the bits are transmitted. but pic16 int is TXREG is empty */
#define TX_BUF_FULL  (!TXIF)
#define TRMT    TRMT
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
#endif
#endif

#ifndef UART_PRINT_H
#define	UART_PRINT_H

#ifdef	__cplusplus
extern "C" {
#endif

void UART_puts(const char *buf);
void UART_init(unsigned long clk,unsigned long bps);
void UART_TX_BK_TASK(void);
unsigned char is_UART_busy(void);
void UART_flush(void);
int UART_put_uint8(unsigned char d);
int UART_put_int8(signed char d);
int UART_put_HEX8(unsigned char d);
int UART_put_HEX16(unsigned short d);
int UART_put_uint16(unsigned short d);
int UART_put_int16(short d);

#ifdef USE_DEBUG_LONG_TYPE
int UART_put_HEX32(unsigned long d);
int UART_put_uint32(unsigned long d);
int UART_put_int32(long d);
#endif

#ifdef UART_TX_INTERRUPT
void UART_TX_Interrupt_Handler(void);
#endif

//#define _PL()	{UART_puts(__FILE__);UART_puts(":");UART_put_uint16(__LINE__);UART_puts("\n");UART_flush();}
//#define _PL()	UART_put_uint16(__LINE__);UART_puts("\n");UART_flush();
#define _PL()	UART_puts("L:");UART_put_uint16(__LINE__);UART_puts("\n");



#ifdef	__cplusplus
}
#endif

#endif	/* UART_PRINT_H */
