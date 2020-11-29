/* 
 * File:   uart_print.h
 * Author: konomu
 *
 * Created on 2019/04/14, 19:13
 */

#ifndef UART_PRINT_H
#define	UART_PRINT_H

#ifdef	__cplusplus
extern "C" {
#endif

void UART_puts(const char *buf);
void UART_init(unsigned long clk,unsigned short bps);
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

#ifdef	__cplusplus
}
#endif

#endif	/* UART_PRINT_H */

