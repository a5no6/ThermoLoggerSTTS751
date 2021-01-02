/* 
 * File:   uart_print.h
 * Author: konomu
 *
 * Created on 2019/04/14, 19:13
 * Last Modified 2019/05/04
 */

#define USE_WITH_MCC_UART
#include <xc.h>
#include <stdbool.h>
#include <stdint.h>
#ifdef USE_WITH_MCC_UART
#include <stdio.h>
#include "mcc_generated_files/eusart.h"
#else
#include "system.h"
#endif
#include "uart_print.h"

#define USE_DEBUG_LONG_TYPE

#ifdef USE_WITH_MCC_UART
void UART_init(unsigned long clk,unsigned long bps)
{
#ifdef    __XC8
    unsigned long dwBaud;
    unsigned char txie = TXIE;
    TXIE = 0;
    while(!EUSART_is_tx_done());
    SYNC = 0;
    BRGH = 1;
    BRG16 = 1;
    dwBaud = ((clk/4) / bps) - 1;
    SPBRG = (unsigned char) dwBaud;
    SPBRGH = (unsigned char)((unsigned short) (dwBaud >> 8));
    TXEN = 1;
    SPEN = 1;
    TXIE = txie;
#else
    U1BRG = (((clk/16) / bps) - 1);
    U1STAbits.UTXISEL = 0b01;//01 = Interrupt is generated and asserted when all characters have been transmitted
    U1STAbits.UTXEN = 1;
    U1MODEbits.ON = 1;
#endif
}

void UART_TX_BK_TASK(void)
{
        EUSART_TxDefaultInterruptHandler();
}

void UART_putc(const char c)
{
    EUSART_Write(c);
}

bool UART_buf_is_available(unsigned short n)
{
    if(eusartTxBufferRemaining>n)
        return (true);
    else
        return(false);
}

unsigned char is_UART_busy(void)
{
    if(EUSART_is_tx_done())
        return(0);
    else
        return(1);
}

void UART_flush(void)
{
    while(is_UART_busy());
}

#else
volatile char g_uart_tx_bk_buf[UART_TX_BK_BUF_LEN];
volatile unsigned short g_uart_tx_bk_len = 0;
volatile unsigned short g_uart_tx_bk_ind = 0;

void UART_TX_BK_TASK(void)
{
    if(!TX_BUF_FULL && g_uart_tx_bk_len>0){
        TXREG = g_uart_tx_bk_buf[g_uart_tx_bk_ind];
        g_uart_tx_bk_ind = (g_uart_tx_bk_ind+1)%UART_TX_BK_BUF_LEN;
        g_uart_tx_bk_len--;
    }
}

#ifdef UART_TX_INTERRUPT
void UART_TX_Interrupt_Handler(void)
{
    int i;
    if(TXIE==0||TXIF==0) return;
    for(i=0;i<(UART_FIFO_DEPTH+1) &&  g_uart_tx_bk_len>0;i++){ // FIFO + Shift register
        TXREG = g_uart_tx_bk_buf[g_uart_tx_bk_ind];
        g_uart_tx_bk_ind = (g_uart_tx_bk_ind+1)%UART_TX_BK_BUF_LEN;
        g_uart_tx_bk_len--;
    }
    if(g_uart_tx_bk_len==0) TXIE=0;
    TXIF = 0;
}
#endif

/*  
 32MM needs no port setting
 * PIC16,32MX needs port setting,(ANSEL,TRIS,PPS)
 */
void UART_init(unsigned long clk,unsigned long bps)
{
#ifdef    __XC8
    unsigned long dwBaud;
    SYNC = 0;
    BRGH = 1;
    BRG16 = 1;
    dwBaud = ((clk/4) / bps) - 1;
    SPBRG = (unsigned char) dwBaud;
    SPBRGH = (unsigned char)((unsigned short) (dwBaud >> 8));
    TXEN = 1;
    SPEN = 1;
#else
    U1BRG = (((clk/16) / bps) - 1);
    U1STAbits.UTXISEL = 0b01;//01 = Interrupt is generated and asserted when all characters have been transmitted
    U1STAbits.UTXEN = 1;
    U1MODEbits.ON = 1;
#endif
}

void UART_putc(const unsigned char c)
{
#ifdef UART_TX_INTERRUPT
        TXIE=0;
#endif
    if(g_uart_tx_bk_len<UART_TX_BK_BUF_LEN ){
        g_uart_tx_bk_buf[(g_uart_tx_bk_ind+g_uart_tx_bk_len)%UART_TX_BK_BUF_LEN] = c;
        g_uart_tx_bk_len++;
    }
#ifdef UART_TX_INTERRUPT
        TXIE=1;
#endif
}

bool UART_buf_is_available(unsigned short n)
{
    if(g_uart_tx_bk_len+n < UART_TX_BK_BUF_LEN)
        return (true);
    else
        return(false);
}

unsigned char is_UART_busy(void)
{
    if(g_uart_tx_bk_len==0 && !TX_BUF_FULL &&  TRMT)
        return(0);
    else
        return(1);
}

void UART_flush(void)
{
    while(is_UART_busy()){
#ifndef UART_TX_INTERRUPT
        UART_TX_BK_TASK();
#endif
        CLEAR_WDT;
    }
}
#endif

void UART_puts(const char *buf)
{
    int i=0;
    while(buf[i]!=0){
        UART_putc(buf[i++]);
    }
}

int UART_put_uint8(unsigned char d)
{
    const unsigned char unit[2] = {100,10};
    unsigned i=0,disp_digit = 0;
    if(!UART_buf_is_available(3))
        return(0);
    for(i=0;i<2;i++){
        int c = 0;
        while(d>=unit[i]){
            c++;
            d -= unit[i];
        }
        if(disp_digit!=0 || c!=0 ) {
            UART_putc( c + '0');
            disp_digit++;
        }
    }
    UART_putc((unsigned char)d + '0' );
    disp_digit++;
    return(disp_digit);
}

int UART_put_int8(signed char d)
{
    if(!UART_buf_is_available(4))
        return(0);
    if(d<0){
            UART_putc('-' );
        d = -d;
        return(UART_put_uint8(d)+1);
    }else{
        return(UART_put_uint8(d));        
    }
}

int UART_put_HEX8(unsigned char d)
{
    unsigned char c;
    if(!UART_buf_is_available(2))
        return(0);
    c = (d>>4) ;
    c = (c<10) ? (c+'0'):(c+'A'-10);
    UART_putc(c);
    c = (d&0xf);
    c = (c<10) ? (c+'0'):(c+'A'-10);
    UART_putc(c);
    return(2);
}

int UART_put_HEX16(unsigned short d)
{
    int i ;
    unsigned char *p = (unsigned char *)&d;
    if(!UART_buf_is_available(4))
        return(0);
    for(i=0;i<2;i++){
        UART_put_HEX8(p[(2-1)-i]); // Little Endian 
    }
    return(4);
}

int UART_put_uint16(unsigned short d)
{
    const unsigned short unit[4] = {10000,1000,100,10};
    unsigned i=0,disp_digit = 0;
    if(!UART_buf_is_available(4+1))
        return(0);
    for(i=0;i<4;i++){
        int c = 0;
        while(d>=unit[i]){
            c++;
            d -= unit[i];
        }
        if(disp_digit!=0 || c!=0 ) {
            UART_putc(c+'0');
            disp_digit++;
        }
    }
    UART_putc((unsigned char)d + '0');
    disp_digit++;
    return(disp_digit);
}

int UART_put_int16(short d)
{
    if(!UART_buf_is_available(6))
        return(0);
    if(d<0){
        UART_putc('-');
        d = -d;
        return(UART_put_uint16(d)+1);
    }else{
        return(UART_put_uint16(d));        
    }
}

#ifdef USE_DEBUG_LONG_TYPE

int UART_put_HEX32(unsigned long d)
{
    int i ;
    unsigned char *p = (unsigned char *)&d;
    if(!UART_buf_is_available(2*4))
        return(0);
    for(i=0;i<4;i++){
        UART_put_HEX8(p[(4-1)-i]); // Little Endian 
    }
    return(8);
}

int UART_put_uint32(unsigned long d)
{
    const unsigned long unit[9] = {1000000000,100000000,10000000,1000000,100000,10000,1000,100,10};
    unsigned i=0,disp_digit = 0;
    if(!UART_buf_is_available(9))
        return(0);
    for(i=0;i<9;i++){
        int c = 0;
        while(d>=unit[i]){
            c++;
            d -= unit[i];
        }
        if(disp_digit!=0 || c!=0 ) {
            UART_putc(c + '0');
            disp_digit++;
        }
    }
    UART_putc((unsigned char)d + '0');
    disp_digit++;
    return(disp_digit);
}

int UART_put_int32(long d)
{
    if(!UART_buf_is_available(9+1))
        return(0);
    if(d<0){
        UART_putc('-');
        d = -d;
        return(UART_put_uint32(d)+1);
    }else{
        return(UART_put_uint32(d));        
    }
}
#endif