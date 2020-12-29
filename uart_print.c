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

#ifndef USE_WITH_MCC_UART
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

void UART_puts(const char *buf)
{
    unsigned i=0;
#ifdef UART_TX_INTERRUPT
        TXIE=0;
#endif
    while(g_uart_tx_bk_len<UART_TX_BK_BUF_LEN && buf[i]){
        g_uart_tx_bk_buf[(g_uart_tx_bk_ind+g_uart_tx_bk_len)%UART_TX_BK_BUF_LEN] = buf[i++];
        g_uart_tx_bk_len++;
    }
#ifdef UART_TX_INTERRUPT
        TXIE=1;
#endif
}
#else

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
    /*
     * Change like below, other wise freeze when buffer gets full.
    void UART1_Write( uint8_t byte)
    {
        while(UART1_IsTxReady() == 0)
        {
            return;
        }
     * 
     */
//    while(UART1_IsTxReady() == false);
//    asm("di");
//    UART1_Write(c);
//    asm("ei");
    EUSART_Write(c);
}

void UART_puts(const char *buf)
{
    int i=0;
    while(buf[i]!=0){
//    _mon_putc(buf[i++]);
        UART_putc(buf[i++]);
    }
}

#endif

unsigned char is_UART_busy(void)
{
#ifdef USE_WITH_MCC_UART
    if(EUSART_is_tx_done())
        return(0);
    else
        return(1);
#else
    if(g_uart_tx_bk_len==0 && !TX_BUF_FULL &&  TRMT)
        return(0);
    else
        return(1);
#endif
}

void UART_flush(void)
{
    while(is_UART_busy()){
#ifdef USE_WITH_MCC_UART
#else
#ifndef UART_TX_INTERRUPT
        UART_TX_BK_TASK();
#endif
        CLEAR_WDT;
#endif
    }
}

int UART_put_uint8(unsigned char d)
{
    const unsigned char unit[2] = {100,10};
    unsigned i=0,disp_digit = 0;
#ifdef UART_TX_INTERRUPT
        TXIE=0;
#endif
#ifndef USE_WITH_MCC_UART
    if(g_uart_tx_bk_len+(2+1) >= UART_TX_BK_BUF_LEN)
        return (0);
#endif
    for(i=0;i<2;i++){
        int c = 0;
        while(d>=unit[i]){
            c++;
            d -= unit[i];
        }
        if(disp_digit!=0 || c!=0 ) {
#ifdef USE_WITH_MCC_UART
            printf("%c", c + '0' );
#else
            g_uart_tx_bk_buf[((g_uart_tx_bk_ind+g_uart_tx_bk_len)%UART_TX_BK_BUF_LEN)] = c + '0';
            g_uart_tx_bk_len++;
#endif
            disp_digit++;
        }
    }
#ifdef USE_WITH_MCC_UART
            printf("%c", (unsigned char)d + '0' );
#else
    g_uart_tx_bk_buf[((g_uart_tx_bk_ind+g_uart_tx_bk_len)%UART_TX_BK_BUF_LEN)] = (unsigned char)d + '0';
    g_uart_tx_bk_len++;
#endif
    disp_digit++;
#ifdef UART_TX_INTERRUPT
    TXIE=1;
#endif
    return(disp_digit);
}

int UART_put_int8(signed char d)
{
#ifdef UART_TX_INTERRUPT
    TXIE=0;
#endif
#ifndef USE_WITH_MCC_UART
    if(g_uart_tx_bk_len+4 >= UART_TX_BK_BUF_LEN)
        return (0);
#endif
    if(d<0){
#ifdef USE_WITH_MCC_UART
            printf("-" );
#else
        g_uart_tx_bk_buf[(g_uart_tx_bk_ind+g_uart_tx_bk_len)%UART_TX_BK_BUF_LEN] =  '-';
        g_uart_tx_bk_len++;
#endif
        d = -d;
        return(UART_put_uint8(d)+1);
    }else{
        return(UART_put_uint8(d));        
    }
}

int UART_put_HEX8(unsigned char d)
{
#ifdef UART_TX_INTERRUPT
    TXIE=0;
#endif
#ifndef USE_WITH_MCC_UART
    if(g_uart_tx_bk_len+2 >= UART_TX_BK_BUF_LEN)
        return (0);
#endif
    unsigned char c;
    c = (d>>4) ;
    c = (c<10) ? (c+'0'):(c+'A'-10);
#ifdef USE_WITH_MCC_UART
            printf("%c",c);
#else
    g_uart_tx_bk_buf[(g_uart_tx_bk_ind+g_uart_tx_bk_len)%UART_TX_BK_BUF_LEN] = c;
    g_uart_tx_bk_len++;
#endif
    c = (d&0xf);
    c = (c<10) ? (c+'0'):(c+'A'-10);
#ifdef USE_WITH_MCC_UART
            printf("%c",c);
#else
    g_uart_tx_bk_buf[(g_uart_tx_bk_ind+g_uart_tx_bk_len)%UART_TX_BK_BUF_LEN] = c;
    g_uart_tx_bk_len++;
#ifdef UART_TX_INTERRUPT
    TXIE=1;
#endif
#endif
    return(2);
}

int UART_put_HEX16(unsigned short d)
{
    int i ;
#ifdef UART_TX_INTERRUPT
    TXIE=0;
#endif
#ifndef USE_WITH_MCC_UART
    if(g_uart_tx_bk_len+2*2 >= UART_TX_BK_BUF_LEN)
        return (0);
#endif
    unsigned char *p = (unsigned char *)&d;
    for(i=0;i<2;i++){
        UART_put_HEX8(p[(2-1)-i]); // Little Endian 
    }
    return(4);
}

int UART_put_uint16(unsigned short d)
{
    const unsigned short unit[4] = {10000,1000,100,10};
    unsigned i=0,disp_digit = 0;
#ifdef UART_TX_INTERRUPT
    TXIE=0;
#endif
#ifndef USE_WITH_MCC_UART
    if(g_uart_tx_bk_len+(4+1) >= UART_TX_BK_BUF_LEN)
        return (0);
#endif
    for(i=0;i<4;i++){
        int c = 0;
        while(d>=unit[i]){
            c++;
            d -= unit[i];
        }
        if(disp_digit!=0 || c!=0 ) {
#ifdef USE_WITH_MCC_UART
            printf("%c",c+'0');
#else
            g_uart_tx_bk_buf[((g_uart_tx_bk_ind+g_uart_tx_bk_len)%UART_TX_BK_BUF_LEN)] = c + '0';
            g_uart_tx_bk_len++;
#endif
            disp_digit++;
        }
    }
#ifdef USE_WITH_MCC_UART
            putchar((unsigned char)d + '0');
#else
    g_uart_tx_bk_buf[((g_uart_tx_bk_ind+g_uart_tx_bk_len)%UART_TX_BK_BUF_LEN)] = (unsigned char)d + '0';
    g_uart_tx_bk_len++;
#endif
    disp_digit++;
#ifdef UART_TX_INTERRUPT
    TXIE=1;
#endif
    return(disp_digit);
}

int UART_put_int16(short d)
{
#ifdef UART_TX_INTERRUPT
    TXIE=0;
#endif
#ifndef USE_WITH_MCC_UART
    if(g_uart_tx_bk_len+6 >= UART_TX_BK_BUF_LEN)
        return (0);
#endif
    if(d<0){
#ifdef USE_WITH_MCC_UART
        printf("-");
#else
        g_uart_tx_bk_buf[(g_uart_tx_bk_ind+g_uart_tx_bk_len)%UART_TX_BK_BUF_LEN] =  '-';
        g_uart_tx_bk_len++;
#endif
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
#ifdef UART_TX_INTERRUPT
    TXIE=0;
#endif
#ifndef USE_WITH_MCC_UART
    if(g_uart_tx_bk_len+2*4 >= UART_TX_BK_BUF_LEN)
        return (0);
#endif
    unsigned char *p = (unsigned char *)&d;
    for(i=0;i<4;i++){
        UART_put_HEX8(p[(4-1)-i]); // Little Endian 
    }
    return(4);
}

int UART_put_uint32(unsigned long d)
{
    const unsigned long unit[9] = {1000000000,100000000,10000000,1000000,100000,10000,1000,100,10};
    unsigned i=0,disp_digit = 0;
#ifdef UART_TX_INTERRUPT
    TXIE=0;
#endif
#ifndef USE_WITH_MCC_UART
    if(g_uart_tx_bk_len+9 >= UART_TX_BK_BUF_LEN)
        return (0);
#endif
    for(i=0;i<9;i++){
        int c = 0;
        while(d>=unit[i]){
            c++;
            d -= unit[i];
        }
        if(disp_digit!=0 || c!=0 ) {
#ifdef USE_WITH_MCC_UART
            printf("%c",c + '0');
#else
            g_uart_tx_bk_buf[((g_uart_tx_bk_ind+g_uart_tx_bk_len)%UART_TX_BK_BUF_LEN)] = c + '0';
            g_uart_tx_bk_len++;
#endif
            disp_digit++;
        }
    }
#ifdef USE_WITH_MCC_UART
            printf("%c",(unsigned char)d + '0');
#else
    g_uart_tx_bk_buf[((g_uart_tx_bk_ind+g_uart_tx_bk_len)%UART_TX_BK_BUF_LEN)] = (unsigned char)d + '0';
    g_uart_tx_bk_len++;
#endif
    disp_digit++;
#ifdef UART_TX_INTERRUPT
    TXIE=1;
#endif
    return(disp_digit);
}

int UART_put_int32(long d)
{
#ifdef UART_TX_INTERRUPT
    TXIE=0;
#endif
#ifndef USE_WITH_MCC_UART
    if(g_uart_tx_bk_len+(9+1) >= UART_TX_BK_BUF_LEN)
        return (0);
#endif
    if(d<0){
#ifdef USE_WITH_MCC_UART
        printf("-");
#else
        g_uart_tx_bk_buf[(g_uart_tx_bk_ind+g_uart_tx_bk_len)%UART_TX_BK_BUF_LEN] =  '-';
        g_uart_tx_bk_len++;
#endif
        d = -d;
        return(UART_put_uint32(d)+1);
    }else{
        return(UART_put_uint32(d));        
    }
}
#endif