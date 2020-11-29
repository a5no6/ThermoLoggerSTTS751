/*
 * File:   newmain.c
 * Author: konomu
 *
 * Created on 2018/05/20, 15:04
 */


#include <xc.h>
#include <string.h>
#include <stdio.h>
#include "system.h"
//#include "xc8_i2c.h"
//#include "xc8_i2c_eeprom.h"
#include "uart_print.h"
#include "stts751.h"

int i2c_state;


#define EEPROM_HEADER_BLOCK_SIZE    (16)
#define TIME_INTERVAL_ADDRESS   (14)

#define UART_DEBUG_LEVEL    3
#define PIN_CHECK  (LATCbits.LATC4)

#define PIN_POWER_ON_DEMAND (LATCbits.LATC3)
#define INPUT_FILTER_THRESH (10)

#define WDTPS_VALUE_WAKE    (0b10010)
#define WDTPS_VALUE_SLEEP   (0b01100)
#define WDT_COUNT_10MINUTES  ((int)((60*DEFAULT_LOG_INTERVAL_MINUTES)/4.35+0.5))

typedef struct {
    unsigned prev :1;
    unsigned output :1;
    unsigned count:6;
} _io_input_filter;

typedef enum {
    dummy,
    init,
    sleep,
    wait_for_external_use_to_be_removed,
    wait_10ms,
    read_st751_status,
    read_st751_lo,
    read_st751_hi,
    wait_eeprom_write,
    prepare_for_external,
    abnormal,
    halt
} state_t;

typedef enum {
    LF31kHz=0,
    HF1MHz
} cpu_clock_t;


typedef struct {
    bool is_eeprom_initialized ;
    bool is_waked_by_ra3 ;
    bool st751_need_power;
    bool i2c_need_power;
    bool external_need_power;
    bool initialize_request;
    state_t mainstate;
    state_t prev_state;
    unsigned short wdt_wake_count;
    unsigned short in_state_wake_count;
} info_t;

void input_filter(_io_input_filter *f,unsigned char input);
void supply_power_on_demand(int on);
void STTS751_read_regsiter(unsigned char regsiter,unsigned short *d);
void update_power_on_demand(info_t* s);

_io_input_filter g_external_request_input;
volatile info_t g_info = {false,false,false,false,false,false,init,dummy,0,0};

//unsigned short stts_log_interval = DEFAULT_LOG_INTERVAL_MINUTES*(60/2); /* (60s/2s)*10min */
unsigned short stts_log_interval = 0; /* (60s/2s)*10min */

void disable_i2c(void){
    SSPCONbits.SSPEN=0;  
}

void enable_i2c(void)
{
    SSPCONbits.SSPEN=1;         // enable MSSP port    
}

void sleep_after_uart_send(void)
{
    while(is_UART_busy()){
        UART_TX_BK_TASK();
    }
    WDTCONbits.WDTPS = WDTPS_VALUE_SLEEP;
    asm("sleep;");
    asm("nop");
    asm("nop");
    asm("CLRWDT");
    WDTCONbits.WDTPS = WDTPS_VALUE_WAKE;
}

void set_pwm_state_monitor_clock(cpu_clock_t clk)
{
    switch(clk)
    {
        case LF31kHz: // 31kHz = 31000/40= 775hz(1.3ms))
            T2CONbits.T2CKPS = 0b00;
            break;
        case HF1MHz: // 1MHz = 1000/64/10 = 1562Hz(640us))
            T2CONbits.T2CKPS = 0b00;
//            T2CONbits.T2CKPS = 0b10;
            break;
    }
}

void switch_clock_to(cpu_clock_t clk)
{
    const unsigned char isr_val[] = {0b0000,0b1011};
    ASSERT(clk<2,"clk wrong value");
    if(OSCCONbits.IRCF ==isr_val[clk])
        return;

    SSPCONbits.SSPEN=0;         // disable MSSP port    
    PWM3CONbits.PWM3EN = 0;
    OSCCONbits.IRCF = isr_val[clk];
    set_pwm_state_monitor_clock(clk);
    switch(clk)
    {
        case LF31kHz: // 31kHz = 31000/40= 775hz(1.3ms))
            UART_init(31000,UART_BPS);
            break;
        case HF1MHz: // 1MHz = 1000/64/10 = 1562Hz(640us))
            UART_init(_XTAL_FREQ,UART_BPS);
            break;
        default :
            break;
    }
    SSPCONbits.SSPEN=1;         // enable MSSP port    
#ifdef ENABLE_LOG_PWM
    PWM3CONbits.PWM3EN = 1;
#endif
}


bool read_log_interval(void)
{
    unsigned char d;
    for(i2c_state=1,eerom_validation=0;i2c_state&& eerom_validation<0xffff;eerom_validation++) 
        Read_EEPROM_DATA_Sequential(TIME_INTERVAL_ADDRESS,&d,1);
    LOG_DEBUG(UART_puts(__FILE__);UART_puts(":");UART_put_uint16(__LINE__);UART_puts(" validation:");UART_put_HEX16(eerom_validation);UART_puts("\n");)
    if(eerom_validation>0x1000)
        return(false);        
//    i2c_state = 1; while(i2c_state>0) Read_EEPROM_DATA_Sequential(TIME_INTERVAL_ADDRESS,&d,1);
    LOG_DEBUG(UART_puts("d=");UART_put_uint8(d);UART_puts("\n");UART_flush(););
    if(d>0){
        stts_log_interval =(WDT_COUNT_10MINUTES*(unsigned short)d)/DEFAULT_LOG_INTERVAL_MINUTES;
        LOG_DEBUG(UART_puts("stts_log_interval ");UART_put_uint16(stts_log_interval);UART_puts("\n");UART_flush(););
    }else{
        stts_log_interval = WDT_COUNT_10MINUTES;        
    }
    return(true);
}

void state_machine(info_t* s)
{
    unsigned short d;
    short temperature;
    long s2;
    switch(s->mainstate){
        case init:
            T1CONbits.TMR1ON = 0;
            PIR1bits.TMR1IF = 0;
            TMR1 = 0; 
            g_info.st751_need_power = false;
            g_info.i2c_need_power = false;
            s->is_eeprom_initialized = false;
            s->mainstate = wait_for_external_use_to_be_removed;
            if(PORTAbits.RA3==0){
                s->external_need_power = true;
            }else{
                if(stts_log_interval==0){
                    supply_power_on_demand(1);
                    switch_clock_to(HF1MHz);
                    if(!read_log_interval()){
                        s->mainstate = abnormal;
                        break;                        
                    } 
                }
            }
            disable_i2c();
            switch_clock_to(LF31kHz);
            break;
        case wait_for_external_use_to_be_removed:
            if(PORTAbits.RA3==1){
                LOG_DEBUG(UART_puts("initialize eeprom cache.\n");UART_flush(););
                if(g_info.is_eeprom_initialized == false){
                    switch_clock_to(HF1MHz);
                    supply_power_on_demand(1);
                    if(!load_eeprom_data(EEPROM_HEADER_BLOCK_SIZE)){
                        s->mainstate = abnormal;
                        break;
                    }
                    if(g_eeprom_cache.eeprom_address==EEPROM_HEADER_BLOCK_SIZE){
                        g_info.wdt_wake_count = 0;
                        if(!read_log_interval()){
                            s->mainstate = abnormal;
                            break;                        
                        } 
                    }
                    update_power_on_demand(&g_info);
                }
                switch_clock_to(LF31kHz);
                s->mainstate = sleep ;
                enable_i2c();
            }else{
                LOG_DEBUG(UART_puts("going sleep.\n");UART_flush(););
                sleep_after_uart_send();
                LOG_DEBUG(UART_puts("woke up.\n");UART_flush(););
                if(g_info.wdt_wake_count<0xffff)
                    g_info.wdt_wake_count++;        
            }
            break;
        case sleep:
            LOG_DEBUG(UART_puts("going sleep.\n");UART_flush(););
            sleep_after_uart_send();
            if(s->is_waked_by_ra3){ 
                LOG_DEBUG(UART_puts("woken up with ra3.\n");UART_flush(););
                s->is_waked_by_ra3 = false;            
            }else{ // waked by WDT
                if(g_info.wdt_wake_count<0xffff)
                    g_info.wdt_wake_count++;        
                LOG_DEBUG(UART_puts("woken up by wdt. cnt = ");UART_flush(););
                LOG_DEBUG(UART_put_uint16(s->wdt_wake_count);UART_flush(););
                LOG_DEBUG(UART_puts("\n");UART_flush(););
                if(s->wdt_wake_count>=stts_log_interval){
                    s->wdt_wake_count -= stts_log_interval;
                    g_info.st751_need_power = true;
                    update_power_on_demand(&g_info);
                    T1CONbits.TMR1ON = 1;
                    s->mainstate = wait_10ms;
                }
            }
            break;
        case wait_10ms:
            if(PIR1bits.TMR1IF || TMR1 > (31*10)){
                T1CONbits.TMR1ON = 0;
                PIR1bits.TMR1IF = 0;
                TMR1 = 0; 
                switch_clock_to(HF1MHz);
               i2c_state = 1;
                s->mainstate = read_st751_status;                
            }
            break;

        case read_st751_status:
            if(i2c_state){
                STTS751_read_regsiter(STTS751_REGISTER_ADDRESS_STATUS,&d);
            }else{
                LOG_DEBUG(UART_puts("ts751 state=");UART_flush(););
                LOG_DEBUG(UART_put_HEX16(d);UART_flush(););
                LOG_DEBUG(UART_puts(".\n");UART_flush(););
                i2c_state = 1;
                if((d!=0) && ((d&0xff)==0))
                    s->mainstate = read_st751_hi;
            }
            break;
        case read_st751_hi:
            if(i2c_state){
                STTS751_read_regsiter(STTS751_REGISTER_ADDRESS_HI,&d);
            }else{
                LOG_DEBUG(UART_puts("hi d=");UART_flush(););
                LOG_DEBUG(UART_put_uint16(d);UART_flush(););
                LOG_DEBUG(UART_puts(".\n");UART_flush(););
                i2c_state = 1;
                temperature = (d&0xff)<<8;
                s->mainstate = read_st751_lo;
                LOG_DEBUG(s2 = (short )((d&0xff) * 256 ););
            }
            break;
        case read_st751_lo:
            if(i2c_state){
                STTS751_read_regsiter(STTS751_REGISTER_ADDRESS_LO,&d);
            }else{
//                LOG_DEBUG(UART_puts("lo d=%x.\n",d);UART_flush(););
                g_info.st751_need_power = 0;
//                LOG_DEBUG(s2 += (d&0xff););
                s2 = (s2 * 1000)/256;
//                LOG_DEBUG(UART_puts("%ld.%ld\n", s2/1000,(s2%1000));UART_flush(););
                s->mainstate = wait_eeprom_write;
                temperature += (d&0xff);
//                LOG_DEBUG(UART_puts("%04x",temperature);UART_flush(););
                if(temperature >= 87*0x100){
                    temperature = 87*0x100;
                }
                temperature += 40*0x100;
//                LOG_DEBUG(UART_puts("offset:%04x",temperature);UART_flush(););
                temperature = temperature>>7;
//                LOG_DEBUG(UART_puts("shift:%04x",temperature);UART_flush(););

                if(write_to_buffered_eeprom((unsigned char)temperature,1)>0){
                    g_info.i2c_need_power = true;
                }
                LOG_DEBUG(UART_puts("write address ");UART_flush(););
                LOG_DEBUG(UART_put_HEX32(g_eeprom_cache.eeprom_address);UART_flush(););
                LOG_DEBUG(UART_puts("\n");UART_flush(););
            }
            break;
        case wait_eeprom_write:
             if(i2c_state){
//                eeprom_write_task(&g_eeprom_bk_write_buf);
             }else{
                switch_clock_to(LF31kHz);
                g_info.i2c_need_power = false;
                s->mainstate = sleep;
            }
            break;
        case prepare_for_external:
            if(i2c_state==0){
                switch_clock_to(LF31kHz);
                s->mainstate = wait_for_external_use_to_be_removed;
                s->i2c_need_power = false;
                disable_i2c();
            }else{
                eeprom_write_task(&g_eeprom_bk_write_buf);                
            }
            break;
        case abnormal:
            WDTCONbits.WDTPS = WDTPS_VALUE_SLEEP;  //2s
            s->mainstate = halt;
            switch_clock_to(LF31kHz);
            break;
        case halt:
            if(s->in_state_wake_count<300){
                sleep_after_uart_send();
            }else{
                asm("reset");                
            }
            break;
    }
}

void input_filter(_io_input_filter *f,unsigned char input)
{
    if(input==f->prev){
        if(f->count<INPUT_FILTER_THRESH){
            f->count++;
        }
        if(f->count==INPUT_FILTER_THRESH){
            f->count++;
            f->output = f->prev;
        }
    }else{
        f->count = 0;
        f->prev = input;
    }
}

void interrupt isr(void)
{
//    WDTCONbits.SWDTEN = 0;
    LATAbits.LATA2 = 1;
    if(INTCONbits.IOCIF && IOCAFbits.IOCAF3){
        LOG_DEBUG(UART_puts("IOCAF3\n"););
        g_external_request_input.count = 0;
        while(g_external_request_input.count < INPUT_FILTER_THRESH)
            input_filter(&g_external_request_input,PORTAbits.RA3);
        g_info.is_waked_by_ra3 = 1;
        if(g_external_request_input.output == 0){
            g_info.external_need_power = true;
        }else{
            g_info.initialize_request = true;
            g_info.external_need_power = false;            
        }
        update_power_on_demand(&g_info);
        INTCONbits.IOCIF = 0;
        IOCAF = 0;
    }
#ifdef UART_TX_INTERRUPT
    UART_TX_Interrupt_Handler();
#endif
    LATAbits.LATA2 = 0;
}

void init_peripheral(void)
{
    INTCONbits.IOCIF = 0;
    IOCAF = 0;
    T1CONbits.TMR1CS = 1; // Fosc
    INTCONbits.IOCIE = 1; // only interrupt;
    IOCAPbits.IOCAP3 = 1;
    IOCANbits.IOCAN3 = 1;
#ifdef UART_TX_INTERRUPT
    TXIE = 1;
#endif
}

void init_i2c(void)
{
    //	PORT C Assignments
    ANSELCbits.ANSC0 = 0;
    ANSELCbits.ANSC1 = 0;
    TRISCbits.TRISC0 = 1; // 
    TRISCbits.TRISC1 = 1; //  SCL signal to SEE (must be set as input)

    RC0PPS = RXXPPS_SOURCE_SDA;
    RC1PPS = RXXPPS_SOURCE_SCL;
    SSPDATPPSbits.SSPDATPPS = PPS_INPUT_RC0;
    SSPCLKPPSbits.SSPCLKPPS = PPS_INPUT_RC1;

    SSPCONbits.SSPM=0x08;       // I2C Master mode, clock = Fosc/(4 * (SSPADD+1))
    SSPCONbits.SSPEN=1;         // enable MSSP port

    // Solving for SSPADD = [(16000000/100000)-4]/4 and we get SSPADD = 39
    SSPADD = ((_XTAL_FREQ/I2C_FREQ)-4)/4;                // set Baud rate clock divider
}

void supply_power_on_demand(int on)
{
    if(on!=0){
        if(PIN_POWER_ON_DEMAND==0){
            PIN_POWER_ON_DEMAND = 1;
            LOG_DEBUG(UART_puts("ext power ON.\n"););
        }
    }else{
        if(PIN_POWER_ON_DEMAND==1){
            PIN_POWER_ON_DEMAND = 0;
            LOG_DEBUG(UART_puts("ext power OFF.\n"););
        }
    }
}

void init_power_on_demand(void)
{
    ANSELCbits.ANSC3 = 0;
    TRISCbits.TRISC3 = 0;
}

void update_power_on_demand(info_t* s)
{
    if(s->st751_need_power || s->i2c_need_power || s->external_need_power){
        supply_power_on_demand(1);
    }else
        supply_power_on_demand(0);
}

char* const statename[] = {
    "dummy",
    "init",
    "sleep",
    "wait_for_external_use_to_be_removed",
    "wait_10ms",
    "read_st751_status",
    "read_st751_lo",
    "read_st751_hi",
    "wait_eeprom_write",
    "prepare_for_external",
    "abnormal",
    "halt"
};

void connect_external_state_machine(info_t* s)
{    
    if(s->external_need_power && i2c_state==0 ){
        switch(s->mainstate){
            case prepare_for_external:
            case wait_for_external_use_to_be_removed:
            case abnormal:
            case halt:
            break;
            default :
                T1CONbits.TMR1ON = 0;
                switch_clock_to(HF1MHz);
               s->mainstate = prepare_for_external;
               s->i2c_need_power = true;
                flush_buffered_eeprom();
                break;
        }
    }
}

void state_monitor(info_t* s)
{    
    switch(s->mainstate){
        case sleep:
        case wait_for_external_use_to_be_removed:
            break;
        case read_st751_status:
            if(s->in_state_wake_count>28*10)
                s->mainstate = abnormal;
            break;
        case read_st751_hi:
            if(s->in_state_wake_count>28*3)
                s->mainstate = abnormal;
        case wait_eeprom_write:
            if(s->in_state_wake_count>207*3)
                s->mainstate = abnormal;
            break;
        case halt:
            break;
        default :
            if(s->in_state_wake_count>0x1000)
                s->mainstate = abnormal;
            break;
    }
}

void main(void) 
{   
    TRISCbits.TRISC4 = 0;
    ANSELCbits.ANSC4 = 0;
    TRISAbits.TRISA2 = 0;
    ANSELAbits.ANSA2 = 0;

#ifdef ENABLE_LOG_PWM
    TRISCbits.TRISC5 = 0;
    ANSELCbits.ANSC5 = 0;
    RC5PPS = 0b01110; //PWM3OUT
    T2CONbits.T2CKPS = 0b00; // 00=1:1,01=1:4,10=1:16,11=1:64
//    PR2 = 19; //31kHz = 31000/80 (2.6ms),  1MHz = 1000/64/10 (1.3ms))
//    PR2 = 49; //31kHz = 31000/80 (2.6ms),  1MHz = 1000/64/10 (1.3ms))
    PR2 = 99; //31kHz = 31000/80 (2.6ms),  1MHz = 1000/64/10 (1.3ms))
    T2CONbits.TMR2ON = 1;
    PWM3DCL = 0;
    PWM3CONbits.PWM3EN = 1;
    PWM3CONbits.PWM3OUT = 1;
#endif
    init_power_on_demand();
    if(PORTAbits.RA3==0){
        supply_power_on_demand(1);
        g_info.external_need_power = true;
    }

    // set up oscillator control register
    WDTCONbits.WDTPS = WDTPS_VALUE_WAKE;
    VREGPM = 1; //Low-Power Sleep mode enabled in Sleep.Draws lowest current in Sleep, slower wake-up

    switch_clock_to(LF31kHz);

    init_peripheral();
    TRISCbits.TRISC2 = 0;
    RC2PPS = 0b10100; // TX;
    init_i2c();
    WPUA = 0b00001000;
    WPUC = 0;
    OPTION_REGbits.nWPUEN = 0;
    INTCONbits.GIE = 1;
    INTCONbits.PEIE = 1;
    LOG_INFO(UART_puts("uart enabled.\n");UART_flush(););

    g_info.prev_state = init;
    while(1){
        state_machine(&g_info);
#ifdef ENABLE_LOG_PWM
        PWM3DCH = g_info.mainstate;
#endif
        connect_external_state_machine(&g_info);
        update_power_on_demand(&g_info);
        asm("CLRWDT");
//        LOG_DEBUG(if(g_info.prev_state != g_info.mainstate){UART_puts("state ");UART_puts(statename[g_info.prev_state]);UART_puts("->");UART_puts(statename[g_info.mainstate]);UART_puts("\n");UART_flush();});           
        if(g_info.prev_state != g_info.mainstate){
            LOG_DEBUG(UART_puts("state ");UART_puts(statename[g_info.prev_state]);UART_puts("->");UART_puts(statename[g_info.mainstate]);UART_puts("\n");UART_flush();); 
            LOG_DEBUG(UART_puts(" count ");UART_put_uint16(g_info.in_state_wake_count);UART_puts("\n");UART_flush();); 
            g_info.in_state_wake_count = 0;
        }else{
            if(g_info.in_state_wake_count<0xffff)
                g_info.in_state_wake_count++;
            state_monitor(&g_info);
            if(g_info.prev_state != g_info.mainstate){
                LOG_DEBUG(UART_puts("state ");UART_puts(statename[g_info.prev_state]);UART_puts("->");UART_puts(statename[g_info.mainstate]);UART_puts("\n");UART_flush();); 
                LOG_DEBUG(UART_puts(" count ");UART_put_uint16(g_info.in_state_wake_count);UART_puts("\n");UART_flush();); 
                g_info.in_state_wake_count = 0;
            }
        }
        g_info.prev_state = g_info.mainstate;
#ifndef UART_TX_INTERRUPT
        UART_TX_BK_TASK();
#endif
            PIN_CHECK = 1 - PIN_CHECK;
    }
    LOG_INFO(UART_puts("end of main\n");UART_flush(););
    while(1);
 }
