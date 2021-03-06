/*
 MIT License

Copyright (c) 2021 Konomu Abe

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

#include <xc.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "system.h"
#include "uart_print.h"
#include "stts751.h"
#include"mcc_generated_files/examples/i2c_master_example.h"
#include "mcc_generated_files/pin_manager.h"

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
    read_st751_status_init,
    read_st751_status_wait,
    read_st751_lo_init,
    read_st751_lo_wait,
    read_st751_hi_init,
    read_st751_hi_wait,
    eeprom_cache_write,
    eeprom_write_init,
    eeprom_write_wait,
    prepare_for_external,
    abnormal,
    halt
} state_t;

char* const statename[] = {
    "dummy",
    "init",
    "sleep",
    "wait_for_external_use_to_be_removed",
    "wait_10ms",
    "read_st751_status_init",
    "read_st751_status_wait",
    "read_st751_lo_init",
    "read_st751_lo_wait",
    "read_st751_hi_init",
    "read_st751_hi_wait",
    "eeprom_cache_write",
    "eeprom_write_init",
    "eeprom_write_wait",
    "prepare_for_external",
    "abnormal",
    "halt"
};


typedef enum {
    LF31kHz=0,
    HF1MHz
} cpu_clock_t;


typedef struct {
    bool is_eeprom_initialized ;
    bool is_waked_by_ra3 ;
    bool st751_need_power;
    bool eeprom_need_power;
    bool external_need_power;
    bool initialize_request;
    state_t mainstate;
    state_t prev_state;            // To monitor state transition
    state_t return_to_state;   // To call function like states
    unsigned short wdt_wake_count;
    unsigned short in_state_wake_count;
} info_t;

#define EEPROM_SEQ_LIMIT    (200*10)

#define EEPROM_BLOCK_SIZE   (64)
#define EEPROM_RING_BUF_SIZE   (EEPROM_BLOCK_SIZE*2)
#define EEPROM_RING_BUF_MASK   (0x7f)
typedef struct {
    unsigned long eeprom_address;
    unsigned long eeprom_address_unwritten;
    unsigned char eeprom_chache[EEPROM_RING_BUF_SIZE];
    unsigned eeprom_available :1; 
} _eeprom_cache_data;

void input_filter(_io_input_filter *f,unsigned char input);
void supply_power_on_demand(int on);
void STTS751_read_regsiter(unsigned char regsiter,unsigned short *d);
void update_power_on_demand(info_t* s);
bool load_eeprom_data(unsigned short search_start);
void I2C_EEPROM_ReadDataBlock(unsigned long mem_address, uint8_t *data, size_t len);
uint8_t make_control_byte(uint32_t  address);

i2c_operations_t rd1RegCompleteHandler(void *ptr);
i2c_operations_t wrBlkRegCompleteHandler(void *ptr);
i2c_operations_t rdBlkRegCompleteHandler(void *ptr);

_io_input_filter g_external_request_input;
volatile info_t g_info = {false,false,false,false,false,false,init,dummy,0,0};
//volatile info_t g_info = {false,false,false,false,false,false,read_st751_status_init,dummy,0,0};
_eeprom_cache_data g_eeprom_cache;
unsigned char g_eeprom_bk_write_buf[EEPROM_BLOCK_SIZE];

//unsigned short stts_log_interval = DEFAULT_LOG_INTERVAL_MINUTES*(60/2); /* (60s/2s)*10min */
unsigned short stts_log_interval = 0; /* (60s/2s)*10min */

system_error_t system_error;

unsigned short wc = 0;

void dump_eeprom_cache(void)
{
    int i;
    for(i=0;i<128;i++){
        UART_put_HEX8(g_eeprom_cache.eeprom_chache[i]);
        UART_puts(" ");
        UART_flush();
    }
        UART_puts("\n");
        UART_flush();
}

static i2c_operations_t i2c_nack_handler(void *ptr)
{
    i2c_is_nack = false;
     return I2C_STOP;        
}

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


void I2C_EEPROM_ReadDataBlock(unsigned long mem_address, uint8_t *data, size_t len)
{
    uint8_t address16[2];
    unsigned char retry = MAX_I2C_NO_NACK_RETRY;
    address16[0] = (mem_address>>8)&0xff;
    address16[1] = (mem_address       )&0xff;
    i2c_buffer_t bufferBlock; // result is little endian
    bufferBlock.data = data;
    bufferBlock.len = len;
    
    while((retry--) >0){
        i2c_is_nack = true;

        while(!I2C_Open(make_control_byte(mem_address))); // sit here until we get the bus..
        I2C_SetDataCompleteCallback(rdBlkRegCompleteHandler,&bufferBlock);
        I2C_SetBuffer(address16,2);
    //    I2C_SetAddressNackCallback(NULL,NULL); //NACK polling?
        I2C_SetAddressNackCallback(i2c_nack_handler,NULL); //NACK polling?
        I2C_SetDataNackCallback(i2c_nack_handler,NULL); //NACK polling?
        I2C_MasterWrite();
        while(I2C_BUSY == I2C_Close()); // sit here until finished.
//        UART_put_HEX8(i2c_is_nack);UART_puts("\n");
        if(i2c_is_nack)
            break;
    }
    if(retry==0){
        system_error.i2c_no_nack = 1;
    }
}

void I2C_EEPROM_WriteDataBlock(unsigned long mem_address, uint8_t *data, size_t len)
{
    uint8_t address16[2];
    address16[0] = (mem_address>>8)&0xff;
    address16[1] = (mem_address       )&0xff;
    i2c_buffer_t bufferBlock; // result is little endian
    bufferBlock.data = data;
    bufferBlock.len = len;
    i2c_is_nack = true;

    while(!I2C_Open(make_control_byte(mem_address))); // sit here until we get the bus..
    I2C_SetDataCompleteCallback(wrBlkRegCompleteHandler,&bufferBlock);
    I2C_SetBuffer(address16,2);
    I2C_SetAddressNackCallback(i2c_nack_handler,NULL); //NACK polling?
    I2C_SetDataNackCallback(i2c_nack_handler,NULL); //NACK polling?
    I2C_MasterWrite();
}

bool read_log_interval(void)
{
    unsigned short eerom_validation;
    unsigned char d;
        I2C_EEPROM_ReadDataBlock(TIME_INTERVAL_ADDRESS,&d,1);
    LOG_DEBUG(UART_puts("d=");UART_put_uint8(d);UART_puts("\n");UART_flush(););
    if(d>0){
        stts_log_interval =(WDT_COUNT_10MINUTES*(unsigned short)d)/DEFAULT_LOG_INTERVAL_MINUTES;
        LOG_DEBUG(UART_puts("stts_log_interval ");UART_put_uint16(stts_log_interval);UART_puts("\n");UART_flush(););
    }else{
        stts_log_interval = WDT_COUNT_10MINUTES;        
    }
    stts_log_interval = 10; /* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    return(true);
}

void state_machine(info_t* s)
{
    short temperature;
    long s2;
    unsigned char returnValue;
    unsigned char reg ;
    unsigned char i2c_nack_retry;
    switch(s->mainstate){
        case init:
            T1CONbits.TMR1ON = 0;
            PIR1bits.TMR1IF = 0;
            TMR1 = 0; 
            s->st751_need_power = false;
            s->eeprom_need_power = false;
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
                if(s->is_eeprom_initialized == false){
                    switch_clock_to(HF1MHz);
                    supply_power_on_demand(1);
                    if(!load_eeprom_data(EEPROM_HEADER_BLOCK_SIZE)){
                        s->mainstate = abnormal;
                        break;
                    }
                    if(g_eeprom_cache.eeprom_address==EEPROM_HEADER_BLOCK_SIZE){
                        s->wdt_wake_count = 0;
                        if(!read_log_interval()){
                            s->mainstate = abnormal;
                            break;                        
                        } 
                    }
                }
                switch_clock_to(LF31kHz);
                s->mainstate = sleep ;
            }else{
                LOG_DEBUG(UART_puts("going sleep.\n");UART_flush(););
                sleep_after_uart_send();
                LOG_DEBUG(UART_puts("woke up.\n");UART_flush(););
                if(s->wdt_wake_count<0xffff)
                    s->wdt_wake_count++;        
            }
            break;
        case sleep:
            if(!s->is_waked_by_ra3){
                LOG_DEBUG(UART_puts("going sleep.\n");UART_flush(););
                sleep_after_uart_send();
            }
            if(s->is_waked_by_ra3){ 
                LOG_DEBUG(UART_puts("woken up with ra3.\n");UART_flush(););
                s->is_waked_by_ra3 = false;   // clear flag. 
                if(g_eeprom_cache.eeprom_address != g_eeprom_cache.eeprom_address_unwritten ){
                    s->eeprom_need_power = true;
                    s->return_to_state = wait_for_external_use_to_be_removed;
                    s->mainstate = eeprom_write_init;
                }else{
                    s->mainstate = wait_for_external_use_to_be_removed;                    
                }
            }else{ // waked by WDT
                if(s->wdt_wake_count<0xffff)
                    s->wdt_wake_count++;        
                LOG_DEBUG(UART_puts("woken up by wdt. cnt = ");UART_flush(););
                LOG_DEBUG(UART_put_uint16(s->wdt_wake_count);UART_flush(););
                LOG_DEBUG(UART_puts("\n");UART_flush(););
                if(s->wdt_wake_count>=stts_log_interval){
                    s->wdt_wake_count -= stts_log_interval;
                    s->st751_need_power = true;
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
                s->mainstate = read_st751_status_init;
                i2c_nack_retry = 3;
            }
            break;

        case read_st751_status_init:
            enable_i2c();
           returnValue = 0x00;
           reg = STTS751_REGISTER_ADDRESS_STATUS;
           i2c_is_nack = true;
            I2C_Open(STTS751_I2C_ADDRESS);
            I2C_SetDataCompleteCallback(rd1RegCompleteHandler,&returnValue);
            I2C_SetBuffer(&reg,1);
            I2C_SetAddressNackCallback(NULL,NULL); //NACK polling?
            I2C_MasterWrite();
            s->mainstate = read_st751_status_wait;
            break;
        case read_st751_status_wait:
           if(I2C_BUSY == I2C_Close()){
               break;
           }else{
                LOG_DEBUG(UART_puts("ts751 state=");UART_flush(););
                LOG_DEBUG(UART_put_HEX16(returnValue);UART_flush(););
                LOG_DEBUG(UART_puts(".\n");UART_flush(););
                if(i2c_is_nack){
                    s->mainstate = read_st751_hi_init;
                    i2c_nack_retry = 3;
                }else{
                    if(--i2c_nack_retry>0){
                        s->mainstate = read_st751_status_init;
                    }else{
                        s->mainstate = abnormal;
                    }
                }
           }
            break;
        case read_st751_hi_init:
             reg = STTS751_REGISTER_ADDRESS_HI;
             i2c_is_nack = true;
             I2C_Open(STTS751_I2C_ADDRESS);
             I2C_SetDataCompleteCallback(rd1RegCompleteHandler,&returnValue);
             I2C_SetBuffer(&reg,1);
             I2C_SetAddressNackCallback(i2c_nack_handler,NULL); //NACK polling?
             I2C_MasterWrite();
             s->mainstate = read_st751_hi_wait;
            break;
        case read_st751_hi_wait:
             if(I2C_BUSY == I2C_Close())
               break;
            LOG_DEBUG(UART_puts("hi d=");UART_flush(););
            LOG_DEBUG(UART_put_uint16(returnValue);UART_flush(););
            LOG_DEBUG(UART_puts(".\n");UART_flush(););
            if(i2c_is_nack){
                temperature = (returnValue&0xff)<<8;
                s->mainstate = read_st751_lo_init;
                i2c_nack_retry = 3;
            }else{
                if(--i2c_nack_retry>0){
                    s->mainstate = read_st751_hi_init;
                }else{
                    s->mainstate = abnormal;
                }
             }
            break;
        case read_st751_lo_init:
             reg = STTS751_REGISTER_ADDRESS_LO;
             i2c_is_nack = true;
             I2C_Open(STTS751_I2C_ADDRESS);
             I2C_SetDataCompleteCallback(rd1RegCompleteHandler,&returnValue);
             I2C_SetBuffer(&reg,1);
             I2C_SetAddressNackCallback(i2c_nack_handler,NULL); //NACK polling?
             I2C_MasterWrite();
             s->mainstate = read_st751_lo_wait;
             break;
        case read_st751_lo_wait:
             if(I2C_BUSY == I2C_Close())
               break;
                LOG_DEBUG(UART_puts("lo d=");UART_flush(););
                LOG_DEBUG(UART_put_uint16(returnValue);UART_flush(););
                LOG_DEBUG(UART_puts(".\n");UART_flush(););
                if(i2c_is_nack){
                    i2c_nack_retry = 3;
                    s->st751_need_power = 0;
                    s2 = (s2 * 1000)/256;
                    temperature += (returnValue&0xff);
                    if(temperature >= 87*0x100){
                        temperature = 87*0x100;
                    }
                    temperature += 40*0x100;
                    temperature = temperature>>7;
                     s->mainstate = eeprom_cache_write;
                     wc++;
                }else{
                    if(--i2c_nack_retry>0){
                        s->mainstate = read_st751_lo_init;
                    }else{
                        s->mainstate = abnormal;
                    }
                }
            break;
        case eeprom_cache_write:
                   LOG_DEBUG(UART_puts("write address ");UART_flush();
                        UART_put_HEX32(g_eeprom_cache.eeprom_address);UART_flush();
                        UART_puts(" ");
                        UART_put_uint8(g_eeprom_cache.eeprom_address&EEPROM_RING_BUF_MASK);UART_flush();
                        UART_puts(" ");
                        UART_put_uint8(temperature);
                        UART_puts("\n");UART_flush();
                   );
                *((unsigned char *)(g_eeprom_cache.eeprom_chache +  (g_eeprom_cache.eeprom_address&EEPROM_RING_BUF_MASK)))  = temperature;
                g_eeprom_cache.eeprom_address ++;
                if(g_eeprom_cache.eeprom_address%EEPROM_BLOCK_SIZE == 0 ){
                        s->eeprom_need_power = true;
                        s->mainstate = eeprom_write_init;
                        s->return_to_state = sleep;
                }   else{
                    UART_flush();
                        switch_clock_to(LF31kHz);
//                        switch_clock_to(HF1MHz);
                        s->eeprom_need_power = false;
                        s->mainstate = sleep;                    
                }
                break;
        case eeprom_write_init:
            I2C_EEPROM_WriteDataBlock(
                    g_eeprom_cache.eeprom_address_unwritten&0x1ffc0,
                    &g_eeprom_cache.eeprom_chache[(g_eeprom_cache.eeprom_address_unwritten)&64],
                    g_eeprom_cache.eeprom_address-(g_eeprom_cache.eeprom_address_unwritten&0x1ffc0));
            UART_puts("EEPROM write ");
            UART_put_HEX32(g_eeprom_cache.eeprom_address_unwritten&0xffffffc0);
            UART_puts(" ");
             UART_put_uint8(g_eeprom_cache.eeprom_address-(g_eeprom_cache.eeprom_address_unwritten&0x1ffc0));
            UART_puts(" ");
            LOG_DEBUG(
                for(s2=0;s2<g_eeprom_cache.eeprom_address-(g_eeprom_cache.eeprom_address_unwritten&0x1ffc0);s2++){
                    UART_put_uint8(g_eeprom_cache.eeprom_chache[((g_eeprom_cache.eeprom_address_unwritten)&64)+s2]);
                    UART_puts(" ");
                  UART_flush();
                }
                UART_puts("\n");
                );
            s->mainstate = eeprom_write_wait;
            break;
        case eeprom_write_wait:
           if(I2C_BUSY == I2C_Close())
               break;
            if(i2c_is_nack){
                g_eeprom_cache.eeprom_address_unwritten += EEPROM_BLOCK_SIZE;
                 switch_clock_to(LF31kHz);
                 s->eeprom_need_power = false;
                 s->mainstate = sleep;
                s->mainstate = s->return_to_state;
             }else{
                 if(--i2c_nack_retry>0){
                     s->mainstate = eeprom_write_init;
                 }else{
                     s->mainstate = abnormal;
                 }
             }
          break;
        case prepare_for_external:
                switch_clock_to(LF31kHz);
                s->mainstate = wait_for_external_use_to_be_removed;
                s->eeprom_need_power = false;
                disable_i2c();
            break;
        case abnormal:
            s->st751_need_power = 0;
            s->eeprom_need_power = false;
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

void  IOC_InterruptHandler(void)
{
    LATAbits.LATA2 = 1;
    if(INTCONbits.IOCIF && IOCAFbits.IOCAF3){
//        LOG_DEBUG(UART_puts("IOCAF3\n"););
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
        INTCONbits.IOCIF = 0;
        IOCAF = 0;
    }
    LATAbits.LATA2 = 0;
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
    if(s->st751_need_power || s->eeprom_need_power || s->external_need_power){
        supply_power_on_demand(1);
    }else
        supply_power_on_demand(0);
}

void connect_external_state_machine(info_t* s)
{    
    if(s->external_need_power  ){
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
               s->eeprom_need_power = true;
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
        case read_st751_status_wait:
            if(s->in_state_wake_count>28*10)
                s->mainstate = abnormal;
            break;
        case read_st751_hi_wait:
            if(s->in_state_wake_count>28*3)
                s->mainstate = abnormal;
        case eeprom_write_wait:
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

void find_empty_addres(unsigned long *address,unsigned short search_start)
{
    long low = 0, high = EEPROM_SIZE_BYTE, step = EEPROM_SIZE_BYTE / 2;

    while (1) {
		*address = low + step;
		unsigned char d ;
//            LOG_DEBUG(UART_puts("find_empty low=");UART_put_HEX32(low);UART_puts(",high=");UART_put_HEX32(high);UART_puts(",step=");UART_put_HEX32(step);UART_puts("\n");UART_flush();)
        
            I2C_EEPROM_ReadDataBlock(*address<search_start? search_start: *address, &d, 1);
            if (d)
                low = *address;
            else 
                high = *address;
            step = step >> 1;
            if (step < EEPROM_BLOCK_SIZE)
                break;
	}

	*address = low;
    unsigned char i;
    for(i=0;i<2;i++){
         if((*address+EEPROM_BLOCK_SIZE)>=EEPROM_SIZE_BYTE) break;
            I2C_EEPROM_ReadDataBlock(*address,g_eeprom_cache.eeprom_chache,EEPROM_BLOCK_SIZE);
        
        for(step=0;step<EEPROM_BLOCK_SIZE;step+=1,*address+=1){
            if(*address>=EEPROM_SIZE_BYTE) break;
            if(search_start>0 && *address < search_start)
                continue;
            if(g_eeprom_cache.eeprom_chache[step]==0){
                LOG_DEBUG(UART_puts("find_empty first empty address=");UART_put_HEX32(*address);UART_puts("\n");UART_flush(););
                return;
            }
        }
    }    
    LOG_DEBUG(UART_puts("find_empty could not find empty address\n");UART_flush();)
    *address = EEPROM_SIZE_BYTE;        
}

bool load_eeprom_data(unsigned short search_start)
{
    long a;
    find_empty_addres(&g_eeprom_cache.eeprom_address,search_start);
    a = g_eeprom_cache.eeprom_address;
/*    if(a>=1){
        g_eeprom_cache.eeprom_address_unwritten = a;
        a -= 1;
        UART_put_HEX32(a&0x1ffc0);UART_puts("\n");UART_flush();
        I2C_EEPROM_ReadDataBlock(a&0x1ffc0,g_eeprom_cache.eeprom_chache,EEPROM_BLOCK_SIZE);
    }*/
        g_eeprom_cache.eeprom_address_unwritten = a;
        if(g_eeprom_cache.eeprom_address%64>0){
            UART_put_HEX32(g_eeprom_cache.eeprom_address&0x1ffc0);UART_puts("\n");UART_flush();
            I2C_EEPROM_ReadDataBlock(g_eeprom_cache.eeprom_address&0x1ffc0,g_eeprom_cache.eeprom_chache,EEPROM_BLOCK_SIZE);
        }
    return(true);
}

unsigned char zeros[64];

void write_zeros(void)
{
    unsigned long ad ;
    for(ad=0;ad<0x20000;ad+=64){
          I2C_EEPROM_WriteDataBlock(ad,zeros,64);
          while(I2C_BUSY == I2C_Close());
          if(ad%1024==0){
              UART_put_HEX32(ad);
              UART_puts("\n");
              UART_flush();
          }
    }        
}

void read_out(void)
{
    unsigned char i;
    unsigned char is_first;
    unsigned long ad ;
    for(ad=0;ad<0x20000;ad+=64){
          I2C_EEPROM_ReadDataBlock(ad,zeros,64);
          while(I2C_BUSY == I2C_Close());
          is_first = true;
          for(i=0;i<64;i++){
              if(ad+i>16 && zeros[i]==0){
                  ad = 0x20000;
                  break;
              }
              if(is_first){
                  is_first = false;
                  UART_put_HEX32(ad);
                  UART_puts(" ");
              }
              UART_put_HEX8(zeros[i]);
              UART_flush();
          }
          if(!is_first)
              UART_puts("\n");
    }        
}

//#define INIT_EEPROM
#define READ_EEPROM
#ifdef INIT_EEPROM
static const unsigned char test_header[16] = {1,0,0,0,  0,0,0,0,0,0,0,0,  0,0,10,0};
#endif

inline void logger_main(void)
{
    
    UART_init(_XTAL_FREQ,UART_BPS);

__delay_ms(10);
    UART_puts("Hello.\n");
     
#ifdef INIT_EEPROM
    LATCbits.LATC3 = 1;
    write_zeros();
    I2C_EEPROM_WriteDataBlock(0,test_header ,16);
    while(I2C_BUSY != I2C_Close());
    UART_puts("Header written.\n");    

    LATCbits.LATC3 = 0;
#endif
#ifdef READ_EEPROM
    LATCbits.LATC3 = 1;
    read_out();
    LATCbits.LATC3 = 0;
#endif
    
   UART_flush();
   WDTCONbits.SWDTEN = 1;
    IOCAF3_SetInterruptHandler(IOC_InterruptHandler);
    while(1){
        asm("CLRWDT");
        if(g_info.prev_state != g_info.mainstate){
            LOG_DEBUG(UART_puts("state ");UART_puts(statename[g_info.prev_state]);UART_puts("->");UART_puts(statename[g_info.mainstate]);UART_puts("\n");UART_flush();); 
            g_info.in_state_wake_count = 0;
            g_info.prev_state = g_info.mainstate;
        }else{
            if(g_info.in_state_wake_count<0xffff)
                g_info.in_state_wake_count++;
                LOG_DEBUG(UART_puts(" count ");UART_put_uint16(g_info.in_state_wake_count);UART_puts("\n");UART_flush();); 
        }
         state_machine(&g_info);
         update_power_on_demand(&g_info);
#ifndef UART_TX_INTERRUPT
        UART_TX_BK_TASK();
#endif
            PIN_CHECK = 1 - PIN_CHECK;
    }
}