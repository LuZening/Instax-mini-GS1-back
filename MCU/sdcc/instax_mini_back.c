/*
PROG: Instax mni back control
CHIP: STC8F2K08S2
DATE: 2019/6/28
AUTHOR: LU, ZENING
*/
#define __SDCC
#define FOSC 6000000L

#include <string.h>
#include <8051.h>
#include <stc12.h>
#include "IO_plan.h"
#include "EEPROM.h"
#include "Serial.h"
#define SAVE_VALID_SIGN 0xAA
#define LVD3V0 0x03
#define LVDF 0x20 //PCON.5 LVD flag

#define SCHED_INTERVAL0 20 // timer0 interval in ms
#define LEN_OP 3
#define ROLLER_DURATION 30000 / SCHED_INTERVAL0 // 12 sec
#define STEPS_PER_ROUND 20
#define STEPPER_GEAR_RATIO 500
#define STEPPER_STEPS_DEFAULT STEPS_PER_ROUND *STEPPER_GEAR_RATIO / 2
#define STEPPER_CALIB_STEPLEN STEPS_PER_ROUND *STEPPER_GEAR_RATIO / 100
#define LEN_KEY_HIST 5
#define EEPROM_START_ADDR 0x1000 // from 4KB~
#define IDLE_TIMEOUT 10000 / SCHED_INTERVAL0 // 10 sec
#define KEY_OP_TIMEOUT 1700 / SCHED_INTERVAL0 // 1.7sec
#define KEY_RLS_CONFIRM_TIMEOUT 60/SCHED_INTERVAL0
#define SHORT_KEY_DURATION 300/SCHED_INTERVAL0
#define LONG_KEY_DURATION 700/SCHED_INTERVAL0

typedef unsigned char uint8_t;
typedef unsigned int uint16_t;

typedef char int8_t;
typedef int int16_t;

// constant lookup tables
const uint8_t SEG_TABLE[] = {
    0x01, 0x4f, 0x12, 0x06, 0x4c, 0x24, 0x20, 0x0f, 0x00, 0x04, // 0-9
    0x08, 0x60, 0x72, 0x42, 0x30, 0x38, 0x21, 0x48, 0x79, 0x47, // a-j
    0x58, 0x71};                                                 // K, L

const uint8_t KEY_OP_TABLE[] = {
    1, 0, 0, 0, 0, // . enable display
    2, 0, 0, 0, 0, // _ print
    2, 1, 2, 1, 0, // _ reset film counter
};
// Variables to be saved on the EEPROM
struct SavedData
{
    uint8_t film_counter;
    uint16_t stepper_cntdn;       // stepper count down
    uint16_t stepper_steps;       // initial value of stepper countdown
    int8_t stepper_dir;          // -1 backward, 1 forward
    int8_t stepper_state;       //0: AB,1:A-B ,2:A-B-,3:AB- 4-cycle driving
    uint8_t is_stepper_rollback; //
    uint16_t roller_counter;
    uint8_t save_valid_check;
};

union EEPROMData {
    struct SavedData data;
    uint8_t bytes[sizeof(struct SavedData)];
} cfg;

struct SavedData *pdata = &(cfg.data);
// global variables
uint16_t idle_period = 0; // number of ticks when no key is pressed
uint8_t key_period = 0;  // execute key commands when key_periods > THRESHOLD
uint8_t key_state = 0;   // key state machine 0: init 1: hold 2: transit 3: process
uint8_t key_hld_count = 0;
uint8_t key_rls_count = 0;
uint8_t key_n_hist_recs = 0; // number of history key records
uint8_t key_hist[LEN_KEY_HIST] = {0};
int16_t stepper_steps_calib = 0; // additional stepper steps by calibration
uint8_t is_calib = 0;            // flag is calibrating
uint8_t seg_pattern = 0xff;      // the pattern on 7SEG display
char UART_wptr;
char UART_rptr;
char buf_recv_UART[LEN_BUF_UART];
void load_saved_data()
{
    register uint8_t i;
    uint16_t addr = EEPROM_START_ADDR;
    for (i = 0; i < sizeof(union EEPROMData); ++i)
    {
        cfg.bytes[i] = IAPRead(addr++);
    }
}

void save_data()
{
    register uint8_t i;
    uint16_t addr = EEPROM_START_ADDR;
    for (i = 0; i < sizeof(union EEPROMData); ++i)
    {
        IAPWrite(addr++, cfg.bytes[i]);
    }
}

void init_savedvars() // initialize EEPROM variables, in case the EEPROM data is corrupted
{
    struct SavedData *p = &(cfg.data);
    p->film_counter = 0;
    p->stepper_cntdn = 0;
    p->stepper_steps = STEPPER_STEPS_DEFAULT;
    p->stepper_dir = 1;
    p->stepper_state = 0;
    p->is_stepper_rollback = 0;
    p->roller_counter = 0;
    p->save_valid_check = SAVE_VALID_SIGN;
}

void init_globalvars()
{
    idle_period = 0; // number of ticks when no key is pressed
    key_period = 0;  // execute key commands when key_periods > THRESHOLD
    key_state = 0;   // key state machine 0: init 1: hold 2: transit 3: process
    key_hld_count = 0;
    key_rls_count = 0;
    key_n_hist_recs = 0; // number of history key records
    memset(key_hist, 0, LEN_KEY_HIST);
    stepper_steps_calib = 0; // additional stepper steps by calibration
    is_calib = 0;            // flag is calibrating
    seg_pattern = 0xff;      // the pattern on 7SEG display
    UART_wptr = 0;
    UART_rptr = 0;
    memset(buf_recv_UART, 0, LEN_BUF_UART);
    
}

void Init_timer0(void) //20毫秒@6.000MHz
{
    AUXR &= 0x7F; //定时器时钟12T模式
    TMOD &= 0xF0; //设置定时器模式
    TL0 = 0xF0;   //设置定时初值
    TH0 = 0xD8;   //设置定时初值
    TF0 = 0;      //清除TF0标志
    TR0 = 1;      //定时器0开始计时
}

void Timer1Init(void) //1毫秒@6.000MHz
{
    AUXR &= 0xBF; //定时器时钟12T模式
    TMOD &= 0x0F; //设置定时器模式
    TL1 = 0x0C;   //设置定时初值
    TH1 = 0xFE;   //设置定时初值
    TF1 = 0;      //清除TF1标志
    TR1 = 1;      //定时器1开始计时
}

void power_down()
{
    disable_display();
    disable_stepper();
    disable_roller();
    save_data();
    VOCTRL = 0x80;    
    EA = 1;
    _nop_();
    _nop_();
    PCON |= 0x02; // set power down flag bit
    _nop_();
    _nop_();
    EX0 = 0;
    init_globalvars();    
    P5M1 ^= 0x10; // resume ROLLER pin mode to STRONG pull up
}


SFR(RSTCFG, 0xff);

void setup()
{
    EA = 0;
    init_IO();
    load_saved_data();
    if (cfg.data.save_valid_check != SAVE_VALID_SIGN)
        init_savedvars();
    // init timer 0
    Init_timer0();
    ET0 = 1;
    // Low voltage detector
    PCON &= ~LVDF;   //上电需要清中断标志
    RSTCFG = LVD3V0; //设置LVD电压为3.0V
    ELVD = 1;        //使能LVD中断

    EA = 1;
}

void update_display(uint8_t id)
{
    p = SEG_TABLE[id];
    SEG_G = p & 1;
    p >>= 1;
    SEG_F = p & 1;
    p >>= 1;
    SEG_E = p & 1;
    p >>= 1;
    SEG_D = p & 1;
    p >>= 1;
    SEG_C = p & 1;
    p >>= 1;
    SEG_B = p & 1;
    p >>= 1;
    SEG_A = p & 1;
}

void disable_display()
{
    SEG_A = 1;
    SEG_B = 1;
    SEG_C = 1;
    SEG_D = 1;
    SEG_E = 1;
    SEG_F = 1;
    SEG_G = 1;
}

void disable_roller()
{
    // change P5_4 IO mode to open-collector
    P5M0 |= 0x10; 
    P5M1 |= 0x10;
    ROLLER = 0;
}

void disable_stepper()
{
    TR1 = 0;
    ET1 = 0;
    AP = 1;
    AN = 1;
    BP = 1;
    BN = 1;
}

void main()
{
    setup();
    update_display(pdata->film_counter);
    while(1)
    {
        // process command from UART0
        if((UART_rptr > 1) && (buf_recv_UART[UART_rptr-1] == '\r')
         || (buf_recv_UART[UART_rptr-1] == '\n')) // deteced the termination of a command
        {
            switch (buf_recv_UART[UART_rptr - 2])
            {
                case  'F': // hock forward
                if(pdata->stepper_cntdn == 0)
                {
                    pdata->stepper_cntdn = STEPPER_CALIB_STEPLEN;
                    pdata->stepper_dir = 1;
                    pdata->is_stepper_rollback = true; // to avoid roll back
                    TR1 = 1;
                    ET1 = 1;
                    if(is_calib) stepper_steps_calib += STEPPER_CALIB_STEPLEN;
                }
                sendstr_UART0("FORWARD\r");
                break;
                case 'B': // hock backward
                if(pdata->stepper_cntdn == 0)
                {
                    pdata->stepper_cntdn = STEPPER_CALIB_STEPLEN;
                    pdata->stepper_dir = -1;
                    pdata->is_stepper_rollback = true; // to avoid roll back
                    TR1 = 1;
                    ET1 = 1;
                    if(is_calib) stepper_steps_calib -= STEPPER_CALIB_STEPLEN;
                }
                sendstr_UART0("BACK\r");
                break;
                case 'P': // roll
                pdata->roller_counter = ROLLER_DURATION;
                ROLLER = 1;
                sendstr_UART0("ROLL\r");
                break;
                case 'L': // calibrate
                is_calib = true;
                stepper_steps_calib = 0;
                sendstr_UART0("CALIB INIT\r");
                break;
                case 'S': // save calibrate
                if(stepper_steps_calib != 0)
                {
                    pdata->stepper_steps += stepper_steps_calib;
                    save_data();
                    sendstr_UART0('SAVED\r');
                }
                break;
                default:
                sendstr_UART0("BAD COMM\r");
            }
            UART_rptr = 0;
        }

    }
}

void execute_OP(uint8_t i)
{
    switch(i)
    {
        case 0: // . enable display
        update_display(pdata->film_counter);
        break;
        case 1: // _ print
        if(pdata->roller_counter == 0)
        {
            // temporarily disable Low Voltage detector
            ELVD = 0;
            // enable stepper
            pdata->stepper_cntdn = pdata->stepper_steps;
            pdata->stepper_dir = 1;
            pdata->is_stepper_rollback = false;
            TR1=1;
            ET1=1;
            // enable roller
            pdata->roller_counter = ROLLER_DURATION;
            ROLLER = 1;
            // proceed to next film
            update_display(++pdata->film_counter);
            ELVD = 1;
        }
        break;
        case 2: // -.-. reset film counter
        pdata->film_counter = 0;
        update_display(0);
    }
}

void timer0_ISR() __interrupt (1) __using (1) // main scheduler
{
    register uint8_t i;
    // handle roller count down
    if (pdata->roller_counter > 0)
    {
        pdata->roller_counter--;
        return;
    }
    else
    {
        ROLLER = 0; // stop roller
    }
    
    if (is_calib) return;

    // process key state machine
    switch (key_state)
    {
        case 0: // IDLE
            if(!KEY) // key pressed
            {
                key_period = 0;
                idle_period = 0;
                key_state++;
            }
            else
            {
                if(++key_period > KEY_OP_TIMEOUT)
                {
                    if(key_n_hist_recs > 0)
                    {
                        // match opcode
                        bool is_matched;
                        for (i=0; i < LEN_OP; ++i) // iterate through all OPcodes
                        {
                            is_matched = true;
                            for (j=0; j < LEN_KEY_HIST; ++j) // iterate through key records
                            {
                                if (KEY_OP_TABLE[i*LEN_KEY_HIST+j] != key_hist[j])
                                {
                                    is_matched = false;
                                    break;
                                }        
                            }
                            if (is_matched) break;
                        }
                        if (is_matched) // matched
                        {
                            execute_OP(i);
                        }
                        key_n_hist_recs = 0;
                        for(i=0 ; i<key_n_hist_recs; ++i) key_hist[i] = 0;
                    }
                    key_period = 0;
                }
                else if (++idle_period > IDLE_TIMEOUT)
                {
                    idle_period = 0;
                    power_down(); 
                }
            }
            break;
        case 1: // hold
            if(!KEY)
            {
                key_rls_count = 0;
                if(key_hld_count < 0xff)
                    key_hld_count++;
            }
            else
            {
                if(++key_rls_count > KEY_RLS_CONFIRM_TIMEOUT)
                {
                    if(key_hld_count >= SHORT_KEY_DURATION)
                    {
                        // process key
                        if(key_hld_count >= LONG_KEY_DURATION) // long key
                        {
                            key_hist[key_n_hist_recs] = 2;
                            key_n_hist_recs = (key_n_hist_recs + 1) % LEN_KEY_HIST;
                            // DEBUG
                            sendstr_UART0("LONG\r");
                        }
                        else     // short key
                        {
                            key_hist[key_n_hist_recs] = 1;
                            key_n_hist_recs = (key_n_hist_recs + 1) % LEN_KEY_HIST;
                            // DEBUG
                            sendstr_UART0("SHORT\r");
                        }
                        key_state = 0; // reset the key state machine
                        
                    }
                    else
                    {
                        key_state--;
                    }
                    key_rls_count = 0;
                    key_hld_count = 0;
                }
            }
            break;
        default:
            break;
    }
    
}

void timer1_ISR() __interrupt (3) __using (2) // stepper pacer 1Kpps
{
    switch (pdata->stepper_state)
    {
        case 0: //AB
        AP=1;
        AN=0;
        BP=1;
        BN=0;
        break;
        case 1: //A-B
        AP=0;
        AN=1;
        BP=1;
        BN=0; 
        break;
        case 2: //A-B-
        AP=0;
        AN=1;
        BP=0;
        BN=1;
        break;
        case 3: //AB-
        AP=1;
        AN=0;
        BP=0;
        BN=1;
    }
    pdata->stepper_state = (pdata->stepper_state + pdata->stepper_dir + 4) % 4;
    if(--pdata->stepper_cntdn == 0)
    {
        if(pdata->is_stepper_rollback)
        {
            disable_roller();
        }
        else // roll back
        {
            pdata->stepper_cntdn = pdata->stepper_steps;
            pdata->stepper_dir = -1;
            pdata->is_stepper_rollback = true;
        }
    }

}

void LVD_ISR() __interrupt (6)
{
    PCON &= ~LVDF; // clear LVD flag
    update_display('L' - 'A' + 10);
}
