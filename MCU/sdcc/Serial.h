#ifndef __SERIAL_
#define __SERIAL_

#include <stc12.h>



__bit busy;
extern char UART_wptr;
extern char UART_rptr;
extern char buf_recv_UART[];

#define BAUD 9600
#define BRT             (65536 - FOSC / BAUD / 4)
#define LEN_BUF_UART 8

void UART0_ISR() __interrupt (4)
{
    if (TI)
    {
        TI = 0;
        busy = 0;
    }
    if (RI)
    {
        RI = 0;
        buf_recv_UART[UART_rptr++] = SBUF;
        UART_rptr %= LEN_BUF_UART;
    }
}

void init_UART0(unsigned int baud) // initialize UART0 using Timer 2
{
    SCON = 0x50;
    T2L = BRT;
    T2H = BRT >> 8;
    AUXR = 0x15;
    PS = 1; // set UART0 interrupt priority to HIGH
    wptr = 0x00;
    rptr = 0x00;
    busy = 0;
}

void write_UART0(char dat)
{
    while (busy);
    busy = 1;
    SBUF = dat;
}

void sendstr_UART0(char *p)
{
    while (*p)
    {
        write_UART0(*p++);
    }
}


#endif