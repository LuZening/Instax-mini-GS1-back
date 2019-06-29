#include <stc12.h>

SBIT(P5_4, 0xC0, 4)

#define KEY P3_2
#define ROLLER P5_4
#define AP P1_5
#define AN P1_4
#define BP P1_7
#define BN P1_6
#define SEG_A P1_2
#define SEG_B P1_1
#define SEG_C P1_0
#define SEG_D P3_7
#define SEG_E P3_4
#define SEG_F P3_6
#define SEG_G P3_5

void init_IO()
{
    ROLLER = 0; 
    P5M0 = 0x10;
    P5M1 = 0x00; // P5 strong Pullup
    P1M0 = 0x00;
    P1M1 = 0x00; // P1 Weak Pullup
    P3M0 = 0x00; // P3 Weak Pullup
    P3M1 = 0x00;
    P1 = 0xff;
    P3 = 0xff;
}