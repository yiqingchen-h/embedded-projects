#ifndef STC8G1K08A_H
#define STC8G1K08A_H

/*
 * Minimal SFR set required by the current UART template.
 * Extend this header when Timer0, PWM, or ADC support is added.
 */

sfr P0       = 0x80;
sfr SP       = 0x81;
sfr DPL      = 0x82;
sfr DPH      = 0x83;
sfr PCON     = 0x87;
sfr TCON     = 0x88;
sfr TMOD     = 0x89;
sfr TL0      = 0x8A;
sfr TL1      = 0x8B;
sfr TH0      = 0x8C;
sfr TH1      = 0x8D;
sfr AUXR     = 0x8E;
sfr P1       = 0x90;
sfr CLKDIV   = 0x97;
sfr SCON     = 0x98;
sfr SBUF     = 0x99;
sfr P2       = 0xA0;
sfr P_SW1    = 0xA2;
sfr IE       = 0xA8;
sfr P3       = 0xB0;
sfr P3M1     = 0xB1;
sfr P3M0     = 0xB2;
sfr P_SW2    = 0xBA;
sfr ADC_CONTR = 0xBC;
sfr ADC_RES  = 0xBD;
sfr ADC_RESL = 0xBE;
sfr IP       = 0xB8;
sfr P4       = 0xC0;
sfr P5       = 0xC8;
sfr P5M1     = 0xC9;
sfr P5M0     = 0xCA;
sfr PSW      = 0xD0;
sfr ACC      = 0xE0;
sfr B        = 0xF0;

sbit IT0     = TCON ^ 0;
sbit IE0     = TCON ^ 1;
sbit IT1     = TCON ^ 2;
sbit IE1     = TCON ^ 3;
sbit TR0     = TCON ^ 4;
sbit TF0     = TCON ^ 5;
sbit TR1     = TCON ^ 6;
sbit TF1     = TCON ^ 7;

sbit RI      = SCON ^ 0;
sbit TI      = SCON ^ 1;

sbit P30     = P3 ^ 0;
sbit P31     = P3 ^ 1;
sbit P32     = P3 ^ 2;
sbit P33     = P3 ^ 3;
sbit P54     = P5 ^ 4;
sbit P55     = P5 ^ 5;

sbit EX0     = IE ^ 0;
sbit ET0     = IE ^ 1;
sbit EX1     = IE ^ 2;
sbit ET1     = IE ^ 3;
sbit ES      = IE ^ 4;
sbit EA      = IE ^ 7;

#endif /* STC8G1K08A_H */
