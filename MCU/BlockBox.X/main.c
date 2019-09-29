/*
 * File:   main.c
 * Author: Alex
 *
 * Created on 5. September 2019, 19:12
 */

// PIC18LF47K42 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1L
#pragma config FEXTOSC = OFF    // External Oscillator Selection (Oscillator not enabled)
#pragma config RSTOSC = HFINTOSC_64MHZ// Reset Oscillator Selection (HFINTOSC with HFFRQ = 64 MHz and CDIV = 1:1)

// CONFIG1H
#pragma config CLKOUTEN = OFF   // Clock out Enable bit (CLKOUT function is disabled)
#pragma config PR1WAY = OFF     // PRLOCKED One-Way Set Enable bit (PRLOCK bit can be set and cleared repeatedly)
#pragma config CSWEN = ON       // Clock Switch Enable bit (Writing to NOSC and NDIV is allowed)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor enabled)

// CONFIG2L
#pragma config MCLRE = EXTMCLR  // MCLR Enable bit (If LVP = 0, MCLR pin is MCLR; If LVP = 1, RE3 pin function is MCLR )
#pragma config PWRTS = PWRT_OFF // Power-up timer selection bits (PWRT is disabled)
#pragma config MVECEN = ON      // Multi-vector enable bit (Multi-vector enabled, Vector table used for interrupts)
#pragma config IVT1WAY = OFF    // IVTLOCK bit One-way set enable bit (IVTLOCK bit can be cleared and set repeatedly)
#pragma config LPBOREN = OFF    // Low Power BOR Enable bit (ULPBOR disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled , SBOREN bit is ignored)

// CONFIG2H
#pragma config BORV = VBOR_190  // Brown-out Reset Voltage Selection bits (Brown-out Reset Voltage (VBOR) set to 1.90V)
#pragma config ZCD = OFF        // ZCD Disable bit (ZCD disabled. ZCD can be enabled by setting the ZCDSEN bit of ZCDCON)
#pragma config PPS1WAY = OFF    // PPSLOCK bit One-Way Set Enable bit (PPSLOCK bit can be set and cleared repeatedly (subject to the unlock sequence))
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config DEBUG = OFF      // Debugger Enable bit (Background debugger disabled)
#pragma config XINST = OFF       // Extended Instruction Set Enable bit (Extended Instruction Set and Indexed Addressing Mode disabled)

// CONFIG3L
#pragma config WDTCPS = WDTCPS_31// WDT Period selection bits (Divider ratio 1:65536; software control of WDTPS)
#pragma config WDTE = OFF        // WDT operating mode (WDT enabled regardless of sleep)

// CONFIG3H
#pragma config WDTCWS = WDTCWS_7// WDT Window Select bits (window always open (100%); software control; keyed access not required)
#pragma config WDTCCS = SC      // WDT input clock selector (Software Control)

// CONFIG4L
#pragma config BBSIZE = BBSIZE_512// Boot Block Size selection bits (Boot Block size is 512 words)
#pragma config BBEN = OFF       // Boot Block enable bit (Boot block disabled)
#pragma config SAFEN = OFF      // Storage Area Flash enable bit (SAF disabled)
#pragma config WRTAPP = OFF     // Application Block write protection bit (Application Block not write protected)

// CONFIG4H
#pragma config WRTB = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-30000Bh) not write-protected)
#pragma config WRTC = OFF       // Boot Block Write Protection bit (Boot Block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)
#pragma config WRTSAF = OFF     // SAF Write protection bit (SAF not Write Protected)
#pragma config LVP = ON         // Low Voltage Programming Enable bit (Low voltage programming enabled. MCLR/VPP pin function is MCLR. MCLRE configuration bit is ignored)

// CONFIG5L
#pragma config CP = OFF         // PFM and Data EEPROM Code Protection bit (PFM and Data EEPROM code protection disabled)

// CONFIG5H

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#define _XTAL_FREQ 64000000

#include <xc.h>
#include "char_lcd.h"
#include <stdio.h>
#include <math.h>

#define clamp(x, l, u) (x < l ? l : (x > u ? u : x))
#define min(x, y) (x < y ? x : y)
#define max(x, y) (x > y ? x : y)

#define BAT_FULL_VALUE 3014.0f
#define BAT_EMPTY_VALUE 2082.0f
#define SOUND_CENTER 1522
#define SOUND_NOISE_FLOOR 23
#define SOUND_AMPLIFY 300
#define SOUND_LAST_LEN 10
#define SOUND_SAMPLE_LEN 50
#define LOG_MULT 5864.55534f

uint8_t ledBrightness = 255;

void setLED(uint32_t r, uint32_t g, uint32_t b) {
    uint32_t sr = r * ledBrightness / 255;
    uint32_t sg = g * ledBrightness / 255;
    uint32_t sb = b * ledBrightness / 255;
    
    CCPR3H = min(sr, 4095) >> 8;
    CCPR3L = min(sr, 4095) & 0xff;
    CCPR2H = min(sg, 4095) >> 8;
    CCPR2L = min(sg, 4095) & 0xff;
    CCPR4H = min(sb, 4095) >> 8;
    CCPR4L = min(sb, 4095) & 0xff;
}

void main_loop() {
    static uint32_t counter = 0;
    static uint32_t sum = 0;
    static uint16_t last[SOUND_LAST_LEN];
    static uint32_t lastSum;
    static int16_t pos = 0;
    
    ADPCH = 0b010101; //analog input: C5
    ADCON0bits.GO = 1;
    while (ADCON0bits.GO) __delay_us(1);
    int16_t res = ((ADRESH << 8) | ADRESL) - SOUND_CENTER;
    sum += res < 0 ? -res : res;
    
    if (counter % SOUND_SAMPLE_LEN == SOUND_SAMPLE_LEN - 1) {
        int32_t sample = sum / SOUND_SAMPLE_LEN;
        sample -= SOUND_NOISE_FLOOR;
        if (sample < 0) sample = 0;
        
        uint16_t lastAvg = lastSum / SOUND_LAST_LEN / 2;
        
        //uint32_t rel = max(0, sample / lastAvg - 1) * SOUND_AMPLIFY;
        pos = max(pos, (sample - lastAvg) * SOUND_AMPLIFY);
        
        setLED(pos, 2048 - min(pos, 2048), 2048 - min(pos, 2048));
        
        pos = max(pos - (pos / 70 + 1), 0);
        
        sum = 0;
        
        lastSum -= last[(counter / SOUND_SAMPLE_LEN) % SOUND_LAST_LEN];
        lastSum += sample;
        last[(counter / SOUND_SAMPLE_LEN) % SOUND_LAST_LEN] = sample;
    }
    
    if (counter % 1000 == 999) {
        PORTCbits.RC0 = !PORTCbits.RC1; //LCD backlight switch

        ADPCH = 0b010110; //analog input: C6
        ADCON0bits.FM = 0; //left justify
        ADCON0bits.GO = 1;
        while (ADCON0bits.GO) __delay_us(1);
        ledBrightness = ADRESH;
        ADCON0bits.FM = 1; //right justify
    }
    
    if (++counter >= 100000) {
        ADPCH = 0b011011; //analog input: D3
        ADCON0bits.GO = 1;
        while (ADCON0bits.GO) __delay_us(1);
        float bat_percent = (((ADRESH << 8) | ADRESL) - BAT_EMPTY_VALUE) / (BAT_FULL_VALUE - BAT_EMPTY_VALUE) * 100.0f;
        //if (bat_percent > 100f) bat_percent = 100f;
        char batmsg[16];
        if (!PORTDbits.RD2) sprintf(batmsg, "Chg fault: %3.0f%%", bat_percent);
        else if (!PORTDbits.RD1) sprintf(batmsg, "Full Chg: %3.0f%%", bat_percent);
        else if (!PORTDbits.RD0) sprintf(batmsg, "Fast Chg: %3.0f%%", bat_percent);
        else sprintf(batmsg, "Battery: %3.0f%%", bat_percent);
        lcd_set_data_addr(0);
        lcd_print(batmsg);

        counter = 0;
    }
    
    asm("clrwdt");
    __delay_us(10);
}

void main(void) {
    TRISA = 0b00000000;
    PORTA = 0b00000000;
    ANSELA = 0b00000000;
    WPUA = 0b00000000;
    INLVLA = 0b11111111;
    SLRCONA = 0b11111111;
    ODCONA = 0b00000000;
    
    TRISB = 0b11001100;
    PORTB = 0b00010000;
    ANSELB = 0b00000000;
    WPUB = 0b11000000;
    INLVLB = 0b11111111;
    SLRCONB = 0b11111111;
    ODCONB = 0b00000000;
    
    TRISC = 0b01101110;
    PORTC = 0b00000000;
    ANSELC = 0b01100000;
    WPUC = 0b00001110;
    INLVLC = 0b11111111;
    SLRCONC = 0b11111111;
    ODCONC = 0b00000000;
    
    TRISD = 0b00001111;
    PORTD = 0b00000000;
    ANSELD = 0b00001000;
    WPUD = 0b00000111;
    INLVLD = 0b11111111;
    SLRCOND = 0b11111111;
    ODCOND = 0b00000000;
    
    TRISE = 0b11111000;
    PORTE = 0b00000100;
    ANSELE = 0b11110000;
    WPUE = 0b00001000;
    INLVLE = 0b11111111;
    SLRCONE = 0b11111111;
    ODCONE = 0b00000000;
        
    PMD0 = 0b01111111; //disable unused modules/peripherals
    PMD1 = 0b11111011;
    PMD2 = 0b01000111;
    PMD3 = 0b01010001;
    PMD4 = 0b11100000;
    PMD5 = 0b00110111;
    PMD6 = 0b00111111;
    PMD7 = 0b00000011;
    
    ADCON0 = 0b10000100;
    ADCON1 = 0b00000000;
    ADCON2 = 0b00000000;
    ADCON3 = 0b00000000;
    ADCLK = 0b00011111; //FOSC/64
    ADREF = 0b00000000;
    
    RC7PPS = 0b001010; //C7 = CCP2
    RD4PPS = 0b001011; //D4 = CCP3
    RD5PPS = 0b001100; //D5 = CCP4
    RD6PPS = 0b001110; //D6 = PWM6
    RD7PPS = 0b010000; //D7 = PWM8
    
    CCPTMRS0 = 0b01010101;
    CCP2CON = 0b10001111;
    CCP3CON = 0b10001111;
    CCP4CON = 0b10001111;
    PWM6CON = 0b10000000;
    PWM8CON = 0b10000000;
    
    T2PR = 255;
    T2CLK = 0b0001;
    T2CON = 0b10010000;
    
    CCPR2L = 0;
    CCPR2H = 0;
    CCPR3L = 0;
    CCPR3H = 0;
    CCPR4L = 0;
    CCPR4H = 0;
    PWM6DCH = 255;
    PWM6DCL = 255;
    PWM8DCH = 255;
    PWM8DCL = 255;
    
    PORTBbits.RB4 = 0; //reset BM62
    __delay_ms(50);
    PORTBbits.RB4 = 1;
    
    __delay_ms(50); //wait a bit longer before LCD init to allow LCD controller to boot up
    lcd_init(true, false, false, false, true);
    lcd_print("Hello World!");
    
    while (1) main_loop();
}
