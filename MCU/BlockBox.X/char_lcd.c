#include "char_lcd.h"

void lcd_write(bool rs, uint8_t data, bool longdelay) {
    PORTEbits.RE2 = 1; //!E high
    PORTEbits.RE1 = 0; //reset R/W
    PORTEbits.RE0 = rs; //set RS if required
    __delay_us(10);
    PORTA = data;
    __delay_us(10);
    PORTEbits.RE2 = 0; //!E low
    __delay_us(10);
    PORTEbits.RE2 = 1; //!E high
    if (longdelay) __delay_ms(2);
    else __delay_us(50);
}

void lcd_init(bool inc, bool shift, bool cursor, bool cursorblink, bool on) {
    lcd_write(false, 0b00110011, false);
    lcd_write(false, 0b00110011, false);
    lcd_write(false, 0b00110011, false); //data length init
    lcd_write(false, 0b00000100 | (inc << 1) | shift, false); //entry mode
    lcd_write(false, 0b00001000 | (on << 2) | (cursor << 1) | cursorblink , false); //display control
    lcd_write(false, 0b00000001, true); //clear
}

void lcd_clear() {
    lcd_write(false, 0b00000001, true);
}

void lcd_home() {
    lcd_write(false, 0b00000010, true);
}

void lcd_set_char_addr(uint8_t addr) {
    lcd_write(false, 0b01000000 | (addr & 0b00111111), false);
}

void lcd_set_data_addr(uint8_t addr) {
    lcd_write(false, 0b10000000 | (addr & 0b01111111), false);
}

void lcd_write_byte(uint8_t data) {
    lcd_write(true, data, false);
}

void lcd_write_data(uint8_t* data, uint8_t offset, uint8_t length) {
    uint8_t i;
    for (i = 0; i < length; i++) {
        lcd_write(true, data[i + offset], false);
    }
}

void lcd_print(char* string) {
    uint8_t i = 0;
    while (string[i] != 0 && i < 255) {
        lcd_write(true, string[i++], false);
    }
}