#include "uart.h"
#include <string.h>

#define RX_BUF_LOC 0x0100
#define RX_BUF_LOC_H 0x01
#define RX_BUF_LOC_L 0x00
#define TX_BUF_LOC 0x0200
#define TX_BUF_LOC_H 0x02
#define TX_BUF_LOC_L 0x00

uint8_t ackCmd[2] = { 0x14, 0x00 };

uint8_t rxBuf[256] __at(RX_BUF_LOC);
uint8_t txBuf[256] __at(TX_BUF_LOC);

void uart_init() {
    DMA1CON1 = 0b01000000; //increment dest pointer, leave src pointer, never clear SIRQEN
    DMA1SSZH = 0x00;
    DMA1SSZL = 0x01; //source size: 1
    DMA1SSAU = 0x00;
    DMA1SSAH = 0x3d;
    DMA1SSAL = 0xe8; //source address: 0x3de8 = U1RXB
    DMA1DSZH = 0x01;
    DMA1DSZL = 0x00; //dest size: 256 = 0x0100
    DMA1DSAH = RX_BUF_LOC_H;
    DMA1DSAL = RX_BUF_LOC_L; //dest address: RX_BUF_LOC
    DMA1SIRQ = 0x1b; //trigger: 0x1b = U1RXIF
    DMA1AIRQ = 0x1d; //abort: 0x1d = U1EIF
    DMA1CON0 = 0b11000100; //set EN and SIRQEN immediately
    
    DMA2CON1 = 0b00000011; //leave dest pointer, increment src pointer, clear SIRQEN after src reload
    DMA2SSZH = 0x00;
    DMA2SSZL = 0x01; //source size: TBD (1 for now)
    DMA2SSAU = 0x00;
    DMA2SSAH = TX_BUF_LOC_H;
    DMA2SSAL = TX_BUF_LOC_L; //source address: TX_BUF_LOC
    DMA2DSZH = 0x00;
    DMA2DSZL = 0x01; //dest size: 1
    DMA2DSAH = 0x3d;
    DMA2DSAL = 0xea; //dest address: 0x3dea = U1TXB
    DMA2SIRQ = 0x1c; //trigger: 0x1c = U1TXIF
    DMA2AIRQ = 0x1d; //abort: 0x1d = U1EIF
    DMA2CON0 = 0b00000100; //set EN, not SIRQEN for now
    
    PRLOCK = 0x55;
    PRLOCK = 0xAA;
    PRLOCKbits.PRLOCKED = 1;
    
    U1BRGH = 0;
    U1BRGL = 34; //set up baud rate: closest possible to 115200 (114285.714)
    U1RXPPS = 0b001001; //RX: Pin B1
    U1CON0 = 0b00110000;
    U1CON1 = 0b10000000;
    U1CON2 = 0b00001000;
    RB0PPS = 0b010011; //TX: Pin B0
}

void handle_message(uint8_t* message, uint8_t len) {
    if (message[0] == 0x17 && message[2] == 0x04 && len == 4) {
        volume_level = message[3] & 0x0f;
    } else if (message[0] == 0x1E && len == 8) {
        on = message[1] != 0x00;
        connected = message[2] != 0x00;
        pairing = message[1] == 0x01;
        streaming = message[6];
    } else if (message[0] == 0x00 && len == 3) {
        
    } else return;
    
    ackCmd[1] = message[0];
    uart_send(ackCmd, 2);
}

void transmitChecksum() {
    while (!U1TXIF);
    U1TXB = -(U1TXCHK - 0xaa);
    DMA2SCNTIF = 0;
}

void uart_tasks() {
    static uint8_t rxPos = 0;
    
    if (DMA2SCNTIF) transmitChecksum();
    
    if (DMA1DPTRL == rxPos) return;
    uint8_t rxLen = DMA1DPTRL - rxPos;
    uint8_t i;
    bool zeroByte = false;
    uint8_t msgLen = 0;
    uint8_t checksum = 0;
    uint8_t message[256];
    uint8_t msgOffset = 0;
    uint8_t rawMsgOffset = 0;
    for (i = 0; i < rxLen; i++) {
        uint8_t bytePos = i - msgOffset;
        uint8_t intPos = rxPos + i;
        uint8_t val = rxBuf[intPos];
        uint8_t posInRawMsg = bytePos - (3 + zeroByte);
        if (bytePos == zeroByte) {
            if (val == 0x00 && !zeroByte) {
                zeroByte = true;
            } else if (val != 0xaa) {
                msgOffset = intPos + 1; //invalid first byte: skip
                zeroByte = false;
            }
        } else if (bytePos == 1 + zeroByte) {
            checksum += val; //high length byte: ignore
        } else if (bytePos == 2 + zeroByte) {
            msgLen = val; //low length byte: use as length
            checksum += val;
            if (msgLen == 0) {
                msgOffset = intPos + 1; //invalid length: command probably incomplete, skip
                zeroByte = false;
                checksum = 0;
            }
        } else if (posInRawMsg < msgLen) {
            message[rawMsgOffset + posInRawMsg] = val; //read actual message bytes
            checksum += val;
        } else if (posInRawMsg == msgLen) {
            checksum += val; //read and add checksum
            if (checksum == 0x00) {
                handle_message(message + rawMsgOffset, msgLen); //valid checksum: handle message
            }
            msgOffset = intPos + 1; //set offsets and reset values for next message
            zeroByte = false;
            rawMsgOffset += msgLen;
            checksum = 0;
        }
    }
    rxPos = msgOffset;
}

void uart_send(uint8_t* buf, uint8_t len) {
    while (DMA2CON0bits.SIRQEN); //wait for current transmission to end
    if (DMA2SCNTIF) transmitChecksum();
    DMA2CON0bits.EN = 0;
    txBuf[0] = 0xAA;
    txBuf[1] = 0x00;
    txBuf[2] = len;
    memcpy(txBuf + 3, buf, len);
    DMA2SSZH = ((uint16_t)len + 3) >> 8;
    DMA2SSZL = (len + 3) & 0xff;
    U1TXCHK = 0;
    DMA2SCNTIF = 0;
    DMA2CON0bits.EN = 1;
    DMA2CON0bits.SIRQEN = 1;
}