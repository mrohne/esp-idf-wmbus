/***********************************************************************************
    Filename: tmode_rf_settings.h
***********************************************************************************/

#ifndef TMODE_RF_SETTINGS
#define TMODE_RF_SETTINGS

// Product = CC1101
// Chip version = A   (VERSION = 0x04)
// Crystal accuracy = 10 ppm
// X-tal frequency = 26 MHz
// RF output power = + 10 dBm
// RX filterbandwidth = 325.000000 kHz
// Deviation = 38 kHz
// Datarate = 32.630920 kBaud
// Modulation = (0) 2-FSK
// Manchester enable = (0) Manchester disabled
// RF Frequency = 868.9497 MHz
// Channel spacing = 199.951172 kHz
// Channel number = 0
// Optimization = -
// Sync mode = (5) 15/16 + carrier-sense above threshold
// Format of RX/TX data = (0) Normal mode, use FIFOs for RX and TX
// CRC operation = (0) CRC disabled for TX and RX
// Forward Error Correction = (0) FEC disabled
// Length configuration = (0) Fixed length packets, length configured in PKTLEN register.
// Packetlength = 255
// Preamble count = (2)  4 bytes
// Append status = 0
// Address check = (0) No address check
// FIFO autoflush = 0
// Device address = 0


const uint8_t tModeRfConfig[] = {
    TI_CCxxx0_FSCTRL1,   0x08,   //Frequency synthesizer control.
    TI_CCxxx0_FSCTRL0,   0x00,   //Frequency synthesizer control.
    TI_CCxxx0_FREQ2,     0x21,   //Frequency control word, high byte.
    TI_CCxxx0_FREQ1,     0x6B,   //Frequency control word, middle byte.
    TI_CCxxx0_FREQ0,     0xD0,   //Frequency control word, low byte.
    TI_CCxxx0_MDMCFG4,   0x5C,   //Modem configuration.  - 103 kBaud
    TI_CCxxx0_MDMCFG3,   0x04,   //Modem configuration.
    TI_CCxxx0_MDMCFG2,   0x05,   //Modem configuration.
    TI_CCxxx0_MDMCFG1,   0x22,   //Modem configuration.
    TI_CCxxx0_MDMCFG0,   0xF8,   //Modem configuration.
    TI_CCxxx0_CHANNR,    0x00,   //Channel number.
    TI_CCxxx0_DEVIATN,   0x44,   //Modem deviation setting (when FSK modulation is enabled).
    TI_CCxxx0_FREND1,    0xB6,   //Front end RX configuration.
    TI_CCxxx0_FREND0,    0x10,   //Front end RX configuration.
    TI_CCxxx0_MCSM0,     0x18,   //Main Radio Control State Machine configuration.
    TI_CCxxx0_FOCCFG,    0x2E,   //Frequency Offset Compensation Configuration.
    TI_CCxxx0_BSCFG,     0xBF,   //Bit synchronization Configuration.
    TI_CCxxx0_AGCCTRL2,  0x43,   //AGC control.
    TI_CCxxx0_AGCCTRL1,  0x09,   //AGC control.
    TI_CCxxx0_AGCCTRL0,  0xB5,   //AGC control.
    TI_CCxxx0_FSCAL3,    0xEA,   //Frequency synthesizer calibration.
    TI_CCxxx0_FSCAL2,    0x2A,   //Frequency synthesizer calibration.
    TI_CCxxx0_FSCAL1,    0x00,   //Frequency synthesizer calibration.
    TI_CCxxx0_FSCAL0,    0x1F,   //Frequency synthesizer calibration.
    TI_CCxxx0_FSTEST,    0x59,   //Frequency synthesizer calibration.
    TI_CCxxx0_TEST2,     0x81,   //Various test settings.
    TI_CCxxx0_TEST1,     0x35,   //Various test settings.
    TI_CCxxx0_TEST0,     0x09,   //Various test settings.
    TI_CCxxx0_IOCFG2,    0x06,   //GDO2 output pin configuration.
    TI_CCxxx0_IOCFG0,    0x00,   //GDO0 output pin configuration. Refer to SmartRF® Studio User Manual for detailed pseudo register explanation.
    TI_CCxxx0_PKTCTRL1,  0x00,   //Packet automation control.
    TI_CCxxx0_PKTCTRL0,  0x00,   //Packet automation control.
    TI_CCxxx0_ADDR,      0x00,   //Device address.
    TI_CCxxx0_PKTLEN,    0xFF,   //Packet length.
};

const uint8_t tModePaTable[] = {0xC2};
const uint8_t tModePaTableLen = 1;

#endif


/***********************************************************************************
  Copyright 2008 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
***********************************************************************************/

