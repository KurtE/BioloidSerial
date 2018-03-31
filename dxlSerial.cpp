/*
  ax12.cpp - ArbotiX library for AX/RX control.
  Copyright (c) 2008-2011 Michael E. Ferguson.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "dxlSerial.h"
#ifndef __ARDUINO_X86__
#include <avr/io.h>
#endif

unsigned char ax_rx_buffer[AX12_BUFFER_SIZE];
uint8_t ax_tx_buffer[AX12_BUFFER_SIZE];

// Lets have the init setup
static Stream* s_paxStream;
static int s_direction_pin = -1;    // assume no direction pin.


/** initializes serial1 transmit at baud, 8-N-1 */
void dxlInit(long baud, Stream* pstream, int direction_pin, int tx_pin, int rx_pin ) {
    // Need to enable the PU resistor on the TX pin
    s_paxStream = pstream;
    s_direction_pin = direction_pin;    // save away.

    // Lets do some init here
    if (s_paxStream == &Serial) {
        Serial.begin(baud);
    }

    if (s_paxStream == (Stream*)&Serial1) {
        Serial1.begin(baud);
#if defined(KINETISK) || defined(__MKL26Z64__)
        if (tx_pin != -1) {
            Serial1.setTX(tx_pin);
        } else {
            tx_pin = 1; // default Serial 1 TX
        }
        if (rx_pin != -1) {
            Serial1.setRX(rx_pin);
        }
        if (s_direction_pin == -1) {
            UART0_C1 |= UART_C1_LOOPS | UART_C1_RSRC;
            volatile uint32_t *reg = portConfigRegister(tx_pin);
            *reg = PORT_PCR_DSE | PORT_PCR_SRE | PORT_PCR_MUX(3) | PORT_PCR_PE | PORT_PCR_PS; // pullup on output pin;
        } else {
            Serial1.transmitterEnable(s_direction_pin);
        }
#elif defined(TEENSYDUINO)
        // Handle on Teensy2...
        if (s_direction_pin != -1) {
            Serial1.transmitterEnable(s_direction_pin);
        }
#endif
    }
#ifdef SERIAL_PORT_HARDWARE1
    if (s_paxStream == &Serial2) {
        Serial2.begin(baud);
#if defined(KINETISK)  || defined(__MKL26Z64__)
        if (s_direction_pin == -1) {
            UART1_C1 |= UART_C1_LOOPS | UART_C1_RSRC;
            CORE_PIN10_CONFIG = PORT_PCR_DSE | PORT_PCR_SRE | PORT_PCR_MUX(3) | PORT_PCR_PE | PORT_PCR_PS; // pullup on output pin;
        } else {
            Serial2.transmitterEnable(s_direction_pin);
        }

#endif
    }
#endif
#ifdef SERIAL_PORT_HARDWARE2
    if (s_paxStream == &Serial3) {
        Serial3.begin(baud);
#if defined(KINETISK)  || defined(__MKL26Z64__)
        if (s_direction_pin == -1) {
            UART2_C1 |= UART_C1_LOOPS | UART_C1_RSRC;
            CORE_PIN8_CONFIG = PORT_PCR_DSE | PORT_PCR_SRE | PORT_PCR_MUX(3) | PORT_PCR_PE | PORT_PCR_PS; // pullup on output pin;
        } else {
            Serial3.transmitterEnable(s_direction_pin);
        }
#endif
    }
#endif
#if defined(__MK64FX512__) || defined(__MK66FX1M0__) || defined(KINETISL)
    else if (s_paxStream == &Serial4) {
        Serial4.begin(baud);
        if (s_direction_pin == -1) {
            UART3_C1 |= UART_C1_LOOPS | UART_C1_RSRC;
            CORE_PIN32_CONFIG = PORT_PCR_DSE | PORT_PCR_SRE | PORT_PCR_MUX(3) | PORT_PCR_PE | PORT_PCR_PS; // pullup on output pin;
        } else {
            Serial4.transmitterEnable(s_direction_pin);
        }
    }
    else if (s_paxStream == &Serial5) {
        Serial5.begin(baud);
        if (s_direction_pin == -1) {
            UART4_C1 |= UART_C1_LOOPS | UART_C1_RSRC;
            CORE_PIN33_CONFIG = PORT_PCR_DSE | PORT_PCR_SRE | PORT_PCR_MUX(3) | PORT_PCR_PE | PORT_PCR_PS; // pullup on output pin;
        } else {
            Serial5.transmitterEnable(s_direction_pin);
        }
    }
#endif

    // Setup direction pin.  If Teensyduino then built in support in hardware serial class.
#if !defined(TEENSYDUINO)
    if (s_direction_pin != -1) {
        // For other setups...
        pinMode(s_direction_pin, OUTPUT);
        digitalWrite(s_direction_pin, LOW);
    }
#endif
    setRX(0);
}

void dxlEnd() {
     if (s_paxStream == &Serial) {
        Serial.end();
    }

    if (s_paxStream == (Stream*)&Serial1) {
        Serial1.end();
    }
#ifdef SERIAL_PORT_HARDWARE1
    if (s_paxStream == &Serial2) {
        Serial2.end();
    }
#endif
#ifdef SERIAL_PORT_HARDWARE2
    if (s_paxStream == &Serial3) {
        Serial3.end();
    }
#endif
#if defined(__MK64FX512__) || defined(__MK66FX1M0__)
    else if (s_paxStream == &Serial4) {
        Serial4.end();
    }
    else if (s_paxStream == &Serial5) {
        Serial5.end();
    }
#endif

    setRX(0);
}
   

/** helper functions to switch direction of comms */
// Removed extra call for normal case...
void setTXall(){
    setTX(0);
}

void setTX(int id){
    if (s_direction_pin != -1) {
#if !defined(TEENSYDUINO)
        digitalWrite(s_direction_pin, HIGH);
#endif
        return;
    }

#if defined(KINETISK)  || defined(__MKL26Z64__)
    // Teensy 3.1/2 or LC

    if (s_paxStream == (Stream*)&Serial1) {
        UART0_C3 |= UART_C3_TXDIR;
    }
    else if (s_paxStream == (Stream*)&Serial2) {
        UART1_C3 |= UART_C3_TXDIR;
    }
    else if (s_paxStream == (Stream*)&Serial3) {
        UART2_C3 |= UART_C3_TXDIR;
    }
#if defined(__MK64FX512__) || defined(__MK66FX1M0__) || defined(KINETISL)
    else if (s_paxStream == (Stream*)&Serial4) {
        UART3_C3 |= UART_C3_TXDIR;
    }
    else if (s_paxStream == (Stream*)&Serial5) {
        UART4_C3 |= UART_C3_TXDIR;
    }
#endif
#elif defined(__ARDUINO_X86__)
    // Currently assume using USB2AX or the like

#else
    if (s_paxStream == (Stream*)&Serial1)
        UCSR1B = /*(1 << UDRIE1) |*/ (1 << TXEN1);
#ifdef SERIAL_PORT_HARDWARE1
    if (s_paxStream == (Stream*)&Serial2)
        UCSR2B = /*(1 << UDRIE3) |*/ (1 << TXEN2);
#endif
#ifdef SERIAL_PORT_HARDWARE2
    if (s_paxStream == (Stream*)&Serial3)
        UCSR3B =  /*(1 << UDRIE3) |*/ (1 << TXEN3);
#endif
#endif
}



void flushAX12InputBuffer(void)  {
    // First lets clear out any RX bytes that may be lingering in our queue
    while (s_paxStream->available()) {
        s_paxStream->read();
    }
}

void setRX(int id){

    // First clear our input buffer
	flushAX12InputBuffer();
    //digitalWriteFast(4, HIGH);
    // Now setup to enable the RX and disable the TX
    // If we are using hardware direction pin, can bypass the rest...
    if (s_direction_pin != -1) {
#if !defined(TEENSYDUINO)
        // Make sure all of the output has happened before we switch the direction pin.
        s_paxStream->flush();
        digitalWrite(s_direction_pin, LOW);
#endif
        return;
    }

    // Make sure everything is output before switching.
    s_paxStream->flush();
#if defined(KINETISK)  || defined(__MKL26Z64__)
    // Teensy 3.1
    if (s_paxStream == (Stream*)&Serial1) {
        UART0_C3 &= ~UART_C3_TXDIR;
    }
    if (s_paxStream == (Stream*)&Serial2) {
        UART1_C3 &= ~UART_C3_TXDIR;
    }
    if (s_paxStream == (Stream*)&Serial3) {
        UART2_C3 &= ~UART_C3_TXDIR;
    }

#if defined(__MK64FX512__) || defined(__MK66FX1M0__) || defined(KINETISL)
    else if (s_paxStream == (Stream*)&Serial4) {
        UART3_C3 &= ~UART_C3_TXDIR;
    }
    else if (s_paxStream == (Stream*)&Serial5) {
        UART4_C3 &= ~UART_C3_TXDIR;
    }
#endif
#elif defined(__ARDUINO_X86__)
    // Currently assume using USB2AX or the like

#else
    if (s_paxStream == (Stream*)&Serial1) {
        UCSR1B = ((1 << RXCIE1) | (1 << RXEN1));
    }
#ifdef SERIAL_PORT_HARDWARE1
    if (s_paxStream == (Stream*)&Serial2)
        UCSR2B = ((1 << RXCIE2) | (1 << RXEN2);
#endif
#ifdef SERIAL_PORT_HARDWARE2
    if (s_paxStream == (Stream*)&Serial3)
        UCSR3B = ((1 << RXCIE3) | (1 << RXEN3));
#endif
#endif
    //digitalWriteFast(4, LOW);
}



void dxlWrite(unsigned char *pdata, int length){
#if defined(ARDUINO_ARCH_AVR)
    while (length--)
        dxlWrite(*pdata++);
#else
    s_paxStream->write(pdata, length);
#endif
}

/** Sends a character out the serial port, and puts it in the tx_buffer */
void dxlWrite(unsigned char data){
    // BUGBUG:: on AVR processors, try to force not using output queue
#if defined(ARDUINO_ARCH_AVR)
    if (s_paxStream == (Stream*)&Serial1) {
        while ( (UCSR1A & (1 << UDRE1)) == 0)
            ;
    }
#ifdef SERIAL_PORT_HARDWARE1
    if (s_paxStream == (Stream*)&Serial2) {
        while ( (UCSR2A & (1 << UDRE2)) == 0)
            ;
    }
#endif
#ifdef SERIAL_PORT_HARDWARE2
    if (s_paxStream == (Stream*)&Serial3) {
        while ( (UCSR3A & (1 << UDRE3)) == 0)
            ;
    }
#endif
#endif
    s_paxStream->write(data);
}
/** We have a one-way recieve buffer, which is reset after each packet is receieved.
    A wrap-around buffer does not appear to be fast enough to catch all bytes at 1Mbps. */

/** read back the error code for our latest packet read */
int dxlError;
int dxlGetLastError(){ return dxlError; }
/** > 0 = success */

#if defined(KINETISK)  || defined(__MKL26Z64__)
#define COUNTER_TIMEOUT 12000
#else
#define COUNTER_TIMEOUT 3000
#endif

int ax12ReadPacket(int length) {
    unsigned long ulCounter;
    unsigned char offset, checksum;
	unsigned char *psz;
	unsigned char *pszEnd;
    int ch;


    offset = 0;

	psz = ax_rx_buffer;
	pszEnd = &ax_rx_buffer[length];

    flushAX12InputBuffer();

	// Need to wait for a character or a timeout...
	do {
		ulCounter = COUNTER_TIMEOUT;
        while ((ch = s_paxStream->read()) == -1) {
			if (!--ulCounter) {
				return 0;		// Timeout
			}
		}
	} while (ch != 0xff) ;
	*psz++ = 0xff;
	while (psz != pszEnd) {
		ulCounter = COUNTER_TIMEOUT;
        while ((ch = s_paxStream->read()) == -1) {
			if (!--ulCounter)  {
				return 0;		// Timeout
			}
		}
		*psz++ = (unsigned char)ch;
	}
    checksum = 0;
    for(offset=2;offset<length;offset++)
        checksum += ax_rx_buffer[offset];
    if(checksum != 255){
        return 0;
    }else{
        return 1;
    }
}



/******************************************************************************
 * Packet Level
 */

/** Read register value(s) */
int ax12GetRegister(int id, int regstart, int length){
    setTX(id);
    // 0xFF 0xFF ID LENGTH INSTRUCTION PARAM... CHECKSUM
    int checksum = ~((id + 6 + regstart + length)%256);
    dxlWrite(0xFF);
    dxlWrite(0xFF);
    dxlWrite(id);
    dxlWrite(4);    // length
    dxlWrite(DXL_READ_DATA);
    dxlWrite(regstart);
    dxlWrite(length);
    dxlWrite(checksum);

    setRX(id);
    if(ax12ReadPacket(length + 6) > 0){
        dxlError = ax_rx_buffer[4];
        if(length == 1)
            return ax_rx_buffer[5];
        else
            return ax_rx_buffer[5] + (ax_rx_buffer[6]<<8);
    }else{
        return -1;
    }
}

/* Set the value of a single-byte register. */
void ax12SetRegister(int id, int regstart, int data){
    setTX(id);
    int checksum = ~((id + 4 + DXL_WRITE_DATA + regstart + (data&0xff)) % 256);
    dxlWrite(0xFF);
    dxlWrite(0xFF);
    dxlWrite(id);
    dxlWrite(4);    // length
    dxlWrite(DXL_WRITE_DATA);
    dxlWrite(regstart);
    dxlWrite(data&0xff);
    // checksum =
    dxlWrite(checksum);
    setRX(id);
    //ax12ReadPacket();
}
/* Set the value of a double-byte register. */
void ax12SetRegister2(int id, int regstart, int data){
    setTX(id);
    int checksum = ~((id + 5 + DXL_WRITE_DATA + regstart + (data&0xFF) + ((data&0xFF00)>>8)) % 256);
    dxlWrite(0xFF);
    dxlWrite(0xFF);
    dxlWrite(id);
    dxlWrite(5);    // length
    dxlWrite(DXL_WRITE_DATA);
    dxlWrite(regstart);
    dxlWrite(data&0xff);
    dxlWrite((data&0xff00)>>8);
    // checksum =
    dxlWrite(checksum);
    setRX(id);
    //ax12ReadPacket();
}

// Lets try a protocol 1 ping for the servo
bool dxlP1Ping(int id) {
    setTX(id);
    // 0xFF 0xFF ID LENGTH INSTRUCTION PARAM... CHECKSUM
    int checksum = ~((id + 2 + DXL_PING)%256);
    dxlWrite(0xFF);
    dxlWrite(0xFF);
    dxlWrite(id);
    dxlWrite(2);    // length
    dxlWrite(DXL_PING);
    dxlWrite(checksum);

    setRX(id);
    if(ax12ReadPacket(6) > 0){
        dxlError = ax_rx_buffer[4];
        return true;
    }
    return false;
}


//==========================================================================================
// Protocol 2 stuff. 
uint16_t update_crc(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size)
{
    uint16_t i, j;
    uint16_t crc_table[256] = {
        0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
        0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
        0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
        0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
        0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
        0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
        0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
        0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
        0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
        0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
        0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
        0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
        0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
        0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
        0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
        0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
        0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
        0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
        0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
        0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
        0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
        0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
        0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
        0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
        0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
        0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
        0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
        0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
        0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
        0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
        0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
        0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
    };
 
    for(j = 0; j < data_blk_size; j++)
    {
        i = ((uint16_t)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
        crc_accum = (crc_accum << 8) ^ crc_table[i];
    }
 
    return crc_accum;
}

//========================================================
// Lets try to make a little more state driven read in function
// versus the hard code stuff from the protocol 1 implemention.
enum {DXL_RP2_FIRST_FF=0, DXL_RP2_SECOND_FF, DXL_RP2_FD, DXL_RP2_R0, DXL_RP2_ID,
    DXL_RP2_LEN1, DXL_RP2_LEN2, DXL_RP2_DATA, DXL_RP2_PACKET_COMPLETED};

int dxlP2ReadPacket() {
    unsigned long ulCounter;
    unsigned char *psz;
    uint16_t packet_length = 0, bytes_left = 0;
    int ch;
    
    uint8_t state = DXL_RP2_FIRST_FF;

    psz = ax_rx_buffer;

    flushAX12InputBuffer();

    // Need to wait for a character or a timeout...
    while (state != DXL_RP2_PACKET_COMPLETED) {
        ulCounter = COUNTER_TIMEOUT;
        while (s_paxStream->available() == 0) {
            if (!--ulCounter) {
                //if (state != DXL_RP2_FIRST_FF)
                    //Serial.printf("DXL2PRP: %d %d %d\n", state, packet_length, bytes_left);
                return 0;       // Timeout
            }
        }
        ch = s_paxStream->read();
        switch (state) {
            case DXL_RP2_FIRST_FF:
                if (ch == 0xff) state = DXL_RP2_SECOND_FF;
                break;
            case DXL_RP2_SECOND_FF:
                state = (ch == 0xff)? DXL_RP2_FD : DXL_RP2_FIRST_FF;
                break;
            case DXL_RP2_FD:
                if (ch == 0xfd) state = DXL_RP2_R0;
                else if (ch != 0xff) state = DXL_RP2_FIRST_FF;
                break;
            case DXL_RP2_R0:
                state = (ch == 0)? DXL_RP2_ID : DXL_RP2_FIRST_FF;
                break;
            case DXL_RP2_ID:
                *psz++ = 0xff;  // setup the start of the packet.
                *psz++ = 0xff;
                *psz++ = 0xfd;
                *psz++ = 0;
                *psz++ = ch;    // save out the ID
                state = DXL_RP2_LEN1;
                break;    
            case DXL_RP2_LEN1: 
                *psz++ = ch;
                packet_length = ch;
                state = DXL_RP2_LEN2;
                break;
            case DXL_RP2_LEN2:
                *psz++ = ch;
                packet_length += (ch << 8);  // we don't handle yet.
                bytes_left = packet_length;
                state = DXL_RP2_DATA;
                break;
            case DXL_RP2_DATA:
                *psz++ = ch;
                bytes_left--;
                if (bytes_left == 0) state = DXL_RP2_PACKET_COMPLETED;
                break;      
        }
    }
    // Ok we have a packet lets verify the checksum
    uint16_t CRC = update_crc ( 0, ax_rx_buffer, packet_length + 5) ;
    if ((ax_rx_buffer[packet_length+5] != ( CRC & 0xff)) ||
            (ax_rx_buffer[packet_length+6] != ( (CRC >> 8) & 0xff))) {
        //Serial.printf("DXL2PRP: (%d)checksum %x != %02x%02x\n", 
        //    packet_length, CRC, 
        //    ax_rx_buffer[packet_length+6], ax_rx_buffer[packet_length+5]);
        return -1;  // checksum error;
    }
    return packet_length;
}

// Protocol2 functions
//
// Assumes size is 1, 2, 4 3 would work
void dxlP2SetRegisters(int id, int regstart, uint32_t data, uint8_t data_size){
    if ((data_size < 1) || (data_size > 4)) return; // 
    setTX(id);
    uint8_t *packet = ax_tx_buffer;
    *packet++ = 0xFF;   //0
    *packet++ = 0xFF;   // 1
    *packet++ = 0xFd;   // 2
    *packet++ = 0;      // 3
    *packet++ = id;     // 4
    *packet++ = 5 + data_size;    // length lsb 5
    *packet++ = 0;    // msb
    *packet++ = DXL_WRITE_DATA;
    *packet++ = regstart & 0xff;        // Reg start low/high
    *packet++ = (regstart >> 8) & 0xff;  
    while (data_size--) {
        *packet++ = data & 0xff;
        data >>= 8; 
    }
    // checksum =
    uint16_t CRC = update_crc ( 0, ax_tx_buffer , ax_tx_buffer[5] + 5) ;
    *packet++ = CRC & 0xff;
    *packet++ = (CRC>>8) & 0xff;

    dxlWrite(ax_tx_buffer, (uint16_t)(packet-ax_tx_buffer)); // 13 bytes...
    setRX(id);
    //ax12ReadPacket();
}

/** Read register value(s) */
int dxlP2GetRegisters(int id, int regstart, int length){
    setTX(id);

    uint8_t *packet = ax_tx_buffer;
    *packet++ = 0xFF;   //0
    *packet++ = 0xFF;   // 1
    *packet++ = 0xFd;   // 2
    *packet++ = 0;      // 3
    *packet++ = id;     // 4
    *packet++ = 7;    // length lsb 5
    *packet++ = 0;    // msb
    *packet++ = DXL_READ_DATA;
    *packet++ = regstart & 0xff;        // Reg start low/high
    *packet++ = (regstart >> 8) & 0xff;  
    *packet++ = length & 0xff;
    *packet++ = (length >> 8) & 0xff;

    // checksum =
    uint16_t CRC = update_crc ( 0, ax_tx_buffer, 7+ 5) ;
    *packet++ = CRC & 0xff;
    *packet++ = (CRC>>8) & 0xff;
    dxlWrite(ax_tx_buffer, (uint16_t)(packet-ax_tx_buffer)); // 
    setRX(id);

    if(dxlP2ReadPacket() > 0){
        dxlError = ax_rx_buffer[8];
        if(length == 1)
            return ax_rx_buffer[9];
        else if (length == 2)
            return ax_rx_buffer[9] + (ax_rx_buffer[10]<<8);
        else
            return ax_rx_buffer[9] + (ax_rx_buffer[10]<<8) + 
                (ax_rx_buffer[11]<<16)+ (ax_rx_buffer[12]<<24);
    }else{
        return -1;
    }
}

uint32_t dxlP2Ping(int id) {
    setTX(id);

    uint8_t *packet = ax_tx_buffer;
    *packet++ = 0xFF;   //0
    *packet++ = 0xFF;   // 1
    *packet++ = 0xFd;   // 2
    *packet++ = 0;      // 3
    *packet++ = id;     // 4
    *packet++ = 3;    // 
    *packet++ = 0;    // msb
    *packet++ = DXL_PING;

    // checksum =
    uint16_t CRC = update_crc ( 0, ax_tx_buffer, 3+ 5) ;
    *packet++ = CRC & 0xff;
    *packet++ = (CRC>>8) & 0xff;
    dxlWrite(ax_tx_buffer, (uint16_t)(packet-ax_tx_buffer)); // 
    setRX(id);

    if(dxlP2ReadPacket() > 0){
        dxlError = ax_rx_buffer[8];
        return ax_rx_buffer[9] + (ax_rx_buffer[10]<<8) + (ax_rx_buffer[11]<<16);
    }else{
        return 0;
    }
}

//=============================================================================

bool dxlP1SyncWrite(uint8_t servo_count, uint8_t regstart, uint8_t regcount, uint8_t *buffer) {
    uint16_t buffer_length = (servo_count * (regcount+1));   
    int checksum = 254 + (buffer_length + 4) + DXL_SYNC_WRITE + regcount + regstart;
    setTXall();
    ax12write(0xFF);
    ax12write(0xFF);
    ax12write(0xFE);
    ax12write(buffer_length + 4);
    ax12write(DXL_SYNC_WRITE);
    ax12write(regstart);
    ax12write(regcount);
    for(int i=0; i<buffer_length; i++)
    {
        checksum += *buffer;
        ax12write(*buffer);
        buffer++;
    } 
    ax12write(0xff - (checksum % 256));
    setRX(0);

    return true;
}

//=============================================================================

bool dxlP2SyncWrite(uint8_t servo_count, uint16_t regstart, uint16_t regcount, uint8_t *buffer) {
    uint8_t *packet = ax_tx_buffer;
    uint16_t buffer_length = (servo_count * (regcount+1));  
    uint16_t packet_length = buffer_length + 7;
    *packet++ = 0xFF;   //0
    *packet++ = 0xFF;   // 1
    *packet++ = 0xFd;   // 2
    *packet++ = 0;      // 3
    *packet++ = 0xfe;     // 4
    *packet++ = packet_length & 0xff;    // 
    *packet++ = packet_length >> 8;    // msb
    *packet++ = DXL_SYNC_WRITE;
    *packet++ = regstart & 0xff;    // 
    *packet++ = regstart >> 8;    // msb
    *packet++ = regcount & 0xff;    // 
    *packet++ = regcount >> 8;    // msb

    // checksum = currently the 
    uint16_t CRC = update_crc ( 0, ax_tx_buffer,  (uint16_t)(packet-ax_tx_buffer));
    CRC = update_crc(CRC, buffer, buffer_length);
    setTXall();
    dxlWrite(ax_tx_buffer, (uint16_t)(packet-ax_tx_buffer)); // output the first part of the packet
    dxlWrite(buffer, buffer_length);
    dxlWrite(CRC & 0xff);
    dxlWrite((CRC>>8) & 0xff);

    setRX(0);
    return true;
}

//=============================================================================

bool dxlP2SyncRead(uint8_t servo_count, uint16_t regstart, uint16_t regcount, uint8_t *buffer) {
    uint8_t *packet = ax_tx_buffer;
    uint16_t buffer_length = servo_count;  
    uint16_t packet_length = buffer_length + 7;
    *packet++ = 0xFF;   //0
    *packet++ = 0xFF;   // 1
    *packet++ = 0xFd;   // 2
    *packet++ = 0;      // 3
    *packet++ = 0xfe;     // 4
    *packet++ = packet_length & 0xff;    // 
    *packet++ = packet_length >> 8;    // msb
    *packet++ = DXL_SYNC_READ;
    *packet++ = regstart & 0xff;    // 
    *packet++ = regstart >> 8;    // msb
    *packet++ = regcount & 0xff;    // 
    *packet++ = regcount >> 8;    // msb

    // checksum = currently the 
    uint16_t CRC = update_crc ( 0, ax_tx_buffer,  (uint16_t)(packet-ax_tx_buffer));
    CRC = update_crc(CRC, buffer, buffer_length);
    setTXall();
    dxlWrite(ax_tx_buffer, (uint16_t)(packet-ax_tx_buffer)); // output the first part of the packet
    dxlWrite(buffer, buffer_length);
    dxlWrite(CRC & 0xff);
    dxlWrite((CRC>>8) & 0xff);

    setRX(0);
    return true;
}

