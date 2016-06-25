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

#include "ax12Serial.h"
#ifndef __ARDUINO_X86__
#include <avr/io.h>
#endif

unsigned char ax_rx_buffer[AX12_BUFFER_SIZE];

// Lets have the init setup
static Stream* s_paxStream;
static int s_direction_pin = -1;    // assume no direction pin.


/** initializes serial1 transmit at baud, 8-N-1 */
void ax12Init(long baud, Stream* pstream, int direction_pin ){
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
        if (s_direction_pin == -1) {
            UART0_C1 |= UART_C1_LOOPS | UART_C1_RSRC;
            CORE_PIN1_CONFIG = PORT_PCR_DSE | PORT_PCR_SRE | PORT_PCR_MUX(3) | PORT_PCR_PE | PORT_PCR_PS; // pullup on output pin;
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


/** Sends a character out the serial port. */
void inline ax12write(unsigned char data){
    ax12writeB(data);
}

void ax12write(unsigned char *pdata, int length){
#if defined(ARDUINO_ARCH_AVR)
    while (length--)
        ax12writeB(*pdata++);
#else
    s_paxStream->write(pdata, length);
#endif
}

/** Sends a character out the serial port, and puts it in the tx_buffer */
void ax12writeB(unsigned char data){
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
int ax12Error;
int ax12GetLastError(){ return ax12Error; }
/** > 0 = success */

#if defined(KINETISK)  || defined(__MKL26Z64__)
#define COUNTER_TIMEOUT 12000
#else
#define COUNTER_TIMEOUT 3000
#endif

int ax12ReadPacket(int length){
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
    ax12writeB(0xFF);
    ax12writeB(0xFF);
    ax12writeB(id);
    ax12writeB(4);    // length
    ax12writeB(AX_READ_DATA);
    ax12writeB(regstart);
    ax12writeB(length);
    ax12writeB(checksum);

    setRX(id);
    if(ax12ReadPacket(length + 6) > 0){
        ax12Error = ax_rx_buffer[4];
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
    int checksum = ~((id + 4 + AX_WRITE_DATA + regstart + (data&0xff)) % 256);
    ax12writeB(0xFF);
    ax12writeB(0xFF);
    ax12writeB(id);
    ax12writeB(4);    // length
    ax12writeB(AX_WRITE_DATA);
    ax12writeB(regstart);
    ax12writeB(data&0xff);
    // checksum =
    ax12writeB(checksum);
    setRX(id);
    //ax12ReadPacket();
}
/* Set the value of a double-byte register. */
void ax12SetRegister2(int id, int regstart, int data){
    setTX(id);
    int checksum = ~((id + 5 + AX_WRITE_DATA + regstart + (data&0xFF) + ((data&0xFF00)>>8)) % 256);
    ax12writeB(0xFF);
    ax12writeB(0xFF);
    ax12writeB(id);
    ax12writeB(5);    // length
    ax12writeB(AX_WRITE_DATA);
    ax12writeB(regstart);
    ax12writeB(data&0xff);
    ax12writeB((data&0xff00)>>8);
    // checksum =
    ax12writeB(checksum);
    setRX(id);
    //ax12ReadPacket();
}

