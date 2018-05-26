/*
  Copyright (c) 2015 Ian R. Haynes.  All right reserved.

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

#ifndef IbusSerial_h
#define IbusSerial_h

#include "Arduino.h"
#include <avr/eeprom.h>
#include "IbusGlobals.h"
#include "RingBuffer.h"

//#define TIE 0x2
//#define TEN 0x1
#define TIE 0x1
#define TEN 0x0
#if defined(__AVR_ATmega2560__)
#define Rx 15
#define Tx 14
#elif defined(__AVR_ATmega328P__) || defined(__MK20DX256__) || defined(__MK20DX128__) || defined(__MK64FX512__)
#define Rx 0
#define Tx 1
#endif

class IbusSerial
{

public:

	IbusSerial();
		~IbusSerial();
	void setIbusSerial(HardwareSerial &newIbusSerial);
#ifdef IBUS_DEBUG	
	void setIbusDebug(Stream &newIbusDebug);
#endif
	void run();
	void write(const byte message[], byte size);
	void printDebugMessage(const __FlashStringHelper *debugPrefix);
	typedef void IbusPacketHandler_t(byte *packet);
	void setIbusPacketHandler(IbusPacketHandler_t newHandler);
	void contentionTimer();
	void startTimer();
	void sleepEnable(unsigned long sleepTime);
	void sleep();
	void stopScroll();
	byte transceiverType;
	boolean sleepEnabled;
	boolean debugEnabled;
	boolean busInSync;
	HardwareSerial *pSerial;
	void setScrollSpeed(uint16_t, uint16_t, uint16_t, uint8_t);
	void radioWrite(const char *);
	void navWrite(const char *, const byte *, const byte *);
	void displayCurrentMessage();
	uint16_t startDelay; // Delay before scrolling starts
	uint16_t endDelay; // Delay before scrolling re-starts
	uint16_t scrollSpeed; // Time in milliseconds between scroll steps
	uint8_t scrollRepeats; // No of scroll repeats
	const uint16_t maxScrollDelay = 6000;
	const uint16_t minScrollDelay = 500;
	const uint16_t maxScrollSpeed = 1000;
	const uint16_t minScrollSpeed = 100;
	const uint8_t maxScrollRepeats = 8;
	const uint8_t minScrollRepeats = 0;

	
private:

	enum ScrollStates 
	{
	SCROLL_IDLE,
	SCROLL_START,
	SCROLL_START_DELAY,
	SCROLL_SCROLLING,
	SCROLL_SCROLLING_DELAY,
	SCROLL_END,
	SCROLL_END_DELAY,
	SCROLL_END_DISPLAY
	} 
	scrollState;

	enum IbusFsmStates 
	{
	FIND_SOURCE,              // Read source byte
	FIND_LENGTH,              // Read length byte
	FIND_MESSAGE,             // Read in main body of message
	GOOD_CHECKSUM,            // Process message if checksum good
	BAD_CHECKSUM,             // Debug print bad message if checksum bad
	} 
	ibusState;
	
	
	void readIbus();
	void compareIbusPacket();
	void sendIbusPacket();
	void sendIbusMessageIfAvailable();
		
#ifdef IBUS_DEBUG
	Stream *pIbusDebug;
#endif
	byte source;
	byte length;
	byte destination;
	byte ibusByte[40];
	static const byte packetGap = 10;
	byte *pData;
	IbusPacketHandler_t *pIbusPacketHandler;
	byte fieldLength;
	
	void scroll();
	void buildStaticMessage(const char *);
	void buildStaticMessage(const char *, const byte *, const byte *);
	void buildScrollMessage(byte = 0);
	void buildScrollMessage(byte startPosn, const byte * field, const byte * posn);
	unsigned long scrollTimer;
	byte maxChars; // Max number of characters to display on headunit before scrolling is required
	byte scrollRepeatCounter; // Used to remember how many scroll repates we have done
	byte loopCounter; // Keep track of how many loops we have done
	byte startPos; // Index position into scrollStrng
	byte loops; // Number of scroll loops to display complete message
	char scrollStrng[128]; // Used to hold messages to be scrolled on radio display
	static byte IBUS_MESSAGE_BUFFER[32];
	

};
#endif