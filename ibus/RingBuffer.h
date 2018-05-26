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

#ifndef RingBuffer_h
#define RingBuffer_h

#include "Arduino.h"

class RingBuffer
{
private:

	int bufferSize;
	unsigned int bufferHead, bufferTail;
	byte *pBuffer;
	
public:
	RingBuffer(int size);
        ~RingBuffer();
	int available(void);
	byte peek(void);
	byte peek(int);
	void remove(int);
	byte read(void);
	byte write(byte);
};
#endif
