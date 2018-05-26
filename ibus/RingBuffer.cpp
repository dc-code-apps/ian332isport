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

#include "RingBuffer.h"

RingBuffer::RingBuffer( int size )
{
	bufferSize = size;
	bufferTail = 0;
	bufferHead = 0;
	pBuffer = ( byte* )malloc( size );
        memset( pBuffer, 0, size );  
}

RingBuffer::~RingBuffer()
{
	if( pBuffer )
          free( pBuffer );
}

// public functions

int RingBuffer::available(void)
{
  int ByteCount = (bufferSize + bufferHead - bufferTail) % bufferSize;
  return ByteCount;
}

byte RingBuffer::read(void) 
{
  if (bufferHead == bufferTail) // Buffer empty
  {
    return -1;
  }
  else {
    byte c = pBuffer[bufferTail];
    bufferTail = (bufferTail + 1) % bufferSize;
    if(bufferHead == bufferTail)
	{
      bufferTail = 0;
      bufferHead = 0;
    }
    return c;
  }
}

byte RingBuffer::write( byte c ) 
{ 
  if ((bufferHead + 1) % bufferSize == bufferTail)
  {
    return -1;
  }
  pBuffer[bufferHead] = c;
  bufferHead = (bufferHead + 1) % bufferSize;
  return 0;
}

void RingBuffer::remove(int n) 
{
  if(bufferHead != bufferTail)
  {
    bufferTail = (bufferTail + n) % bufferSize;
  }
}

byte RingBuffer::peek(void) 
{
  if (bufferHead == bufferTail)
  {
    return -1;
  }
  else {
    return pBuffer[bufferTail];
  }
}

byte RingBuffer::peek(int n)
{

  if (bufferHead == bufferTail)
  {
    return -1;
  }
  else {
    return pBuffer[(bufferTail + n) % bufferSize];
  }
}
