/*------------------------------------------------------------------------------* \
 * This source file is subject to the GPLv3 license that is bundled with this   *
 * package in the file COPYING.                                                 *
 * It is also available through the world-wide-web at this URL:                 *
 * http://www.gnu.org/licenses/gpl-3.0.txt                                      *
 * If you did not receive a copy of the license and are unable to obtain it     *
 * through the world-wide-web, please send an email to                          *
 * siempre.aprendiendo@gmail.com so we can send you a copy immediately.         *
 *                                                                              *
 * @category  Robotics                                                          *
 * @copyright Copyright (c) 2011 Jose Cortes (http://www.siempreaprendiendo.es) *
 * @license   http://www.gnu.org/licenses/gpl-3.0.txt GNU v3 Licence            *
 *                                                                              *
 \*------------------------------------------------------------------------------*/

#include "stdio.h"
#include <string.h>

#include "agse_package/Dynamixel.h"
#include "agse_package/Utils.h"

Dynamixel::Dynamixel()
{
  cleanBuffers();
}

Dynamixel::~Dynamixel()
{
	
}

// pos: input as gotten from getPosition()
// returns: floating point current angle
float Dynamixel::posToAngle(short pos)
{
  // ax-12a has an operating range: [0,300] degrees
  // which corresponds to a digital value range: [0,1024]
  // this gives us a precision of 0.29 degrees
  float angle = 0;
  angle = (float)pos * 0.29f;
  return angle;
}

// angle: floating point value between 0 and 300 degrees
// returns: short which can be sent to the motor
short Dynamixel::angleToPos(float angle)
{
  // ax-12a has an operating range: [0,300] degrees
  // which corresponds to a digital value range: [0,1024]
  // this gives us a precision of 0.29 degrees
  short pos = 0;
  pos = (short)(angle/0.29f);
  return pos;
}

void Dynamixel::cleanBuffers()
{
  memset(buffer,0,BufferSize);
  memset(bufferIn,0,BufferSize);
}


void Dynamixel::toHexHLConversion(short pos, byte *hexH, byte *hexL)
{    
  *hexH = (byte)(pos >> 8);
  *hexL = (byte)pos;
}

short Dynamixel::fromHexHLConversion(byte hexH, byte hexL)
{
  return (short)((hexL << 8) + hexH);
}

byte Dynamixel::checkSumatory(byte  data[], int length)
{
  int cs = 0;
  for (int i = 2; i < length; i++)
    {
      cs += data[i];
    }            
  return (byte)~cs;
}

int Dynamixel::getReadAX12LoadCommand(byte id)
{
  //OXFF 0XFF ID LENGTH INSTRUCTION PARAMETER1 �PARAMETER N CHECK SUM
  int pos = 0;

  buffer[pos++] = 0xff;
  buffer[pos++] = 0xff;
  buffer[pos++] = id;

  // length = 4
  buffer[pos++] = 4; //placeholder

  //the instruction, read => 2
  buffer[pos++] = 2;

  // pos registers 36 and 37
  buffer[pos++] = 40;

  //bytes to read
  buffer[pos++] = 2;

  byte checksum = checkSumatory(buffer, pos);
  buffer[pos++] = checksum;

  return pos;
}

int Dynamixel::getReadAX12PositionCommand(byte id)
{
  //OXFF 0XFF ID LENGTH INSTRUCTION PARAMETER1 �PARAMETER N CHECK SUM
  int pos = 0;

  buffer[pos++] = 0xff;
  buffer[pos++] = 0xff;
  buffer[pos++] = id;

  // length = 4
  buffer[pos++] = 4; //placeholder

  //the instruction, read => 2
  buffer[pos++] = 2;

  // pos registers 36 and 37
  buffer[pos++] = 36;

  //bytes to read
  buffer[pos++] = 2;

  byte checksum = checkSumatory(buffer, pos);
  buffer[pos++] = checksum;

  return pos;
}

int Dynamixel::getSetAX12PositionCommand(byte id, short goal)
{
  int pos = 0;
  byte numberOfParameters = 0;
  //OXFF 0XFF ID LENGTH INSTRUCTION PARAMETER1 �PARAMETER N CHECK SUM

  buffer[pos++] = 0xff;
  buffer[pos++] = 0xff;
  buffer[pos++] = id;

  // bodyLength
  buffer[pos++] = 0; //place holder

  //the instruction, query => 3
  buffer[pos++] = 3;

  // goal registers 30 and 31
  buffer[pos++] = 0x1E;// 30;

  //bytes to write
  byte hexH = 0;
  byte hexL = 0;
  toHexHLConversion(goal, &hexH, &hexL);
  buffer[pos++] = hexL;
  numberOfParameters++;
  buffer[pos++] = hexH;
  numberOfParameters++;

  // bodyLength
  buffer[3] = (byte)(numberOfParameters + 3);

  byte checksum = checkSumatory(buffer, pos);
  buffer[pos++] = checksum;

  return pos;
}

int setResponseLength = 6;
int getResponseLength = 8;

int Dynamixel::getPosition(SerialPort *serialPort, int idAX12) 
{
  int ret=0;

  int n=getReadAX12PositionCommand(idAX12);
  //bf(buffer,n);
  long l=serialPort->sendArray(buffer,n);
  Utils::sleepMS(waitTimeForResponse);

  memset(bufferIn,0,BufferSize);
  n=serialPort->getArray(bufferIn, n);
  //bf(bufferIn,n);
  memset(bufferIn,0,BufferSize);
  n=serialPort->getArray(bufferIn, getResponseLength);
  //bf(bufferIn,getResponseLength);

  short pos = -1;
  if (n>7)
    {
      pos = fromHexHLConversion(bufferIn[5], bufferIn[6]);				
    }

  printf("getPosition: id=<%i> pos=<%i>\n", idAX12, pos);
  if (pos<0 || pos > 1023)
    ret=-2;
  else
    ret=pos;

  return ret;
}

int Dynamixel::setPosition(SerialPort *serialPort, int idAX12, int position) 
{
  int error=0;

  int n=getSetAX12PositionCommand(idAX12, position);
  //bf(buffer,n);
  long l=serialPort->sendArray(buffer,n);
  Utils::sleepMS(waitTimeForResponse);

  memset(bufferIn,0,BufferSize);
  n=serialPort->getArray(bufferIn, n);
  //bf(bufferIn,n);
  memset(bufferIn,0,BufferSize);
  n=serialPort->getArray(bufferIn, setResponseLength);
  //bf(bufferIn,setResponseLength);

  if (n>4 && bufferIn[4] == 0)
    printf("setPosition: id=<%i> set at pos=<%i>\n", idAX12, position);
  else {
    error=-1;
    printf("setPosition: id=<%i> error: <%i>\n", idAX12, bufferIn[4]);
    bf(bufferIn, n);
  }

  return error;
}

int Dynamixel::getSetAX12SpeedCommand(byte id, short speed)
{
  int pos = 0;
  byte numberOfParameters = 0;
  //OXFF 0XFF ID LENGTH INSTRUCTION PARAMETER1 �PARAMETER N CHECK SUM

  buffer[pos++] = 0xff;
  buffer[pos++] = 0xff;
  buffer[pos++] = id;

  // bodyLength
  buffer[pos++] = 0; //place holder

  //the instruction, query => 3
  buffer[pos++] = 3;

  // goal registers 32 and 33
  buffer[pos++] = 0x20;// 20;

  //bytes to write
  byte hexH = 0;
  byte hexL = 0;
  toHexHLConversion(speed, &hexH, &hexL);
  buffer[pos++] = hexL;
  numberOfParameters++;
  buffer[pos++] = hexH;
  numberOfParameters++;

  // bodyLength
  buffer[3] = (byte)(numberOfParameters + 3);

  byte checksum = checkSumatory(buffer, pos);
  buffer[pos++] = checksum;

  return pos;
}

int Dynamixel::setSpeed(SerialPort *serialPort, int idAX12, int speed) 
{
  int error=0;

  int n=getSetAX12SpeedCommand(idAX12, speed);
  //bf(buffer,n);
  long l=serialPort->sendArray(buffer,n);
  Utils::sleepMS(waitTimeForResponse);

  memset(bufferIn,0,BufferSize);
  n=serialPort->getArray(bufferIn, n);
  //bf(bufferIn,n);
  memset(bufferIn,0,BufferSize);
  n=serialPort->getArray(bufferIn, setResponseLength);
  //bf(bufferIn,setResponseLength);

  if (n>4 && bufferIn[4] == 0)
    printf("setSpeed: id=<%i> set at pos=<%i>\n", idAX12, speed);
  else {
    error=-1;
    printf("setSpeed: id=<%i> error: <%i>\n", idAX12, bufferIn[4]);
    bf(bufferIn, n);
  }

  return error;
}

void Dynamixel::bf (byte *buffer, int n)
{
  printf ("Response (length <%i>): ",n);
  for (int i=0;i<n;i++)
    {
      if (i<(n-1))
	printf("%i,", buffer[i]);
      else
	printf("%i",buffer[i]);
    }
  printf("\n");
}
