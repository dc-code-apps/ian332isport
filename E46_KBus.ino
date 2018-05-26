// v0.1.0 - Beta branch

#include <avr/interrupt.h>
#include <IbusGlobals.h>
#include <IbusSerial.h>
#include <RingBuffer.h>

IbusSerial ibus; // Create an instance of the Ibus library called ibus.

// If using processor board with more than one HardwareSerial port, you don't need to use SoftwareSerial, or AltSoftSerial for
// DEBUG, so just define which HardwareSerial port you plan to use.
//#define debugSerial Serial2

// If using Arduino with only one hardware serial port (Uno, Nano etc) AND using MCP interface chip, use AltSoftSerial instead  
// of SoftwareSerial for DEBUG messages. SoftwareSerial uses pin change interrrupts that clash with the Ibus library. Also note
// that AltSoftSerial only works on pins 8(Rx) & 9(Tx).
//#include <AltSoftSerial.h>
//AltSoftSerial debugSerial;

// Software serial can be used with Melexis interface chip, but not with Microchip MCP Lin bus transceiver chips.
#include <SoftwareSerial.h>
SoftwareSerial debugSerial(7, 8); // 7 is Rx, 8 is Tx

// Setting up our variables
boolean goodPacket; //boolean value set true if good IBUS message detected
byte source;
byte length;
byte *pDebugByte;
byte databytes[36]; // Byte array to store message data bytes. ibusByte array of 40 - 4 for (Source, length, destination, checksumByte) = 36
byte outgoingMsg[32];
int outgoingSize;
unsigned long currentTime;


void setup()
{
  ibus.setIbusSerial(Serial); // Tell library which serial port is used for IBUS. Must be HardwareSerial port.
  ibus.setScrollSpeed(1500,1000,200,2); // Start delay, End delay, scroll speed & number of repeats. Defaults to 2000, 1000, 200, 2 if no alternatives specified
  ibus.setIbusPacketHandler(packetHandler); // Define callback function. This is the function that is called by the library when it has a complete IBUS packet/message.
  ibus.sleepEnable(60); // Enable sleep timer, and set sleep time in seconds. Comment out/delete line to disable timer.
  debugSerial.begin(9600); // Set debug Baud rate
  ibus.setIbusDebug(debugSerial); // Tell library which serial port is used for DEBUG. Comment out line if debug disabled in IbusGlobals.h
  debugSerial.println(F("--IBUS reader V2--"));
  attachInterrupt(digitalPinToInterrupt(3), startTimer, CHANGE);
  pinMode(ledPin, OUTPUT);
}
void loop()
{
  ibus.run(); // This keeps the IbusSerial library running.
}

// This is where we compare the received message with all the message bytes
// in IbusGlobals.h, or to known source/destination, and respond accordingly.
// 2. Implement timer to remove written text from radio display after X seconds

void packetHandler(byte *packet) // callback function
{
  
  source = packet[0];
  length = packet[1];
  byte destination = packet[2];
  pDebugByte = packet;

  // Identify message databytes - populate our databytes array
  for (int i = 0, s = 3; i <= length - 3; i++, s++) {
    databytes[i] = packet[s];
  }
  // Message from MFL_RT
  if(memcmp_P(packet, MFL_RT_PRESS, 6) == 0 ) {
    // This is how to send a known i-Bus message, from list in IbusGlobals.h.
    // It's sending the CD_STOP message whenever you press the
    // R/T button on the steering wheel. This is just an example, and probably
    // not something you would ever want to do.
    ibus.write(CD_STOP, sizeof(CD_STOP));
    }
  // Message from MFL_VOL_UP
  if(memcmp_P(packet, MFL_VOL_UP, 6) == 0 ) {
    // This is how to send text message to the Business CD/Radio display
    ibus.radioWrite("Vol UP Vol UP Vol UP ");
  }
  // Message from MFL_VOL_DOWN
  if(memcmp_P(packet, MFL_VOL_DOWN, 6) == 0 ) {
    ibus.radioWrite("Vol DOWN");
  }
  // Message from RLS to LCM
  if ((source == 0xE8) && (destination == 0xD0)){
    if (databytes[1] == 0x11) { 
      debugSerial.println(F("It's dark out, lights commanded on"));
      // TODO: 
      // Blinking alert LED wire will come into arduino as a digital input (Radar detector alert LED).
      // Implement dimming feature here. If this I-Bus message is present, find a method to dim this LED
      // Arduino output to actual LED == Increase resistance to ground? PWM? Or activate a relay that includes resistor?
    }
  }
}

void startTimer()
{
  ibus.startTimer();
}



