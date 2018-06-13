// Print HEX I-BUS message to SoftwareSerial - for debug 
void printDebug(char debugType[3]) {
  if (debugType == "rx") {
    debugSerial.print("Rx: ");
    for(int i = 0; i <= length + 1; i++) {
      debugSerial.print(pDebugByte[i], HEX);
      debugSerial.print(" ");
    }
    if(goodPacket == true) {
      debugSerial.println();
    }
    else {
      debugSerial.print(" ");
      debugSerial.println("Message Bad");
    }
  }
  if (debugType == "tx") {
    debugSerial.print("Tx: ");
    for ( int i = 0; i < outgoingSize; i++) {
      debugSerial.print(outgoingMsg[i], HEX);
      debugSerial.print(" ");
    }
    debugSerial.println();
  }
}


