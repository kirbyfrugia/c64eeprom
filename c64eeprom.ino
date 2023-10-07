#include <SoftwareSerial.h>
#include <Wire.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"

#define SHIFT_DATA 2
#define SHIFT_CLK 3
#define SHIFT_LATCH 4
#define EEPROM_D0 5
#define EEPROM_D7 12
#define WRITE_TIMEOUT_MS 500
#define EEPROM_CE 13
#define EEPROM_OE A1
#define EEPROM_WE A2
#define EXT_RX A0
#define EXT_TX A3

#define EXT_DATA_REPLY_TIMEOUT_MS 2000

// Wiring for OLED
// A4 - SDA, A5 - SCL

// Wiring for Commodore 64 serial
// A + N = Ground
// B + C = RX (64 side) <- TX (A3, Arduino side)
// M = TX (64 side) -> RX (A0, Arduino side)

#define OLED_I2C_ADDRESS 0x3C

SSD1306AsciiWire oled;

SoftwareSerial ext_Serial(EXT_RX, EXT_TX);

unsigned int startAddress = 0;
unsigned int currentAddress = 0;
int selectedEEPROM = -1;

const char * S_EEPROM_UPPER28 = "Upper EEPROM (28pin)";
const char * S_EEPROM_LOWER24 = "Lower EEPROM (24pin)";
const char * S_AWAITING_DATA = "Awaiting data...";
const char * S_CHECKSUM_ERROR = "Checksum error";
const char * S_WRITE_EEPROM_ERROR = "Error writing EEPROM";
const char * S_OK = "OK";
const char * S_ERROR_RESET = "Err. Reset to retry";
const char * S_DONE_RESET = "Ok. Reset to retry";

const char REPLY_TIMEOUT = 'T';
const char REPLY_ACK = 'A';
const char REPLY_CHECKSUM_ERROR = 'C';
const char REPLY_RETRY = 'R';

enum State { ERROR, DONE, AWAITING_NEXT_PACKET, AWAITING_HEADER_CHECKSUM_LO, AWAITING_HEADER_CHECKSUM_HI, AWAITING_BYTES };

State ext_state = AWAITING_NEXT_PACKET;

unsigned int ext_messageType = 0;
unsigned int ext_bytesExpected = 0;
unsigned int ext_bytesRead = 0;
unsigned int ext_totalBytesRead = 0;

unsigned int ext_checksumLo = 0;
unsigned int ext_checksumHi = 0;

unsigned long dataLastReceivedMS = 0; // how long it has been since we received any data at all

int ext_bytesBuf[16];
int ext_headerBuf[4];

void setDataPinsToRead() {
  for (unsigned int pin = EEPROM_D7; pin >= EEPROM_D0; pin -= 1) {
    pinMode(pin, INPUT);
  }
}

void setDataPinsToWrite() {
  for (unsigned int pin = EEPROM_D7; pin >= EEPROM_D0; pin -= 1) {
    pinMode(pin, OUTPUT);
  }
}

void setCurrentAddress(unsigned int address) {
  currentAddress = address;

  shiftOut(SHIFT_DATA, SHIFT_CLK, MSBFIRST, address >> 8);
  shiftOut(SHIFT_DATA, SHIFT_CLK, MSBFIRST, address);

  digitalWrite(SHIFT_LATCH, LOW);
  digitalWrite(SHIFT_LATCH, HIGH);
  digitalWrite(SHIFT_LATCH, LOW);
}

byte readData() {
  digitalWrite(EEPROM_OE, LOW);
  setDataPinsToRead();

  byte data = 0;
  for (unsigned int pin = EEPROM_D7; pin >= EEPROM_D0; pin -= 1) {
    data = (data << 1) + digitalRead(pin);
  }

  return data;
}

void enableSDP() {
  setDataPinsToWrite();
  setCurrentAddress(0x5555);
  writeData(0xAA, false);
  setCurrentAddress(0x2AAA);
  writeData(0x55, false);
  setCurrentAddress(0x5555);
  writeData(0xA0, false);
  delay(500);
}

void disableSDP() {
  setDataPinsToWrite();
  setCurrentAddress(0x5555);
  writeData(0xAA, false);
  setCurrentAddress(0x2aaa);
  writeData(0x55, false);
  setCurrentAddress(0x5555);
  writeData(0x80, false);
  setCurrentAddress(0x5555);
  writeData(0xaa, false);
  setCurrentAddress(0x2aaa);
  writeData(0x55, false);
  setCurrentAddress(0x5555);
  writeData(0x20, false);
  delay(500);
}

void pulseWrite() {
  digitalWrite(EEPROM_OE, HIGH);
  digitalWrite(EEPROM_WE, LOW);
  delayMicroseconds(1);
  digitalWrite(EEPROM_WE, HIGH);
  digitalWrite(EEPROM_OE, LOW);
}

// poll until we are sure we fully wrote the data, or until we time out
bool validateData(byte origData) {
  setDataPinsToRead();

  unsigned long startTime = millis();
  unsigned long elapsedTime = 0;

  byte data = 0;
  do {
    delay(1);
    data = readData();
    elapsedTime = millis() - startTime;
  } while (data != origData && elapsedTime < WRITE_TIMEOUT_MS);

  if (data != origData) {
    return false;
  }
  else {
    return true;
  }
}

bool writeData(byte data, bool validate) {
  byte origData = data;

  setDataPinsToWrite();

  for (unsigned int pin = EEPROM_D0; pin <= EEPROM_D7; pin += 1) {
    digitalWrite(pin, data & 1);
    data = data >> 1;
  }

  pulseWrite();
  if (validate && !validateData(origData)) {
    return false;
  }
  return true;
}

void printLine(const char *line) {
  Serial.println(line);
  oled.println(line);
}

void printData(unsigned int base, byte *data) {
  char buf0[20];
  char buf1[20];
  char buf2[20];

  sprintf(buf0, "Address: %04x", base);
  sprintf(buf1, "%02x%02x %02x%02x %02x%02x %02x%02x", data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
  sprintf(buf2, "%02x%02x %02x%02x %02x%02x %02x%02x", data[8], data[9], data[10], data[11], data[12], data[13], data[14], data[15]);

  printLine(buf0);
  printLine(buf1);
  printLine(buf2);

}

void printContents(unsigned int lastByte) {
  for (unsigned int base = 0; base < lastByte; base += 16) {
    byte data[16];

    for (unsigned int offset = 0; offset <= 15; offset += 1) {
      setCurrentAddress(base + offset);
      data[offset] = readData();
    }

    printData(base, data);
  }
}

void setupEXT() {
  pinMode(EXT_RX, INPUT);
  pinMode(EXT_TX, OUTPUT);

  ext_Serial.begin(9600);
}

int writeTestData(unsigned int finalAddress, unsigned int fakeDataOffset) {
  disableSDP();

  unsigned int errorCount = 0;
  unsigned int successCount = 0;
  unsigned int lastDataWritten = 0;
  unsigned int lastAddressWritten = 0;

  char bufBytes[20];
  sprintf(bufBytes, "Writing %u bytes", finalAddress + 1);
  printLine(bufBytes);

  for (unsigned int address = 0; address <= finalAddress; ++address) {
    lastAddressWritten = address;
    lastDataWritten = (address + fakeDataOffset) % 255;

    setCurrentAddress(address);
    if (writeData(lastDataWritten, true)) {
      successCount++;
    } else {
      errorCount++;
      break;
    }

    if (successCount % 256 == 0) {
      char bufBytesWritten[35];
      sprintf(bufBytesWritten, "P: %u of %u", successCount, finalAddress + 1);
      printLine(bufBytesWritten);
    }
  }

  if (errorCount == 0) {
    char buf[20];
    sprintf(buf, "S:%u, E:%u", successCount, errorCount);
    printLine(buf);
  }
  else {
    printLine("Error writing data");

    char buf[20];
    sprintf(buf, "A: %04x, D: %02x", lastAddressWritten, lastDataWritten);
    printLine(buf);
    printLine("Is SDP enabled?");
  }

  return errorCount;
}

void printSelectedEEPROM() {
  char buf[30];
  sprintf(buf, "%s", (selectedEEPROM == LOW) ? S_EEPROM_UPPER28 : S_EEPROM_LOWER24);
  printLine(buf);
}

void checkAndSetSelectedEEPROM() {
  int selected = digitalRead(EEPROM_CE);

  if (selected == selectedEEPROM) return;

  selectedEEPROM = selected;

  printSelectedEEPROM();
}

void ext_printReceivedBytes(int startIndex) {
  char buf[3 + ext_bytesRead * 2];
  char *p = buf;  // location to add to string
  sprintf(buf, "RX: ");

  for (int i = startIndex; i < ext_bytesRead; ++i) {
    p = buf + 3 + ((i - startIndex) * 2);  // start of string plus 4 chars per bytet
    sprintf(p, "%02x", ext_bytesBuf[i]);
  }
  
  printLine(buf);
}

// Ok, here's what a typical received file comm would look like:
// Very first packet received:
//   Byte 0:         Message Header -> Message Type 1, Number of Bytes
//   Bytes 1 and 2:  1's complement checksum bytes, lo first
//   Bytes 3 and 4:  address to write the ROM data to.
//   Bytes 5+:       rest of bytes
// Subsequent bytes:
//   Byte 0:         Message Header -> Message Type 2, Number of Bytes
//   Bytes 1 and 2:  1's complement checksum bytes, lo first
//   Bytes 3+:       rest of bytes
// Last packet received:
//   Byte 0:         Message Header -> Message Type 3, Number of Bytes = 0
//   Bytes 1 and 2:  1's complement checksum bytes, lo first
// Regarding error states:
//   We throw away bytes we receive while we are in an error
// Returns state after processing
State ext_processReceivedBytes() {
  int startIndex = 0;
  if (ext_messageType == 1) {
    // we're processing the very first bytes
    if (ext_bytesRead < 2) {
      printLine("ERR: Read less than 2 bytes on start of byte array. Expected start address.");
      return ERROR;
    }

    currentAddress = (ext_bytesBuf[1] << 8) | ext_bytesBuf[0];
    startAddress = currentAddress;

    char buf[20];
    sprintf(buf, "A: %04x", currentAddress);
    printLine(buf);

    startIndex = 2;
  }
  else if(ext_messageType == 3) {
    printLine(S_OK);
    return DONE;
  }

  ext_printReceivedBytes(0);

  for (int i = startIndex; i < ext_bytesRead; ++i) {
    setCurrentAddress(currentAddress);
    unsigned int data = ext_bytesBuf[i];
    bool success = writeData(data, true);

    if (!success) {
      printLine(S_WRITE_EEPROM_ERROR);

      char buf[20];
      sprintf(buf, "D: %02x A: %04x ST: E", data, currentAddress);
      printLine(buf);

      return ERROR;
    }
    ++currentAddress;
  }

  char buf[30];
  sprintf(buf, "ROM St:%04x Next:%04x", startAddress, currentAddress);
  printLine(buf);

  return AWAITING_NEXT_PACKET;
}

// To validate the checksum
//   Add the checksum we receive, which is actually a 1's complement
//   of the checksum, to the sum of bytes received. Then make sure
//   all bits in our summed value are 1's.
// Note: we only validate the checksum for the bytes, not the header.
//   If the header gets corrupted, we're kinda in trouble and don't
//   do anything about it. TODO implement a solution to that problem.
// Note: we simplified this and handle 12-bit checksums
bool ext_validateChecksum() {
  unsigned int checksumReceived = ((unsigned int)ext_checksumHi << 8) + ext_checksumLo;

  unsigned int checksumCalculated = 0;
  for (unsigned int i = 0; i < ext_bytesRead; ++i) {
    checksumCalculated += ext_bytesBuf[i];
  }

  unsigned int summed = checksumCalculated + checksumReceived;

  if(summed != 4095) {
    printLine(S_CHECKSUM_ERROR);
    char buf[60];
    sprintf(buf, "calc=%d, rcvd=%d, sum=%d", checksumCalculated, checksumReceived, summed);
    printLine(buf);
    return false;
  }

  return true;
}

void decodeHeader(int headerByte) {
  // top 3 bits are message type, bottom 5 bits are number of bytes to read
  ext_bytesExpected = headerByte & 31;
  ext_messageType = headerByte >> 5;
}

void setup() {
  pinMode(SHIFT_DATA, OUTPUT);
  pinMode(SHIFT_CLK, OUTPUT);
  pinMode(SHIFT_LATCH, OUTPUT);

  digitalWrite(EEPROM_OE, HIGH);
  pinMode(EEPROM_OE, OUTPUT);

  digitalWrite(EEPROM_WE, HIGH);
  pinMode(EEPROM_WE, OUTPUT);
  pinMode(EEPROM_CE, INPUT);

  Serial.begin(57600);
  while (!Serial) {
    ;
  }

  Wire.begin();
  Wire.setClock(400000L);

  oled.begin(&Adafruit128x64, OLED_I2C_ADDRESS);
  oled.setFont(Adafruit5x7);
  oled.setScrollMode(SCROLL_MODE_AUTO);
  oled.clear();

  checkAndSetSelectedEEPROM();

  disableSDP();

  ext_resetPipe();

  setupEXT();
  printLine(S_AWAITING_DATA);
}

// Get ready for new data. Only call this if you know we are in a good
// state to receive data.
void ext_resetPipe() {
  dataLastReceivedMS = millis();
  ext_messageType = 0;
  ext_bytesExpected = 0;
  ext_bytesRead = 0;
  ext_state = AWAITING_NEXT_PACKET;
}

// Checks to see if too much time has passed while we were expecting data.
// We will reply back to the sender if so.
// Returns true if timeout.
bool ext_checkReplyTimeout() {
  unsigned long currentTimeMS = millis();
  if((currentTimeMS - dataLastReceivedMS) > EXT_DATA_REPLY_TIMEOUT_MS) {
    return true;
  }
  return false;
}

// Send a reply
//  T = Timeout, try again
//  A = ACK
void ext_reply(char reply) {
  dataLastReceivedMS = millis();
  ext_Serial.write(reply);
}

void loop() {
  checkAndSetSelectedEEPROM();

  bool byteAvailable = ext_Serial.available() > 0 ? true : false;
  int byteRead = 0;
  if(byteAvailable) {
    byteRead = ext_Serial.read();
    ++ext_totalBytesRead;
  }

  if(byteAvailable && (ext_state == ERROR || ext_state == DONE)) {
    return; // ignore due to ERROR or DONE state, stay silent so message stays on screen
  }
  else if(byteAvailable && ext_state == AWAITING_NEXT_PACKET) {
    decodeHeader(byteRead);
    if(ext_messageType == 3) {
      ext_state = DONE;
      ext_reply(REPLY_ACK);
      return;
    }
    ext_state = AWAITING_HEADER_CHECKSUM_LO;
    return;
  }
  else if(byteAvailable && ext_state == AWAITING_HEADER_CHECKSUM_LO) {
    ext_checksumLo = byteRead;
    ext_state = AWAITING_HEADER_CHECKSUM_HI;
    return;
  }
  else if(byteAvailable && ext_state == AWAITING_HEADER_CHECKSUM_HI) {
    ext_checksumHi = byteRead;
    ext_state = AWAITING_BYTES;
    return;
  }
  else if(byteAvailable && ext_state == AWAITING_BYTES) {
    ext_bytesBuf[ext_bytesRead] = byteRead;
    ++ext_bytesRead;
    if (ext_bytesRead < ext_bytesExpected) {
      return; // get next byte
    }

    if(!ext_validateChecksum()) {
      ext_resetPipe();
      ext_reply(REPLY_CHECKSUM_ERROR);
      return;
    }

    ext_state = ext_processReceivedBytes();
    if(ext_state == ERROR) {
      return; // errors here are non-recoverable
    }
    else if(ext_state == DONE) {
      return; // nothing to do here
    }

    // if here, we successfully processed everything
    ext_resetPipe();
    ext_reply(REPLY_ACK);

    return;
  }

  if(ext_state > DONE && ext_totalBytesRead > 0 && ext_checkReplyTimeout()) {
    ext_reply(REPLY_TIMEOUT);
    return;
  }

}
