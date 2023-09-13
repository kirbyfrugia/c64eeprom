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
#define c64_RX A0
#define C64_TX A3

#define C64_DATA_TIMEOUT_MS 10000


// Wiring for OLED
// A4 - SDA, A5 - SCL

// Wiring for Commodore 64 serial
// A + N = Ground
// B + C = RX (64 side) <- TX (A3, Arduino side)
// M = TX (64 side) -> RX (A0, Arduino side)

#define OLED_I2C_ADDRESS 0x3C

SSD1306AsciiWire oled;

SoftwareSerial c64_Serial(c64_RX, C64_TX);

unsigned int startAddress = 0;
unsigned int currentAddress = 0;
int selectedEEPROM = -1;

const char * S_EEPROM_UPPER28 = "Upper EEPROM (28pin)";
const char * S_EEPROM_LOWER24 = "Lower EEPROM (24pin)";
const char * S_AWAITING_DATA = "Awaiting data...";
const char * S_CHECKSUM_ERROR = "Checksum error";
const char * S_WRITE_EEPROM_ERROR = "Error writing EEPROM";
const char * S_OK = "OK";

unsigned int c64_currentHeaderByte = 0;
unsigned int c64_messageType = 0;
unsigned int c64_bytesExpected = 0;
unsigned int c64_bytesRead = 0;

unsigned int c64_checksumLo = 0;
unsigned int c64_checksumHi = 0;

bool c64_acceptingData = false;

unsigned long dataLastReceivedMS = 0; // how long it has been since we received any data at all

int c64_bytesBuf[16];
int c64_headerBuf[4];

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

void setupC64() {
  pinMode(c64_RX, INPUT);
  pinMode(C64_TX, OUTPUT);

  c64_Serial.begin(1200);
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

void c64_printReceivedBytes(int startIndex) {
  char buf[3 + c64_bytesRead * 2];
  char *p = buf;  // location to add to string
  sprintf(buf, "RX: ");

  for (int i = startIndex; i < c64_bytesRead; ++i) {
    p = buf + 3 + ((i - startIndex) * 2);  // start of string plus 4 chars per bytet
    sprintf(p, "%02x", c64_bytesBuf[i]);
  }
  
  printLine(buf);
}

// Ok, here's what a typical received file comm would look like:
// Very first bytes received:
//   Byte 0:         Message Header -> Message Type 1, Number of Bytes
//   Bytes 1 and 2:  1's complement checksum bytes, lo first
//   Bytes 3 and 4:  address to write the ROM data to.
//   Bytes 5+:       rest of bytes
// Subsequent bytes:
//   Byte 0:         Message Header -> Message Type 2, Number of Bytes
//   Bytes 1 and 2:  1's complement checksum bytes, lo first
//   Bytes 3+:       rest of bytes
// Note: we don't detect the end of writes other than through a timeout.
//   We probably should.
// Regarding error states:
//   We throw away bytes we receive while we are in an error
//   state (c64_acceptingData is false) unless we get a brand new byte array
//   with message type 1. This allows us to continue after a failure.
// Returns true if successfully processed
bool c64_processReceivedBytes() {
  int startIndex = 0;
  if (c64_messageType == 1) {
    // we're processing the very first bytes
    if (c64_bytesRead < 2) {
      c64_acceptingData = false;      
      printLine("ERR: Read less than 2 bytes on start of byte array. Expected start address.");
      return false;
    }

    currentAddress = (c64_bytesBuf[1] << 8) | c64_bytesBuf[0];
    startAddress = currentAddress;

    char buf[20];
    sprintf(buf, "A: %04x", currentAddress);
    printLine(buf);

    startIndex = 2;
  }

  c64_printReceivedBytes(0);

  for (int i = startIndex; i < c64_bytesRead; ++i) {
    setCurrentAddress(currentAddress);
    unsigned int data = c64_bytesBuf[i];
    bool success = writeData(data, true);

    if (!success) {
      printLine(S_WRITE_EEPROM_ERROR);

      char buf[20];
      sprintf(buf, "D: %02x A: %04x ST: E", data, currentAddress);
      printLine(buf);

      c64_acceptingData = false;
      return false;
    }
    ++currentAddress;
  }

  char buf[30];
  sprintf(buf, "ROM St:%04x End:%04x", startAddress, currentAddress);
  printLine(buf);

  return true;
}

// To validate the checksum
//   Add the checksum we receive, which is actually a 1's complement
//   of the checksum, to the sum of bytes received. Then make sure
//   all bits in our summed value are 1's.
// Note: we only validate the checksum for the bytes, not the header.
//   If the header gets corrupted, we're kinda in trouble and don't
//   do anything about it. TODO implement a solution to that problem.
bool c64_validateChecksum() {
  unsigned int checksumReceived = ((unsigned int)c64_checksumHi << 8) + c64_checksumLo;

  unsigned int checksumCalculated = 0;
  for (unsigned int i = 0; i < c64_bytesRead; ++i) {
    checksumCalculated += c64_bytesBuf[i];
  }

  unsigned int summed = checksumCalculated + checksumReceived;

  // this feels dumb, but since we don't know for sure how big the checksum might be,
  // we find out the number of bits we need to check based on how big of a sum we have.
  // As long as we're doing the bit check against an integer bigger than our sum,
  // we should be ok.
  unsigned int numBits = 0;
  unsigned int result = 1;
  do {
    if (result <= summed) {
      ++numBits;
      result = result << 1;
    }
    else {
      break;
    }
  } while (true);

  bool checksumError = false;
  unsigned int checked = summed;
  // check that all bits are 1
  for (int i = 0; i < numBits; ++i) {
    if ((checked & 1) == 0) {
      checksumError = true;
      c64_acceptingData = false;
      break;
    }
    checked = checked >> 1;
  }

  if (checksumError) {
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
  c64_bytesExpected = headerByte & 31;
  c64_messageType = headerByte >> 5;
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

  c64_resetPipe();

  setupC64();
  printLine(S_AWAITING_DATA);
}

// Get ready for new data. Only call this if you know we are in a good
// state to receive data.
void c64_resetPipe() {
  dataLastReceivedMS = millis();
  c64_messageType = 0;
  c64_bytesExpected = 0;
  c64_bytesRead = 0;
  c64_currentHeaderByte = 0;
  c64_acceptingData = true;

}

// if we got some data but it's been too long since we got the next bit of data
// then let's clean everything up and start listening for new data.
void c64_checkTimeoutAndUpdateState() {
  unsigned long currentTimeMS = millis();

  if((c64_messageType != 0) && (currentTimeMS - dataLastReceivedMS) > C64_DATA_TIMEOUT_MS) {
    c64_resetPipe();
    printLine(S_AWAITING_DATA);
  }
}

void loop() {
  checkAndSetSelectedEEPROM();
  c64_checkTimeoutAndUpdateState();

  // IF THE C64 HAS DATA THEN PRINT IT HERE
  if (c64_Serial.available() > 0) {
    dataLastReceivedMS = millis();
    int byteRead = c64_Serial.read();

    // if we're in an error state, we'll just throw away anything we receive until we time out
    if (!c64_acceptingData) {
      return;
    }

    if (c64_currentHeaderByte == 0) {
      decodeHeader(byteRead);
      ++c64_currentHeaderByte;
    } else if (c64_currentHeaderByte == 1) {
      c64_checksumLo = byteRead;
      ++c64_currentHeaderByte;
    } else if (c64_currentHeaderByte == 2) {
      c64_checksumHi = byteRead;
      ++c64_currentHeaderByte;
    } else {
      // if here, we're actually reading the byte array
      c64_bytesBuf[c64_bytesRead] = byteRead;

      ++c64_bytesRead;

      if (c64_bytesRead == c64_bytesExpected) {
        if(!c64_validateChecksum()) {
          return;
        }
        if(!c64_processReceivedBytes()) {
          return;
        }

        c64_resetPipe(); // wait for data again since this was successful.
      }
    }
  }

}
