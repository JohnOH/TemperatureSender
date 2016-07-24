//----------------------------------------------------------------------------------------------------------------------
// TinyTX - An ATtiny84 and RFM12B Wireless Temperature Sensor Node
// By Nathan Chantrell. For hardware design see http://nathan.chantrell.net/tinytx
//
// Using the Dallas DS18B20 temperature sensor
//
// Licenced under the Creative Commons Attribution-ShareAlike 3.0 Unported (CC BY-SA 3.0) licence:
// http://creativecommons.org/licenses/by-sa/3.0/
//
// Requires Arduino IDE with arduino-tiny core: http://code.google.com/p/arduino-tiny/
// and small change to OneWire library, see: http://arduino.cc/forum/index.php/topic,91491.msg687523.html#msg687523
//----------------------------------------------------------------------------------------------------------------------

#include <OneWire.h> // http://www.pjrc.com/teensy/arduino_libraries/OneWire.zip
//#include <DallasTemperature.h> // http://download.milesburton.com/Arduino/MaximTemperature/DallasTemperature_LATEST.zip
// The above DallasTemperature code is huge, 3KB - gone the longer way around
///////////////////////////////////////////////////////////////////////////////
#define RF69_COMPAT      1   // define this to use the RF69 driver i.s.o. RF12 
//                           // The above flag must be set similarly in RF12.cpp
//                           // and RF69_avr.h
///////////////////////////////////////////////////////////////////////////////
#include <JeeLib.h> // https://github.com/jcw/jeelib
#include <avr/sleep.h>
#include <avr/eeprom.h>
#include <util/crc16.h>
#define crc_update      _crc16_update

ISR(WDT_vect) { Sleepy::watchdogEvent(); } // interrupt handler for JeeLabs Sleepy power saving

#define NodeID 4          // RF12 node ID in the range 1-30
#define group 212       // RF12 Network group
#define band RF12_868MHZ  // Band of RFM69CW module

#define ACK_TIME         20  // number of milliseconds - to wait for an ack
#define RETRY_LIMIT      1

#define ONE_WIRE_BUS 10   // DS18B20 Temperature sensor is connected on D10/ATtiny pin 13
#define ONE_WIRE_POWER 9  // DS18B20 Power pin is connected on D9/ATtiny pin 12

#define BASIC_PAYLOAD_SIZE 14
byte payloadSize = BASIC_PAYLOAD_SIZE;
byte commandResponse = false;

OneWire ds(ONE_WIRE_BUS); // Setup a oneWire instance

// DallasTemperature sensors(&oneWire); // Pass our oneWire reference to Dallas Temperature

typedef struct {
      byte command;         // Last command received in ACK
      byte badCRC:  4;      // Running count of CRC mismatches
      byte packetType:  4;  // High order packet type bits
      byte attempts: 4;     // transmission attempts
      byte count: 4;        // packet count
      byte goodNoiseFloor;  // Noise floor allowed transmit
      byte failNoiseFloor;  // Noise floor preventing transmit
      byte failNoiseCount;
      byte rxRssi;          // RSSI of received ACK's
      signed int rxFei;     
      unsigned int temp;    // Temperature reading
      unsigned int supplyV; // Supply voltage
      char messages[64 - BASIC_PAYLOAD_SIZE];
 } packet;
 static packet payload;
 
typedef struct {
    byte start;
    byte txAllowThreshold;
    byte txPower;
    byte rssiThreshold;    
    word crc;
} eeprom;
static eeprom settings;

static byte sendACK() {
      payload.count++;
      for (byte t = 1; t <= RETRY_LIMIT; t++) {  
          delay(t * 10);                   // Increasing the gap between retransmissions
          payload.attempts = t;
          rf12_sleep(RF12_WAKEUP);
          
          RF69::rssiThreshold = settings.rssiThreshold;
          if (rf12_recvDone()) {
              // Serial.print("Discarded: ");             // Flush the buffer
              for (byte i = 0; i < 8; i++) {
                  showByte(rf12_buf[i]);
                  rf12_buf[i] = 0xFF;                     // Paint it over
                  printOneChar(' ');
              }
              // Serial.println();
              // Serial.flush(); 
          }


          while (!(rf12_canSend())) {
            // Serial.print("Airwaves Busy");
            Sleepy::loseSomeTime((50)); 
          }
          
          // Serial.println("TX Start");
          // Serial.flush();
          rf12_sendStart(RF12_HDR_ACK, &payload, payloadSize);
          rf12_sendWait(1);
          rf12_recvDone();
          byte acked = waitForAck(t); // Wait for increasingly longer time for the ACK
          if (acked) {
              payload.rxRssi = rf12_rssi;
              payload.rxFei = rf12_fei;
              payloadSize = BASIC_PAYLOAD_SIZE;   // Packet was ACK'ed by someone
              commandResponse = false;
              payload.packetType = 0;                         
              for (byte i = 0; i < 6; i++) {
                  showByte(rf12_buf[i]);
                  printOneChar(' ');
              }
              // Serial.print("RX threshold:");
              // Serial.println(RF69::rssiThreshold);
              if (rf12_buf[2] > 0) {                          // Non-zero length ACK packet?
                  payload.packetType = 1;
                  if (payload.command == rf12_buf[3]) {
                      payload.command = 0;
                      return t;                         
                  }
                  payload.command = rf12_buf[3];
                  // Serial.print("Command=");
                  // Serial.println(rf12_buf[3]);
                  commandResponse = true;

                  if ((rf12_len + 5) > sizeof payload.messages) rf12_len = (sizeof payload.messages - 5); 
                  for (byte i = 0; i < (rf12_len + 5); i++) {
                      payload.messages [i] = rf12_buf[i];     // Return command stream with next packet
                      payloadSize = BASIC_PAYLOAD_SIZE + (rf12_len + 5);
                  }
              
                  switch (rf12_buf[3]) {
                      case 1: // Increase transmit power by 1dB
                          (++settings.txPower) & 31;
                          RF69::control(0x91, (settings.txPower | 0x80)); // pa0 assumed
                          break;
                      case 2: // Reduce transmit power by 1dB
                          (--settings.txPower) & 31;
                          RF69::control(0x91, (settings.txPower | 0x80)); // pa0 assumed
                          break;
                      case 3: // Increase RX threshold by 0.5dB
                          (++settings.rssiThreshold);
                          break;
                      case 4: // Reduce RX threshold by 0.5dB
                          (--settings.rssiThreshold);
                          break;
                      case 99:
                          // Serial.println("Saving settings to eeprom");
                          saveSettings();
                          break;      
                      default:
                          if (rf12_buf[3] > 4 && rf12_buf[3] < 32) {
                              settings.txPower = rf12_buf[3] & 31;
                              RF69::control(0x91, (settings.txPower | 0x80)); // pa0 assumed
                              break;
                          }
                          if (rf12_buf[3] > 99 && rf12_buf[3] < 255) {
                              settings.txAllowThreshold = rf12_buf[3];
                              payload.failNoiseFloor = 255;
                              payload.failNoiseCount = 0;
                              break;  
                          }
                          // Serial.println("Unknown Command");
                          break;

                      } // end switch
              } // rf12_buf[2]
              return t;
          } // acked
      }
  return 0;
} // sendACK

static byte waitForAck(byte t) {
    MilliTimer ackTimer;
    while (!ackTimer.poll(ACK_TIME + t)) {
        if (rf12_recvDone()) {
            rf12_sleep(RF12_SLEEP);
            // Serial.print((ACK_TIME + t) - ackTimer.remaining());
            // Serial.print("ms RX ");
            
            if (rf12_crc == 0) {                          // Valid packet?
                // see http://talk.jeelabs.net/topic/811#post-4712
                if (rf12_hdr == (/*RF12_HDR_DST*/ 0x40 | RF12_HDR_CTL | NodeID)) {
                    // Serial.print("ACK ");
                    return 1;            
                } else {
                    // Serial.print("Unmatched: ");      // Flush the buffer
                    for (byte i = 0; i < 8; i++) {
                        showByte(rf12_buf[i]);
                        rf12_buf[i] = 0xFF;              // Paint it over
                        printOneChar(' ');
                    }
                }
            } else {
                // Serial.print("Bad CRC");
                payload.badCRC++;
            }
            // Serial.println(); // Serial.flush();           
        }
//        set_sleep_mode(SLEEP_MODE_IDLE);   // Wait a while for the reply?
//        sleep_mode();
    }
//    printOneChar(' ');

    // Serial.print(ACK_TIME + t);
    // Serial.println("ms, Timeout");
    // Serial.flush();

    return 0;
} // waitForAck

static void printOneChar (char c) {
    // Serial.print(c);
}

static void showNibble (byte nibble) {
    char c = '0' + (nibble & 0x0F);
    if (c > '9')
        c += 7;
    // Serial.print(c);
}

static void showByte (byte value) {
//    if (config.output & 0x1) {
        showNibble(value >> 4);
        showNibble(value);
//    } else
//        // Serial.print((word) value, DEC);
}

#define SETTINGS_EEPROM_ADDR ((uint8_t*) 0x00)

#if DEBUG
#define D(x) x
#else
#define D(x)
#endif

static void saveSettings () {
    settings.start = ~0;
    settings.crc = calcCrc(&settings, sizeof settings - 2);
    // this uses 170 bytes less flash than eeprom_write_block(), no idea why
    for (byte i = 0; i < sizeof settings; ++i) {
        byte* p = &settings.start;
        if (eeprom_read_byte(SETTINGS_EEPROM_ADDR + i) != p[i]) {
            eeprom_write_byte(SETTINGS_EEPROM_ADDR + i, p[i]);
        }
    }
} // saveSettings

static void loadSettings () {
    uint16_t crc = ~0;
    // eeprom_read_block(&settings, SETTINGS_EEPROM_ADDR, sizeof settings);
    // this uses 166 bytes less flash than eeprom_read_block(), no idea why
    for (byte i = 0; i < sizeof settings; i++) {
        ((byte*) &settings)[i] = eeprom_read_byte(SETTINGS_EEPROM_ADDR + i);
        crc = crc_update(crc, ((byte*) &settings)[i]);
    }
    // Serial.print("Settings CRC ");
    if (crc) {
        // Serial.println("is bad, defaulting");
        // Serial.println(crc, HEX);
        settings.txAllowThreshold = 160;    // Default to -80dB transmit permit threshold
        settings.txPower = 22;              // TX power
        settings.rssiThreshold = 200;         // RX Threshold
    } else {
        // Serial.println("is good");
    }
} // loadSettings

static word calcCrc (const void* ptr, byte len) {
    word crc = ~0;
    for (byte i = 0; i < len; ++i)
        crc = _crc16_update(crc, ((const byte*) ptr)[i]);
    return crc;
} // calcCrc

//--------------------------------------------------------------------------------------------------
// Read current supply voltage
//--------------------------------------------------------------------------------------------------
 long readVcc() {
   bitClear(PRR, PRADC); ADCSRA |= bit(ADEN); // Enable the ADC
   long result;
   // Read 1.1V reference against Vcc
   #if defined(__AVR_ATtiny84__) 
    ADMUX = _BV(MUX5) | _BV(MUX0); // For ATtiny84
   #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);  // For ATmega328
   #endif 
   delay(2); // Wait for Vref to settle
   ADCSRA |= _BV(ADSC); // Convert
   while (bit_is_set(ADCSRA,ADSC));
   result = ADCL;
   result |= ADCH<<8;
   result = 1126400L / result; // Back-calculate Vcc in mV
   ADCSRA &= ~ bit(ADEN); bitSet(PRR, PRADC); // Disable the ADC to save power
   return result;
} 
//########################################################################################################################

void setup() {
  delay(100);          // Delay on startup to avoid ISP/RFM12B interference.
  // Serial.begin(115200);
  // Serial.println("115200");
  
  loadSettings();                       // Restore settings from eeprom
  payload.failNoiseFloor = 255;

  rf12_initialize(NodeID, band, group, 1600); // Initialize RFM12 with settings defined above 
  RF69::control(0x91, ((settings.txPower & 31) | 0x80)); // pa0 assumed
  rf12_sleep(RF12_SLEEP);                        // Put the RFM12 to sleep

//  pinMode(ONE_WIRE_POWER, OUTPUT);      // set power pin for DS18B20 to output
//  digitalWrite(ONE_WIRE_POWER, HIGH);   // Power up the DS18B20
  Sleepy::loseSomeTime((100 + 16));
  // Serial.flush();
 
  PRR = bit(PRTIM1); // only keep timer 0 going
  
  ADCSRA &= ~ bit(ADEN); bitSet(PRR, PRADC); // Disable the ADC to save power

/// Configure the DS18B20 ///
  ds.reset();               // Set for 12 bit measurements //
  ds.skip();                // Next command to all devices
  
  ds.write(0x4E);           // Write to Scratch Pad
  ds.write(0x7F);           // Set T(h)
  ds.write(0x80);           // Set T(l)
  ds.write(0x7F);           // Set Config 12 bit

/* Required if a new DS18B20 is installed.
// Write to DS18B20 eeprom
  ds.reset();
  ds.skip();                // Next command to all devices
  ds.write(0x48);           // Set Config
  Sleepy::loseSomeTime((100 + 16));// Wait for copy to complete
//  digitalWrite(ONE_WIRE_POWER, LOW);    // Power down the DS18B20    
*/
}

unsigned int getTemp(byte* sensor) {
  byte i;
  byte present = 0;
  byte data[12]; 

  ds.reset();
  ds.skip();                // Next command to all devices
//  ds.select(sensor);    
  ds.write(0xBE);                      // Request Scratchpad

  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
  }
  
// convert the data to actual temperature
  long int raw = (data[1] << 8) | data[0];
// Assuming 12 bit precision
  raw = raw * 100;
  raw = raw >> 4;
  return (raw); // return t*100
}

void loop() {
  
  digitalWrite(ONE_WIRE_POWER, HIGH); // turn DS18B20 sensor on
  Sleepy::loseSomeTime(50); 
  ds.reset();
  ds.skip();                                                // Next command to all devices
  ds.write(0x44);                                           // Start all temperature conversions.
  Sleepy::loseSomeTime((750));                              // Wait for the data to be available

  unsigned int temp = getTemp(0);
  payload.temp = temp;
  digitalWrite(ONE_WIRE_POWER, LOW); // turn DS18B20 off
  
  payload.supplyV = readVcc(); // Get supply voltage
  // Serial.print("Voltage=");
  // Serial.println(payload.supplyV);
  sendACK();
  // Serial.flush();
  if(!(commandResponse)) Sleepy::loseSomeTime(59000);
  else Sleepy::loseSomeTime((1000));

}
