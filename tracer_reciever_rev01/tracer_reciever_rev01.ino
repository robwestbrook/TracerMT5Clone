/**
 * Ping and Pong
 */

 /**
 * PONG = RECEIVER
 * PING = TRANSMITTER
 */

#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"

// Set up nRF24L01 radio on SPI bus plus pins 9 & 10

RF24 radio(9,10);

// sets the role of this unit in hardware.  Connect to GND to be the 'pong' receiver
// Leave open to be the 'ping' transmitter
const int role_pin = 7;

//
// Topology
//

// Radio pipe addresses for the 2 nodes to communicate.
const uint64_t pipes[1] = { 0xF0F0F0F0E1LL };


int ledPin = 2; // added

byte rx[32];

void setup(void)
{
  pinMode(ledPin, OUTPUT); // added
  Serial.begin(9600);
  Serial.println("RF24 Examples Pingpair");
  //
  // Setup and configure rf radio
  //

  radio.begin();

  // optionally, increase the delay between retries & # of retries
  radio.setRetries(15,15);

  // optionally, reduce the payload size.  seems to
  // improve reliability
  radio.setPayloadSize(32);

  // set to lowest data speed
  radio.setDataRate(RF24_250KBPS);

  //
  // Open pipes to other nodes for communication
  //

  radio.openReadingPipe(1,pipes[0]);
  

  //
  // Start listening
  //

  radio.startListening();

  //
  // Dump the configuration of the rf unit for debugging
  //

  radio.printDetails();
}

void loop(void)
{
  
    
    if(radio.available()) {
      radio.read( &rx, sizeof(rx) );
      //printf("Got response %lu, round-trip delay: %lu\n\r",got_time,millis()-got_time);
      //for(int q = 0; q <32; q++) {
        //Serial.println(rx[q], HEX);
      //}
      Serial.println("*********************");
      float battery = to_float(rx, 9);
      Serial.print("Battery Voltage = ");
      Serial.println(battery);
      float pv = to_float(rx, 11);
      Serial.print("Panel Voltage = ");
      Serial.println(pv);
      float load_current = to_float(rx, 15);
      Serial.print("Load Current = ");
      Serial.println(load_current);
      float over_discharge = to_float(rx, 17);
      Serial.print("Over Discharge = ");
      Serial.println(over_discharge);
      float battery_max = to_float(rx, 19);
      Serial.print("Battery Max = ");
      Serial.println(battery_max);
      uint8_t loadOnOff = rx[21];// 21 load on/off
      Serial.print("Load On = ");
      Serial.println(loadOnOff);
      uint8_t loadOverload = rx[22];// 22 overload yes/no
      Serial.print("Load Overload = ");
      Serial.println(loadOverload);
      uint8_t loadShort = rx[23];// 23 load short yes/no
      Serial.print("Load Short = ");
      Serial.println(loadShort);
      uint8_t soc = rx[24];// 24 reserved
      Serial.print("SOC = ");
      Serial.println(soc);
      uint8_t batteryOverload = rx[25];// 25 battery overload
      Serial.print("Battery Overload = ");
      Serial.println(batteryOverload);
      uint8_t batteryOverDischarge = rx[26];// 26 over discharge yes/no
      Serial.print("Battery Over Discharge = ");
      Serial.println(batteryOverDischarge);
      uint8_t full = rx[27];
      Serial.print("Battery Full = ");
      Serial.println(full);
      uint8_t charging = rx[28];
      Serial.print("Battery Charging = ");
      Serial.println(charging);
      int8_t battery_temp = rx[29];
      Serial.print("Battery Temperature = ");
      Serial.println(battery_temp + 30);
      float charge_current = to_float(rx, 30);
      Serial.print("Panel Current = ");
      Serial.println(charge_current);
    } else {
      delay(5000);
    }
  }

/*********************************
**                              **
** FUNCTION TO CONVERT          **
** DATA TO FLOAT                **
**                              **
*********************************/
float to_float(uint8_t* buffer, int offset){
  unsigned short full = buffer[offset+1] << 8 | rx[offset];
  return full / 100.0;
}
