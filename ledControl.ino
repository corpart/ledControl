#include <Ethernet.h>
#include <EthernetClient.h>
#include <EthernetServer.h>

#include "Adafruit_TLC5947.h"

#include <SPI.h>

#include <EthernetUdp.h>
#include "mac.h" //local import

/* MUX SELECTOR SWITCH PINS */

#define s0 17
#define s1 16
#define s2 15
#define s3 14

/* communication */
EthernetUDP Udp;
char packetBuffer[UDP_TX_PACKET_MAX_SIZE];
IPAddress ip;
unsigned int localPort = 3333; //UDP Listening port
const unsigned int resetPin = 9; //Ethernet Reset Pin

//would like to replace with function getmac...
byte macDebug[] = {
  0x04, 0xE9, 0xE5, 0x03, 0x5F, 0x44
};
//04:E9:E5:03:5F:10 [Teensy A]
//04:E9:E5:03:5F:44 [Teensy B]

//extern uint8_t mac[6]; // MKif you need to declare, declare it this way

int  i;

/* PANEL */
int currentModule;

/* LED variables */
#define NUM_TLC5974 2
const int LedCount = 16;
const int LedBufferSize = LedCount * 6; // uint_16[3] -> 6 bytes per led
char ledBuffer[LedBufferSize];
#define data   4
#define clock   5
#define latch   6

Adafruit_TLC5947 tlc = Adafruit_TLC5947(NUM_TLC5974, clock, data, latch);

void setup() {
  Serial.begin(9600);

  //initialize LED driver
  tlc.begin();

  //Begin Network Setup

  //Read address from DIP
  currentModule = readDIPAddress();
  ip = IPAddress(192, 168, 0, currentModule + 100); //101-103 are touch sensor. 104-1011 ceiling
  getMAC(); //Get macaddress from Teensy
  resetEthernet();
  Ethernet.begin(mac, ip);
  delay(200);
  Udp.begin(localPort);
  Serial.println(ip);
  Serial.println(Ethernet.localIP());

}

void loop() {

    int packetSize = Udp.parsePacket();
    Serial.println(packetSize);
    if (packetSize == LedBufferSize)  {
        Udp.read(ledBuffer, LedBufferSize);
  //       Serial.print(packetSize);
  //       Serial.print("\t");
  //       Serial.println(ledBuffer);

        for(uint16_t i = 0; i < LedCount; i++) {
          uint16_t j = i * 6; // each color is 6 (bigendian) bytes -> rrggbb
          uint16_t r = (ledBuffer[j] << 8) + ledBuffer[j + 1];
          uint16_t g = (ledBuffer[j + 2] << 8) + ledBuffer[j + 3];
          uint16_t b = (ledBuffer[j + 4] << 8) + ledBuffer[j + 5];

          tlc.setLED(i, r, g, b);
        }

        tlc.write();
  }
}




int readDIPAddress() {

  int address;
  // State of each switch (0 or 1)
  int s0state;
  int s1state;
  int s2state;
  int s3state;


  pinMode(s0, INPUT_PULLUP);
  pinMode(s1, INPUT_PULLUP);
  pinMode(s2, INPUT_PULLUP);
  pinMode(s3, INPUT_PULLUP);
  delay(20);

  s0state = digitalReadFast(s0);
  s1state = digitalReadFast(s1);
  s2state = digitalReadFast(s2);
  s3state = digitalReadFast(s3);

  bitWrite(address, 0, !s0state);
  bitWrite(address, 1, !s1state);
  bitWrite(address, 2, !s2state);
  bitWrite(address, 3, !s3state);


  Serial.print("DIP Address =>  ");
  Serial.println(address);
  //delay(2000);
  //    Serial.println(muxValue);

  return address;

}

void getMAC() {
  delay(1000);

  Serial.println("Reading MAC from hardware...");
  read_mac();

  Serial.print("MAC: ");
  print_mac();
  Serial.println();


  Serial.println("Finished.");

}

void resetEthernet() {

  //set MUX to resetpin channel 6


  Serial.println("Reset Ready");
  digitalWrite(s0, LOW);
  digitalWrite(s1, HIGH);
  digitalWrite(s2, HIGH);
  delay(1000);
  Serial.println("Reset Ethernet");


  pinMode(resetPin, OUTPUT);
  digitalWrite(resetPin, LOW);
  delayMicroseconds(10);
  //delay(1000);
  pinMode(resetPin, INPUT);

  Serial.println("Reset Done");
  delay(1000);
}
