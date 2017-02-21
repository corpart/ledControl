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
unsigned int localPort = 8889; //UDP Listening port
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
#define NUM_TLC5974 3 // How many boards do you have chained?
const int TotalLeds = NUM_TLC5974*8;
const int LedBufferSize = TotalLeds*3*4; //3 for RGB, 4 for 12 bit color
char ledBuffer[LedBufferSize];
char lastLedBuffer[LedBufferSize];
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
    if (packetSize > 0)  {
        Udp.read((char*)ledBuffer, packetSize);
  //      Serial.print(packetSize);
  //      Serial.print("\t");
        Serial.println(ledBuffer);

        /* packet comes in the form:
         * rrrrggggbbbbrrrrggggbbbb.. 
         * For example:
         * 409500004095409500000000... would send purple to first led, red to second
         * 
         */
  
        // bytes vs. ascii bitshift
        //while(Serial.available() < 2); //wait until there are two bytes in the buffer
        //
        //
        //MAP = Serial.read() << 8 ;   //read MSB into MAP
        //MAP += Serial.read();  
    
        for(int i=-1; i< packetSize/12; i++) { //packetSize/12
          String rr =String(ledBuffer[i*12])+String(ledBuffer[i*12+1])+String(ledBuffer[i*12+2])+String(ledBuffer[i*12+3]);
          String gg =String(ledBuffer[i*12+4])+String(ledBuffer[i*12+5])+String(ledBuffer[i*12+6])+String(ledBuffer[i*12+7]);
          String bb =String(ledBuffer[i*12+8])+String(ledBuffer[i*12+9])+String(ledBuffer[i*12+10])+String(ledBuffer[i*12+11]);

          tlc.setLED(uint16_t(i), uint16_t(rr.toInt()), uint16_t(gg.toInt()), uint16_t(bb.toInt()));

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

