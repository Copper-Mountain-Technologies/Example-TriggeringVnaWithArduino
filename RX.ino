// rf69 demo tx rx.pde
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messageing client
// with the RH_RF69 class. RH_RF69 class does not provide for addressing or
// reliability, so you should only use RH_RF69  if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example rf69_server.
// Demonstrates the use of AES encryption, setting the frequency and modem 
// configuration

#include <SPI.h>
#include <RH_RF69.h>
#include <SoftwareSerial.h>

/************ Radio Setup ***************/

// Change to 434.0 or other frequency, must match RX's freq!
#define RF69_FREQ 433.0

#if defined (__AVR_ATmega32U4__) // Arduino Micro Wiring
  #define RFM69_CS      4
  #define RFM69_INT     3
  #define RFM69_RST     2
  #define LED           12
#endif
  
#if defined (__AVR_ATmega328P__)  // Arduino Uno Wiring
  #define RFM69_INT     3  // 
  #define RFM69_CS      4  //
  #define RFM69_RST     2  // "A"
  #define LED           5
#endif

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

int16_t packetnum = 0;  // packet counter, we increment per xmission
int triggerPin = 7;
int yellow = 9;
int incomingByte; 
int reset = 0;
const unsigned int MAX_MESSAGE_LENGTH = 12;


void setup() 
{
  
  Serial.begin(9600);
  //while (!Serial) { delay(1); } // wait until serial console is open, remove if not tethered to computer
  Serial.println("Setup");
  pinMode(LED, OUTPUT);    
  pinMode(yellow, OUTPUT); 
  pinMode(triggerPin, OUTPUT);
  digitalWrite(triggerPin, LOW);
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

  // manual reset
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);
  
  if (!rf69.init()) {
    while (1);
  }
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
  // No encryption
  if (!rf69.setFrequency(RF69_FREQ)) {
  }

  // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
  // ishighpowermodule flag set like this:
  rf69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

  // The encryption key has to be the same as the one in the server
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  rf69.setEncryptionKey(key);

}

void loop() {

 static char message[MAX_MESSAGE_LENGTH];
 static unsigned int message_pos = 0;
 
 if (rf69.available()) {
    if (reset =0){
      reset++;
    }
    // Should be a message for us now   
    uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    // Read character buffer and convert it to a string
    if (rf69.recv(buf, &len)) {
      if (!len) return;
      buf[len] = 0;

      String x = (char*)buf;

      if (x == "H") {
        // If rx buffer is "H" set trigger pin to high for some interval and blink led
        TRIG(triggerPin, 10);
        Blink(LED, 100, 1); //Blink green LED
        Serial.println("1");
      }
    }
  }
  if (Serial.available() > 0) {
  // reads the oldest byte in the serial buffer coming from the python script:
  incomingByte = Serial.read();
  // If it's a 1 the limit test failed, send reply back to other node
  if (incomingByte == '1') {
    Serial.println("Arduino received a 1");
    Blink(yellow, 1000, 1);

    uint8_t sendbuffer[] = "Fail";
    rf69.send(sendbuffer, sizeof(sendbuffer));
  }
  // If it's a 0 the limit test passed, send reply back to other node
  else if (incomingByte == '0') {
    Serial.println("Arduino received a 0");
    Blink(LED, 1000, 1);
    uint8_t sendbuffer[] = "Pass";
    rf69.send(sendbuffer, sizeof(sendbuffer));
  }
  else{
    Serial.println("Received neither a 1 or 0");
  }
}
}


void Blink(byte PIN, int DELAY_MS, byte loops) {
  for (byte i=0; i<loops; i++)  {
    digitalWrite(PIN,HIGH);
    delay(DELAY_MS);
    digitalWrite(PIN,LOW);
    delay(DELAY_MS);
  }
}

void TRIG(byte PIN, byte DELAY_MS){
  digitalWrite(PIN, HIGH);
  delay(DELAY_MS);
  digitalWrite(PIN, LOW);
}
