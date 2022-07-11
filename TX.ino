/*
 * The following code is for the mobie node in the setup.
 */

#include <SPI.h>
#include <RH_RF69.h>

/************ Radio Setup ***************/

// Change to 434.0 or other frequency, must match RX's freq!
#define RF69_FREQ 433.0

#if defined (__AVR_ATmega32U4__) // Arduino Micro Wiring 
  #define RFM69_CS      4
  #define RFM69_INT     3
  #define RFM69_RST     2
  #define LED           12
#endif
  
#if defined (__AVR_ATmega328P__)  // Arduino Micro Wiring
  #define RFM69_INT     3 
  #define RFM69_CS      4 
  #define RFM69_RST     2
  #define LED           5
#endif

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

int16_t packetnum = 0;  // packet counter, we increment per xmission
int red = 10;
int yellow = 8;
int button = 6;

void setup() 
{
  Serial.begin(9600);
  //while (!Serial) { delay(1); } // wait until serial console is open, remove if not tethered to computer

  pinMode(button, INPUT_PULLUP);
  pinMode(LED, OUTPUT);  
  pinMode(red, OUTPUT);
  pinMode(yellow, OUTPUT);   
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
  
  pinMode(LED, OUTPUT);
}

void loop() {
  static char sendbuffer[62];
  static int sendlength = 0;
  int reset = 0;

  // SENDING

  if (digitalRead(button) == LOW)
  {
      // On buton press send a capital H, this indicates a trigger, blink yellow LED
      sendbuffer[sendlength] = 'H';
      sendlength++;
      rf69.send((uint8_t *)sendbuffer, sendlength);
      rf69.waitPacketSent();
      Serial.println("Sending a trigger...");
      Blink(yellow,100, 1);

      sendlength = 0; // reset the packet
      }

  // RECEIVING

  // In this section, we'll check with the RFM69HCW to see
  //if it has received any packets:
  // Should be a message for us after the other node is finished with the trigger signal  
  uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

if (rf69.available()) {

  if (rf69.recv(buf, &len)) {
      Serial.print("Got a reply: ");
      Serial.println((char*)buf);
      String x = (char*)buf;

      // Takes rx buffer and converts to string; if string is "Fail" then blink red LED; if "Pass" blink the green LED
      if (x == "Fail") {
        Blink(red, 200, 5); //Blink red LED (FAIL)
        //Serial.println("Sweep Failed");
      }
      else if (x == "Pass"){
        Blink(LED, 200, 3); //Blink green LED (Pass)
        //Serial.println("Sweep Passed");
}
}
}
}
void Blink(byte PIN, byte DELAY_MS, byte loops) {
  for (byte i=0; i<loops; i++)  {
    digitalWrite(PIN,HIGH);
    delay(DELAY_MS);
    digitalWrite(PIN,LOW);
    delay(DELAY_MS);
  }
}
