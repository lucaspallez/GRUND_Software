#include <Arduino.h>
#include <SPI.h>
#include <RH_RF95.h>

// ======================================================== Defines ========================================================
// Status Indicator Pins
#define STATUS_LED_PIN    (5)
#define RX_LED_PIN        (6)
#define TX_LED_PIN        (7)
#define BUZZER_PIN        (9)

//Interface Pins
#define TE_INT_PIN        (3)
#define TE_CS_PIN         (10)

//LoRa Pins
#define LORA_PACKET_SIZE  (12)
#define LORA_FREQ         (868.0)


// ======================================================== Global Variables ========================================================
uint32_t Rx_buffer[RH_RF95_MAX_MESSAGE_LEN] = {0};
uint32_t Tx_buffer[RH_RF95_MAX_MESSAGE_LEN] = {0};

RH_RF95 rf95(TE_CS_PIN, TE_INT_PIN);

// ======================================================== Function Prototypes ========================================================
void heartbeat(void);
void LoRa_parse(uint8_t buf[RH_RF95_MAX_MESSAGE_LEN]);


// ======================================================== Setup ========================================================
void setup() {

  //General Init
  Serial.begin(9600);
  pinMode(STATUS_LED_PIN, OUTPUT);
  pinMode(RX_LED_PIN, OUTPUT);
  pinMode(TX_LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  // LoRa Init
  while (!Serial) ; // Wait for serial port to be available
  if (!rf95.init())
    Serial.println("init failed"); 
  Serial.println("Init Sucess!");
  rf95.setTxPower(12, false);
  rf95.setFrequency(LORA_FREQ);


}

// ======================================================== Loop ========================================================
void loop()
{
  if (rf95.available())
  {
    // Should be a message for us now   
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    if (rf95.recv(buf, &len))
    {
      digitalWrite(RX_LED_PIN, HIGH);
      //RH_RF95::printBuffer("request: ", buf, len);
      Serial.print("RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);
      LoRa_parse(buf);
      digitalWrite(RX_LED_PIN, LOW);

      // Send a reply
      digitalWrite(TX_LED_PIN, HIGH);
      uint8_t data[] = "ACK";
      rf95.send(data, sizeof(data));
      rf95.waitPacketSent();
      Serial.println("Sent ACK");
      digitalWrite(TX_LED_PIN, LOW);
    }
    else
    {
      Serial.println("Rx Error");
    }
  }
  heartbeat();
  delay(1000);
}

// ======================================================== Functions ========================================================
void heartbeat(void) {
  for (int i = 0; i < 2; i++) {
    digitalWrite(STATUS_LED_PIN, HIGH);
    delay(100);
    digitalWrite(STATUS_LED_PIN, LOW);
    delay(100);
  }
}

void LoRa_parse(uint8_t buf[RH_RF95_MAX_MESSAGE_LEN]){

  //buf = strtol(buf, NULL, DEC);

  Serial.print("Timestamp: ");
  Serial.println(buf[0]);

  Serial.print("Acceleration X: ");
  Serial.print(buf[1]);
  Serial.print(", Y: ");
  Serial.print(buf[2]);
  Serial.print(", Z: ");
  Serial.print(buf[3]);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(buf[4]);
  Serial.print(", Y: ");
  Serial.print(buf[5]);
  Serial.print(", Z: ");
  Serial.print(buf[6]);
  Serial.println(" rad/s");

  Serial.print("Temperature: ");
  Serial.print(buf[7]);
  Serial.println(" degC");

  Serial.print(F("Temperature = "));
  Serial.print(buf[8]);
  Serial.println(" *C");

  Serial.print(F("Pressure = "));
  Serial.print(buf[9]);
  Serial.println(" Pa");

  Serial.print(F("Approx altitude = "));
  Serial.print(buf[10]); /* Adjusted to local forecast! */
  Serial.println(" m");

  Serial.print(F("Vbatt 1 = "));
  Serial.print(buf[11]);
  Serial.println(" V");
  
  Serial.print(F("Vbatt 2 = "));
  Serial.print(buf[12]);
  Serial.println(" V");

  Serial.print(F("Vbatt Ematch = "));
  Serial.print(buf[13]);
  Serial.println(" V");

  Serial.print(F("Current State = "));
  Serial.println(buf[14]);

  Serial.print(F("Ematch (1-4) Status = "));
  Serial.print(buf[15]);
  Serial.print(F(" "));
  Serial.print(buf[16]);
  Serial.print(F(" "));
  Serial.print(buf[17]);
  Serial.print(F(" "));
  Serial.println(buf[18]);

  Serial.print(F("GPS Nb Fix = "));
  Serial.print(buf[19]);
  Serial.println(" Sats");

  Serial.print(F("GPS Longitude = "));
  Serial.print(buf[20]);
  Serial.println(" *");

  Serial.print(F("GPS latitude = "));
  Serial.print(buf[21]);
  Serial.println(" *");

  Serial.print(F("GPS Altitude = "));
  Serial.print(buf[22]);
  Serial.println(" m");

  Serial.print(F("GPS Speed = "));
  Serial.print(buf[23]);
  Serial.println(" km/h");

  Serial.println("");


}
