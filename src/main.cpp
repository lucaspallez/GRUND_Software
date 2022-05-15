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
#define LORA_FREQ         (868.0)
#define LORA_SF           (7)

// ======================================================== LoRa Commands ========================================================
#define LORA_ARM              (0xA0)
#define LORA_RESET            (0xFF)

// ======================================================== Global Variables ========================================================
uint32_t Rx_buffer[RH_RF95_MAX_MESSAGE_LEN] = {0};
uint32_t Tx_buffer[RH_RF95_MAX_MESSAGE_LEN] = {0};

RH_RF95 rf95(TE_CS_PIN, TE_INT_PIN);

// ======================================================== Function Prototypes ========================================================
void heartbeat(void);
void LoRa_parse(uint8_t buf[RH_RF95_MAX_MESSAGE_LEN]);
void LoRa_parse_to_interface(uint8_t buf[RH_RF95_MAX_MESSAGE_LEN]);

// ======================================================== Setup ========================================================
void setup() {

  //General Init
  Serial.begin(115200);
  delay(1000);

  pinMode(STATUS_LED_PIN, OUTPUT);
  pinMode(RX_LED_PIN, OUTPUT);
  pinMode(TX_LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  // LoRa Init
  rf95.init();
    //Serial.println("init failed"); 
  //Serial.println("Init Sucess!");
  rf95.setTxPower(12, false);
  rf95.setFrequency(LORA_FREQ);
  rf95.setSpreadingFactor(LORA_SF);

  digitalWrite(STATUS_LED_PIN, HIGH);
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
      //Serial.print("RSSI: ");
      //Serial.println(rf95.lastRssi(), DEC);

      //LoRa_parse(buf); // USE FOR PC DEBUG
      LoRa_parse_to_interface(buf); //USE FOR USE WITH GRUND STATION
      digitalWrite(RX_LED_PIN, LOW);
    }
    else
    {
      Serial.println("Rx Error");
    }
  }
  
  //heartbeat();
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

  float float_buffer;

  Serial.print("Timestamp: ");
  memcpy(&float_buffer, &(buf[0]), sizeof(float));
  Serial.println(float_buffer);

  Serial.print("Acceleration X: ");
  memcpy(&float_buffer, &(buf[4]), sizeof(float));
  Serial.print(float_buffer);
  Serial.print(", Y: ");
  memcpy(&float_buffer, &(buf[8]), sizeof(float));
  Serial.print(float_buffer);
  Serial.print(", Z: ");
  memcpy(&float_buffer, &(buf[12]), sizeof(float));
  Serial.print(float_buffer);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  memcpy(&float_buffer, &(buf[16]), sizeof(float));
  Serial.print(float_buffer);
  Serial.print(", Y: ");
  memcpy(&float_buffer, &(buf[20]), sizeof(float));
  Serial.print(float_buffer);
  Serial.print(", Z: ");
  memcpy(&float_buffer, &(buf[24]), sizeof(float));
  Serial.print(float_buffer);
  Serial.println(" rad/s");

  Serial.print("Temperature: ");
  memcpy(&float_buffer, &(buf[28]), sizeof(float));
  Serial.print(float_buffer);
  Serial.println(" degC");

  Serial.print(F("Temperature = "));
  memcpy(&float_buffer, &(buf[32]), sizeof(float));
  Serial.print(float_buffer);
  Serial.println(" *C");

  Serial.print(F("Pressure = "));
  memcpy(&float_buffer, &(buf[36]), sizeof(float));
  Serial.print(float_buffer/1000);
  Serial.println(" Pa");

  Serial.print(F("Approx altitude = "));
  memcpy(&float_buffer, &(buf[40]), sizeof(float));
  Serial.print(float_buffer); /* Adjusted to local forecast! */
  Serial.println(" m");

  Serial.print(F("Vbatt 1 = "));
  memcpy(&float_buffer, &(buf[44]), sizeof(float));
  Serial.print(float_buffer);
  Serial.println(" V");
  
  Serial.print(F("Vbatt 2 = "));
  memcpy(&float_buffer, &(buf[48]), sizeof(float));
  Serial.print(float_buffer);
  Serial.println(" V");

  Serial.print(F("Vbatt Ematch = "));
  memcpy(&float_buffer, &(buf[52]), sizeof(float));
  Serial.print(float_buffer);
  Serial.println(" V");

  Serial.print(F("Current State = "));
  Serial.println(buf[56]);

  Serial.print(F("Ematch (1-4) Status = "));
  Serial.print(buf[57]);
  Serial.print(F(" "));
  Serial.print(buf[58]);
  Serial.print(F(" "));
  Serial.print(buf[59]);
  Serial.print(F(" "));
  Serial.println(buf[60]);

  Serial.print(F("GPS Nb Fix = "));
  memcpy(&float_buffer, &(buf[61]), sizeof(float));
  Serial.print(float_buffer);
  Serial.println(" Sats");

  Serial.print(F("GPS HDOP = "));
  memcpy(&float_buffer, &(buf[65]), sizeof(float));
  Serial.println(float_buffer);

  Serial.print(F("GPS Longitude = "));
  memcpy(&float_buffer, &(buf[69]), sizeof(float));
  Serial.print(float_buffer);
  Serial.println(" *");

  Serial.print(F("GPS latitude = "));
  memcpy(&float_buffer, &(buf[73]), sizeof(float));
  Serial.print(float_buffer);
  Serial.println(" *");

  Serial.print(F("GPS Altitude = "));
  memcpy(&float_buffer, &(buf[77]), sizeof(float));
  Serial.print(float_buffer);
  Serial.println(" m");

  Serial.print(F("GPS Speed = "));
  memcpy(&float_buffer, &(buf[81]), sizeof(float));
  Serial.print(float_buffer);
  Serial.println(" m/s");

  Serial.print(F("RSSI = "));
  memcpy(&float_buffer, &(buf[85]), sizeof(float));
  Serial.print(float_buffer);

  Serial.println("");
}

void LoRa_parse_to_interface(uint8_t buf[RH_RF95_MAX_MESSAGE_LEN]){

  float float_buffer;

  memcpy(&float_buffer, &(buf[0]), sizeof(float));
  Serial.print(float_buffer);
  Serial.print(F(" ; "));
  memcpy(&float_buffer, &(buf[4]), sizeof(float));
  Serial.print(float_buffer);
  Serial.print(F(" ; "));
  memcpy(&float_buffer, &(buf[8]), sizeof(float));
  Serial.print(float_buffer);
  Serial.print(F(" ; "));
  memcpy(&float_buffer, &(buf[12]), sizeof(float));
  Serial.print(float_buffer);
  Serial.print(F(" ; "));

  memcpy(&float_buffer, &(buf[16]), sizeof(float));
  Serial.print(float_buffer);
  Serial.print(F(" ; "));
  memcpy(&float_buffer, &(buf[20]), sizeof(float));
  Serial.print(float_buffer);
  Serial.print(F(" ; "));
  memcpy(&float_buffer, &(buf[24]), sizeof(float));
  Serial.print(float_buffer);
  Serial.print(F(" ; "));

  memcpy(&float_buffer, &(buf[28]), sizeof(float));
  Serial.print(float_buffer);
  Serial.print(F(" ; "));

  memcpy(&float_buffer, &(buf[32]), sizeof(float));
  Serial.print(float_buffer);
  Serial.print(F(" "));

  memcpy(&float_buffer, &(buf[36]), sizeof(float));
  Serial.print(float_buffer);
  Serial.print(F(" ; "));

  memcpy(&float_buffer, &(buf[40]), sizeof(float));
  Serial.print(float_buffer); /* Adjusted to local forecast! */
  Serial.print(F(" ; "));

  memcpy(&float_buffer, &(buf[44]), sizeof(float));
  Serial.print(float_buffer);
  Serial.print(F(" ; "));

  memcpy(&float_buffer, &(buf[48]), sizeof(float));
  Serial.print(float_buffer);
  Serial.print(F(" ; "));

  memcpy(&float_buffer, &(buf[52]), sizeof(float));
  Serial.print(float_buffer);
  Serial.print(F(" ; "));

  Serial.print(buf[56]);
  Serial.print(F(" ; "));

  Serial.print(buf[57]);
  Serial.print(F(" ; "));
  Serial.print(buf[58]);
  Serial.print(F(" ; "));
  Serial.print(buf[59]);
  Serial.print(F(" ; "));
  Serial.print(buf[60]);
  Serial.print(F(" ; "));

  memcpy(&float_buffer, &(buf[61]), sizeof(float));
  Serial.print(float_buffer);
  Serial.print(F(" ; "));

  memcpy(&float_buffer, &(buf[65]), sizeof(float));
  Serial.print(float_buffer);
  Serial.print(F(" ; "));

  memcpy(&float_buffer, &(buf[69]), sizeof(float));
  Serial.print(float_buffer);
  Serial.print(F(" ; "));

  memcpy(&float_buffer, &(buf[73]), sizeof(float));
  Serial.print(float_buffer);
  Serial.print(F(" ; "));

  memcpy(&float_buffer, &(buf[77]), sizeof(float));
  Serial.print(float_buffer);
  Serial.print(F(" ; "));

  memcpy(&float_buffer, &(buf[81]), sizeof(float));
  Serial.print(float_buffer);
  Serial.print(F(" ; "));

  memcpy(&float_buffer, &(buf[85]), sizeof(float));
  Serial.print(float_buffer);

  Serial.println(F(" \r"));
}