#include <RH_RF95.h>
#include <SPI.h>

#define RFM95_CS 8
#define RFM95_INT 3
#define RFM95_RST 4
#define RF95_FREQ 900.0

RH_RF95 rf95(RFM95_CS, RFM95_INT);

int body;
int wheels;
int driveForward;
int driveSide;
int headDown;
int headSide;
int mecanum;
char radiopacket[100];
String message = "0,0,0,0,0,0";
byte sendLen;

void setup() {
  pinMode(RFM95_RST, OUTPUT);
  pinMode(13, INPUT_PULLUP);
  digitalWrite(RFM95_RST, HIGH);
  Serial.begin(115200);
  delay(100);

  Serial.println("Feather LoRa TX Test!");
  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for "
                   "detailed debug info");
    while (1)
      ;
  }

  Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1)
      ;
  }
  Serial.print("Set Freq to: ");
  Serial.println(RF95_FREQ);
  rf95.setTxPower(23, false);
}

int16_t packetnum = 0; // packet counter, we increment per xmission
void loop() {
  body = analogRead(A0);
  body = map(body, 0, 1023, 0, 99);

  mecanum = digitalRead(13);

  wheels = analogRead(A1);
  wheels = map(wheels, 0, 1023, 0, 99);

  driveForward = analogRead(A2);
  driveForward = map(driveForward, 0, 1023, 0, 99);

  driveSide = analogRead(A3);
  driveSide = map(driveSide, 0, 1023, 0, 99);

  headDown = analogRead(A4);
  headDown = map(headDown, 0, 1023, 0, 99);

  headSide = analogRead(A5);
  headSide = map(headSide, 0, 1023, 0, 99);

  message = String(body) + "," + String(wheels) + "," + String(driveForward) +
            "," + String(driveSide) + "," + String(headDown) + "," +
            String(headSide) + "," + String(mecanum) + ",";
  Serial.println(message);

  message.toCharArray(radiopacket, 100);
  sendLen = strlen(radiopacket);

  rf95.send((uint8_t *)radiopacket, sendLen);
  rf95.waitPacketSent();

  delay(10);
}
