#include <RH_RF95.h>
#include <SPI.h>

#define RFM95_CS 8
#define RFM95_INT 3
#define RFM95_RST 4
#define RF95_FREQ 900.0

RH_RF95 rf95(RFM95_CS, RFM95_INT);

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

void loop() {
  int body = analogRead(A0);
  int wheels = analogRead(A1);
  int driveForward = analogRead(A2);
  int driveSide = analogRead(A3);
  int headDown = analogRead(A4);
  int headSide = analogRead(A5);
  int mecanum = digitalRead(13);

  body = map(body, 0, 1023, 0, 99);
  wheels = map(wheels, 0, 1023, 0, 99);
  driveForward = map(driveForward, 0, 1023, 0, 99);
  driveSide = map(driveSide, 0, 1023, 0, 99);
  headDown = map(headDown, 0, 1023, 0, 99);
  headSide = map(headSide, 0, 1023, 0, 99);

  String message =
    String(body) + "," +
    String(wheels) + "," +
    String(driveForward) + "," +
    String(driveSide) + "," +
    String(headDown) + "," +
    String(headSide) + "," +
    String(mecanum) + ","; // TODO make trailing "," not necessary
  Serial.println(message);

  char radiopacket[100];
  message.toCharArray(radiopacket, 100);
  byte sendLen = strlen(radiopacket);

  rf95.send((uint8_t *)radiopacket, sendLen);
  rf95.waitPacketSent();

  delay(10);
}
