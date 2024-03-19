#include <Adafruit_MotorShield.h>
#include <RH_RF95.h>
#include <SPI.h>
#include <Servo.h>
#include <Wire.h>

#define RFM95_CS 8
#define RFM95_INT 3
#define RFM95_RST 4
#define RF95_FREQ 900.0

#define EYE_PIN A3

#define WHEELS_SERVO 9
#define BODY_SERVO 10
#define HEAD_NOD_SERVO 11
#define HEAD_SIDE_SERVO 12

RH_RF95 rf95(RFM95_CS, RFM95_INT);

float bodyPosition;
float trackPosition;
float headForward;
float headSide;
float driveSide;
float driveForward;

float bodyPositionIn;
float trackPositionIn;
float headForwardIn;
float headSideIn;
float driveSideIn;
float driveForwardIn;
float omniIn;

float bodyPositionSmooth;
float trackPositionSmooth;
float headForwardSmooth;
float headSideSmooth;
float driveSideSmooth;
float driveForwardSmooth;

float bodyPositionPrev;
float trackPositionPrev;
float headForwardPrev;
float headSidePrev;
float driveSidePrev;
float driveForwardPrev;

float bodyNodFactor;
float headNod;

float speedL;
float speedR;

float motorSpeedL;
float motorSpeedR;

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motorBL = AFMS.getMotor(1);
Adafruit_DCMotor *motorFL = AFMS.getMotor(2);
Adafruit_DCMotor *motorFR = AFMS.getMotor(3);
Adafruit_DCMotor *motorBR = AFMS.getMotor(4);

Servo ServoWheels;
Servo ServoBody;
Servo ServoHeadSide;
Servo ServoHeadNod;

void setup() {
  pinMode(EYE_PIN, OUTPUT);
  analogWrite(EYE_PIN, 230);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  Serial.begin(115200);
  delay(100);

  Serial.println("Feather LoRa RX Test !");
  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    Serial.println("Uncomment ‘#define SERIAL_DEBUG’ in RH_RF95.cpp for detailed debug info");
    while (1)
      ;
  }

  Serial.println("LoRa radio init OK !");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1)
      ;
  }
  Serial.print("Set Freq to : ");
  Serial.println(RF95_FREQ);
  rf95.setTxPower(23, false);
  AFMS.begin();

  ServoHeadSide.attach(HEAD_SIDE_SERVO);
  ServoHeadNod.attach(HEAD_NOD_SERVO);
  ServoBody.attach(BODY_SERVO);
  ServoWheels.attach(WHEELS_SERVO);
}

void loop() {
  handleRadio();

  smoothState();
  controlBody();
  controlDrives();

  delay(20);
}

void handleRadio() {
  if (rf95.available()) {
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    if (rf95.recv(buf, &len)) {
      readComm(buf);
      mapInputs();
      digitalWrite(LED_BUILTIN, HIGH);
    } else {
      Serial.println("Receive failed");
    }
  }
}

void readComm(const uint8_t *mess) {
  byte index = 0;
  const char *ptr = strtok((char *)mess, ",");
  while (ptr != NULL && index < 7) {
    switch (index) {
      case 0: bodyPositionIn = atoi(ptr); break;
      case 1: trackPositionIn = atoi(ptr); break;
      case 2: headForwardIn = atoi(ptr); break;
      case 3: headSideIn = atoi(ptr); break;
      case 4: driveSideIn = atoi(ptr); break;
      case 5: driveForwardIn = atoi(ptr); break;
      case 6: omniIn = atoi(ptr); break;
    }
    index++;
    ptr = strtok(NULL, ",");
  }
}

void mapInputs() {
  bodyPosition = map(bodyPositionIn, 30, 66, 0, 180);
  headForward = map(headForwardIn, 10, 89, 0, 180);
  headSide = map(headSideIn, 0, 99, 0, 180);
  trackPosition = map(trackPositionIn, 30, 66, 0, 180);
  driveSide = map(driveSideIn, 5, 95, 255, -255);
  driveForward = map(driveForwardIn, 5, 95, 255, -255);

  bodyPosition = constrain(bodyPosition, 0, 180);
  headForward = constrain(headForward, 0, 180);
  headSide = constrain(headSide, 2, 180);
  trackPosition = constrain(trackPosition, 5, 180);
  driveSide = constrain(driveSide, -255, 255);
  driveForward = constrain(driveSide, -255, 255);
}

void smoothState() {
  bodyPositionSmooth = (bodyPosition * 0.1) + (bodyPositionPrev * 0.90);
  headForwardSmooth = (headForward * 0.2) + (headForwardPrev * 0.8);
  headSideSmooth = (headSide * 0.2) + (headSidePrev * 0.8);
  trackPositionSmooth = (trackPosition * 0.1) + (trackPositionPrev * 0.90);
  driveSideSmooth = (driveSide * 0.3) + (driveSidePrev * 0.70);
  driveForwardSmooth = (driveForward * 0.3) + (driveForwardPrev * 0.70);

  bodyPositionPrev = bodyPositionSmooth;
  headForwardPrev = headForwardSmooth;
  headSidePrev = headSideSmooth;
  trackPositionPrev = trackPositionSmooth;
  driveSidePrev = driveSideSmooth;
  driveForwardPrev = driveForwardSmooth;
}

void controlBody() {
  bodyNodFactor = map(bodyPositionSmooth, 0, 160, 100, 0);
  bodyNodFactor = constrain(bodyNodFactor, 0, 100);

  headNod = map(headForwardSmooth, 0, 180, -90, 90);
  headNod = headNod * (bodyNodFactor / 100);
  headNod = map(headNod, -90, 90, 50, 130);

  ServoHeadSide.write(headSideSmooth);
  ServoHeadNod.write(headNod);
  ServoBody.write(bodyPositionSmooth);
  ServoWheels.write(trackPositionSmooth);
}

void controlDrives() {
  speedR = driveForwardSmooth + driveSideSmooth;
  speedL = driveForwardSmooth - driveSideSmooth;
  speedR = constrain(speedR, -255, 255);
  speedL = constrain(speedL, -255, 255);

  if (omniIn > 0) {
    if (speedR > 20) {
      motorSpeedR = map(speedR, 20, 255, 150, 255);
      motorBR->run(FORWARD);
      motorFR->run(FORWARD);
      motorBR->setSpeed(motorSpeedR);
      motorFR->setSpeed(motorSpeedR);
    } else if (speedR < -20) {
      motorSpeedR = map(speedR, -20, -255, 150, 255);
      motorBR->run(BACKWARD);
      motorFR->run(BACKWARD);
      motorBR->setSpeed(motorSpeedR);
      motorFR->setSpeed(motorSpeedR);
    } else {
      motorSpeedR = 0;
      motorBR->run(RELEASE);
      motorFR->run(RELEASE);
    }
    if (speedL > 20) {
      motorSpeedL = map(speedL, 20, 255, 150, 255);
      motorBL->run(FORWARD);
      motorFL->run(FORWARD);
      motorBL->setSpeed(motorSpeedL);
      motorFL->setSpeed(motorSpeedL);
    } else if (speedL < -20) {
      motorSpeedL = map(speedL, -20, -255, 150, 255);
      motorBL->run(BACKWARD);
      motorFL->run(BACKWARD);
      motorBL->setSpeed(motorSpeedL);
      motorFL->setSpeed(motorSpeedL);
    } else {
      motorSpeedL = 0;
      motorBL->run(RELEASE);
      motorFL->run(RELEASE);
    }
  } else {
    if (speedR > 20) {
      motorSpeedR = map(speedR, 20, 255, 150, 255);
      motorBR->run(FORWARD);
      motorFL->run(FORWARD);
      motorBR->setSpeed(motorSpeedR);
      motorFL->setSpeed(motorSpeedR);
    } else if (speedR < -20) {
      motorSpeedR = map(speedR, -20, -255, 150, 255);
      motorBR->run(BACKWARD);
      motorFL->run(BACKWARD);
      motorBR->setSpeed(motorSpeedR);
      motorFL->setSpeed(motorSpeedR);
    } else {
      motorSpeedR = 0;
      motorBR->run(RELEASE);
      motorFL->run(RELEASE);
    }

    if (speedL > 20) {
      motorSpeedL = map(speedL, 20, 255, 150, 255);
      motorBL->run(FORWARD);
      motorFR->run(FORWARD);
      motorBL->setSpeed(motorSpeedL);
      motorFR->setSpeed(motorSpeedL);
    } else if (speedL < -20) {
      motorSpeedL = map(speedL, -20, -255, 150, 255);
      motorBL->run(BACKWARD);
      motorFR->run(BACKWARD);
      motorBL->setSpeed(motorSpeedL);
      motorFR->setSpeed(motorSpeedL);
    } else {
      motorSpeedL = 0;
      motorBL->run(RELEASE);
      motorFR->run(RELEASE);
    }
  }
}
