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

#define SERVO_WHEELS 9
#define SERVO_BODY 10
#define SERVO_HEAD_NOD 11
#define SERVO_HEAD_SIDE 12

#define MOTOR_LEFT_BACK 1
#define MOTOR_LEFT_FRONT 2
#define MOTOR_RIGHT_FRONT 3
#define MOTOR_RIGHT_BACK 4

#define DRIVE_STOP 20
#define DRIVE_LIMIT 255

#define MOTOR_MIN 150
#define MOTOR_MAX 255

#define EYE_FADE 0.9 // eye pulse fade factor per tick
#define EYE_THRESHOLD 0.1 // LED cutoff after pulse fades under

RH_RF95 rf95(RFM95_CS, RFM95_INT);

float eyePulse;

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
Adafruit_DCMotor *motorBL = AFMS.getMotor(MOTOR_LEFT_BACK);
Adafruit_DCMotor *motorFL = AFMS.getMotor(MOTOR_LEFT_FRONT);
Adafruit_DCMotor *motorFR = AFMS.getMotor(MOTOR_RIGHT_FRONT);
Adafruit_DCMotor *motorBR = AFMS.getMotor(MOTOR_RIGHT_BACK);

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

  ServoHeadSide.attach(SERVO_HEAD_SIDE);
  ServoHeadNod.attach(SERVO_HEAD_NOD);
  ServoBody.attach(SERVO_BODY);
  ServoWheels.attach(SERVO_WHEELS);
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
      eyePulse = 1.0;
    } else {
      eyePulse = 0.0;
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
  eyePulse *= EYE_FADE;
  digitalWrite(LED_BUILTIN, eyePulse > EYE_THRESHOLD ? HIGH : LOW);

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
  if (omniIn > 0) {
    motorSpeedR = driveMotorPair(motorBR, motorFR, speedR);
    motorSpeedL = driveMotorPair(motorBL, motorFL, speedL);
  } else {
    motorSpeedR = driveMotorPair(motorBR, motorFL, speedR);
    motorSpeedL = driveMotorPair(motorBL, motorFR, speedL);
  }
}

float driveMotorPair(Adafruit_DCMotor *motorA, Adafruit_DCMotor *motorB, float speed) {
  float motorSpeed = 0;
  speed = constrain(speed, -DRIVE_LIMIT, DRIVE_LIMIT);
  if (speed > DRIVE_STOP) {
    motorSpeed = map(speed, DRIVE_STOP, DRIVE_LIMIT, MOTOR_MIN, MOTOR_MAX);
    motorA->run(FORWARD);
    motorB->run(FORWARD);
    motorA->setSpeed(motorSpeed);
    motorB->setSpeed(motorSpeed);
  } else if (speed < -DRIVE_STOP) {
    motorSpeed = map(speed, -DRIVE_STOP, -DRIVE_LIMIT, MOTOR_MIN, MOTOR_MAX);
    motorA->run(BACKWARD);
    motorB->run(BACKWARD);
    motorA->setSpeed(motorSpeed);
    motorB->setSpeed(motorSpeed);
  } else {
    motorA->run(RELEASE);
    motorB->run(RELEASE);
  }
  return motorSpeed;
}
