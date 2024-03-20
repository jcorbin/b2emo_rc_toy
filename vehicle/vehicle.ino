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

#define FADE_EYE 0.9 // eye pulse fade factor per tick
#define FADE_BODY 0.90 // body position smoothing factor
#define FADE_HEAD 0.80 // head position smoothing factor
#define FADE_TRACK 0.90 // track position smoothing factor
#define FADE_DRIVE 0.70 // drive vector smoothing factor

#define DRIVE_STOP 20
#define DRIVE_LIMIT 255

#define MOTOR_MIN 150
#define MOTOR_MAX 255

#define EYE_THRESHOLD 0.1 // LED cutoff after pulse fades under

RH_RF95 rf95(RFM95_CS, RFM95_INT);

float eyePulse;

typedef union vec2 { struct { float x, y; }; float e[2]; } vec2;

float bodyPosition;
float trackPosition;
vec2 head, drive;

float bodyPositionIn;
float trackPositionIn;
vec2 headIn, driveIn;

float bodyPositionSmooth;
float trackPositionSmooth;
vec2 headForward, driveSmooth;

float bodyPositionPrev;
float trackPositionPrev;
vec2 headPrev, drivePrev;

float bodyNodFactor;
float headNod;

vec2 driveSpeed, motorSpeed;

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motorBL = AFMS.getMotor(MOTOR_LEFT_BACK);
Adafruit_DCMotor *motorFL = AFMS.getMotor(MOTOR_LEFT_FRONT);
Adafruit_DCMotor *motorFR = AFMS.getMotor(MOTOR_RIGHT_FRONT);
Adafruit_DCMotor *motorBR = AFMS.getMotor(MOTOR_RIGHT_BACK);

typedef struct driveScheme {
  Adafruit_DCMotor *L[2];
  Adafruit_DCMotor *R[2];
} driveScheme;

const driveScheme omniDriveMode = {{motorBL, motorFL}, {motorBR, motorFR}};
const driveScheme normDriveMode = {{motorBL, motorFR}, {motorBR, motorFL}};
driveScheme driveMode = normDriveMode;

Servo ServoWheels, ServoBody, ServoHeadSide, ServoHeadNod;

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
  control();
  delay(20);
}

void handleRadio() {
  if (rf95.available()) {
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    if (rf95.recv(buf, &len)) {
      readComm(buf);
      eyePulse = 1.0;
    } else {
      eyePulse = 0.0;
      Serial.println("Receive failed");
    }
  }
}

void readComm(const uint8_t *mess) {
  float omniIn = 0;
  byte index = 0;
  const char *ptr = strtok((char *)mess, ",");
  while (ptr != NULL && index < 7) {
    switch (index) {
      case 0: bodyPositionIn = atoi(ptr); break;
      case 1: trackPositionIn = atoi(ptr); break;
      case 2: headIn.y = atoi(ptr); break;
      case 3: headIn.x = atoi(ptr); break;
      case 4: driveIn.x = atoi(ptr); break;
      case 5: driveIn.y = atoi(ptr); break;
      case 6: omniIn = atoi(ptr); break;
    }
    index++;
    ptr = strtok(NULL, ",");
  }

  bodyPosition = constrain(map(bodyPositionIn, 30, 66, 0, 180), 0, 180);
  head.x = constrain(map(headIn.x, 0, 99, 0, 180), 2, 180);
  head.y = constrain(map(headIn.y, 10, 89, 0, 180), 0, 180);
  trackPosition = constrain(map(trackPositionIn, 30, 66, 0, 180), 5, 180);
  drive.x = constrain(map(driveIn.x, 5, 95, 255, -255), -255, 255);
  drive.y = constrain(map(driveIn.y, 5, 95, 255, -255), -255, 255);

  driveMode = omniIn > 0 ? omniDriveMode : normDriveMode;
}

float mix(const float a, const float b, const float p) {
  return a * (1.0 - p) + b * p;
}

vec2 mix(const vec2 a, const vec2 b, const float p) {
  return {mix(a.x, b.x, p), mix(a.y, b.y, p)};
}

void control() {
  bodyPositionSmooth = mix(bodyPosition, bodyPositionPrev, FADE_BODY);
  headSmooth = mix(head, headPrev, FADE_HEAD);
  trackPositionSmooth = mix(trackPosition, trackPositionPrev, FADE_TRACK);
  driveSmooth = mix(drive, drivePrev, FADE_DRIVE);
  bodyPositionPrev = bodyPositionSmooth;
  headPrev = headSmooth;
  trackPositionPrev = trackPositionSmooth;
  drivePrev = driveSmooth;

  eyePulse *= FADE_EYE;
  digitalWrite(LED_BUILTIN, eyePulse > EYE_THRESHOLD ? HIGH : LOW);

  bodyNodFactor = map(bodyPositionSmooth, 0, 160, 100, 0);
  bodyNodFactor = constrain(bodyNodFactor, 0, 100);

  headNod = map(headSmooth.y, 0, 180, -90, 90);
  headNod = headNod * (bodyNodFactor / 100);
  headNod = map(headNod, -90, 90, 50, 130);

  driveSpeed = {
    driveSmooth.y - driveSmooth.x,
    driveSmooth.y + driveSmooth.x
  };

  ServoHeadSide.write(headSmooth.x);
  ServoHeadNod.write(headNod);
  ServoBody.write(bodyPositionSmooth);
  ServoWheels.write(trackPositionSmooth);

  motorSpeed = {
    driveMotorPair(driveMode.L, driveSpeed.e[0]),
    driveMotorPair(driveMode.R, driveSpeed.e[1])
  };
}

float driveMotorPair(Adafruit_DCMotor *motors[2], float speed) {
  float motorSpeed = 0;
  speed = constrain(speed, -DRIVE_LIMIT, DRIVE_LIMIT);
  if (speed > DRIVE_STOP) {
    motorSpeed = map(speed, DRIVE_STOP, DRIVE_LIMIT, MOTOR_MIN, MOTOR_MAX);
    motors[0]->run(FORWARD);
    motors[1]->run(FORWARD);
    motors[0]->setSpeed(motorSpeed);
    motors[1]->setSpeed(motorSpeed);
  } else if (speed < -DRIVE_STOP) {
    motorSpeed = map(speed, -DRIVE_STOP, -DRIVE_LIMIT, MOTOR_MIN, MOTOR_MAX);
    motors[0]->run(BACKWARD);
    motors[1]->run(BACKWARD);
    motors[0]->setSpeed(motorSpeed);
    motors[1]->setSpeed(motorSpeed);
  } else {
    motors[0]->run(RELEASE);
    motors[1]->run(RELEASE);
  }
  return motorSpeed;
}
