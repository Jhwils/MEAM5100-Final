#include <WiFi.h>
#include <websitekeys.h>
#include "html510.h"
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_VL53L1X.h"
#include <math.h>

// ================= WIFI ================
const char* ssid = "LEDcontrol";
HTML510Server html(80);
uint8_t packets = 0;
unsigned long lastPacketMillis = 0;
const unsigned long PACKET_INTERVAL_MS = 500;  // 2 Hz

// ============= TOP HAT =================
// I2C Slave Communication
#define I2C_SLAVE_ADDR 0x28
#define SDA_PIN 36
#define SCL_PIN 37

// ================= IMU =================
#define SDA_PIN 36
#define SCL_PIN 37
Adafruit_BNO055 bno(55, 0x29, &Wire);

// ================= TOF =================
Adafruit_VL53L1X tofLeft, tofFront, tofRight;

#define XSHUT_LEFT   12
#define XSHUT_FRONT  13
#define XSHUT_RIGHT  14

int16_t distLeft = -1;
int16_t distFront = -1;
int16_t distRight = -1;

// ================= MOTORS =================
const int pwmL = 15;
const int pwmR = 40;
const int fwdL = 6;
const int revL = 7;
const int fwdR = 41;
const int revR = 42;
const int PWM_RES  = 12;
const int MAX_DUTY = 4095;

// ================= SERVO =================
// Servo control pin (choose an appropriate pin for your servo)
const int SERVO_PIN = 21;  // You can change this to any valid PWM pin
const uint16_t SERVO_MIN_US = 1350;
const uint16_t SERVO_MAX_US = 2400;
uint16_t servoPulseUs = SERVO_MIN_US;
unsigned long lastServoFrameUs = 0;
bool servoHigh = false;
unsigned long lastServoMoveMs = 0;
const unsigned long SERVO_MOVE_INTERVAL_MS = 500; // 2 Hz
bool servoForward = true;


// ================= SPEED =================
int basePWM = 45;
#define TURN_PWM 30

// ================= WALL FOLLOW =================
#define WALL_TARGET_MM 100
#define WALL_KP        0.2
#define MAX_BIAS       10
#define WALL_DEADBAND  15

// ================= YAW =================
float yawRaw = 0;
float yawOffset = 0;
float yawRobot = 0;
float targetYaw = 0;
float headingHold = 0;
bool yawZeroed = false;

#define YAW_KP  0.8
#define YAW_TOL 3.0

float lastLPWM = 0;
float lastRPWM = 0;

// ================= STATE =================
// 0 stop, 1 fwd, 2 back, 3 left, 4 right, 5 turn
bool autoMode = false;
int motion = 0;

// ================= UTILS =================
float wrap180(float a) {
  while (a > 180) a -= 360;
  while (a < -180) a += 360;
  return a;
}

float quatYaw(float w, float x, float y, float z) {
  return atan2f(
    2 * (w * z + x * y),
    1 - 2 * (y * y + z * z)
  ) * 180 / M_PI;
}

void setPWM(int l, int r) {
  l = constrain(l, 0, 100);
  r = constrain(r, 0, 100);
  ledcWrite(pwmL, (MAX_DUTY * l) / 100);
  ledcWrite(pwmR, (MAX_DUTY * r) / 100);
  lastLPWM = l;
  lastRPWM = r;
}

void stopMotors() {
  setPWM(0, 0);
  digitalWrite(fwdL, LOW);
  digitalWrite(revL, LOW);
  digitalWrite(fwdR, LOW);
  digitalWrite(revR, LOW);
}

bool sendWiFiPacketsByte(uint8_t value) {
  // Send data to slave
  Wire.beginTransmission(I2C_SLAVE_ADDR);
  Wire.write(value);
  uint8_t error = Wire.endTransmission();   // 0 = success

  if (error == 0) {
    Serial.printf("[TX] Sent: %u\n", value);
    return true;
  } else {
    Serial.printf("[TX ERROR] Code: %d\n", error);
    return false;
  }
}

// ================= TOF =================
void updateTOF() {
  if (tofLeft.dataReady()) {
    distLeft = tofLeft.distance();
    tofLeft.clearInterrupt();
  }

  if (tofFront.dataReady()) {
    distFront = tofFront.distance();
    tofFront.clearInterrupt();
  }

  // if (tofRight.dataReady()) {
  //   distRight = tofRight.distance();
  //   tofRight.clearInterrupt();
  // }
}

// ================ SERVO ================
void updateServo() {
  unsigned long now = micros();

  // Start of 20ms frame
  if (!servoHigh && (now - lastServoFrameUs >= 20000)) {
    lastServoFrameUs = now;
    digitalWrite(SERVO_PIN, HIGH);
    servoHigh = true;
  }

  // End of pulse
  if (servoHigh && (now - lastServoFrameUs >= servoPulseUs)) {
    digitalWrite(SERVO_PIN, LOW);
    servoHigh = false;
  }

  Serial.println("Updating servo speed");
}

void updateServoMotion() {
  unsigned long now = millis();

  if (now - lastServoMoveMs >= SERVO_MOVE_INTERVAL_MS) {
    lastServoMoveMs = now;

    if (servoForward) {
      servoPulseUs = SERVO_MAX_US;
    } else {
      servoPulseUs = SERVO_MIN_US;
    }

    servoForward = !servoForward;

    Serial.println("Updating servo speed");
  }
}

// ================= TURN (AUTO) =================
void startTurn(float delta) {
  targetYaw = wrap180(yawRobot + delta);
  motion = 5;
}

void handleTurn() {
  float err = wrap180(targetYaw - yawRobot);

  if (fabs(err) < YAW_TOL) {
    yawOffset = yawRaw;
    yawRobot = 0;
    stopMotors();
    motion = 1;
    return;
  }

  if (err > 0) {
    digitalWrite(fwdL, LOW);
    digitalWrite(revL, HIGH);
    digitalWrite(fwdR, HIGH);
    digitalWrite(revR, LOW);
  } else {
    digitalWrite(fwdL, HIGH);
    digitalWrite(revL, LOW);
    digitalWrite(fwdR, LOW);
    digitalWrite(revR, HIGH);
  }

  setPWM(TURN_PWM, TURN_PWM);
}

// ================= WALL FOLLOW =================
void wallFollow() {

  if (motion == 5) {
    handleTurn();
    return;
  }

  if (distLeft > 250) {
    startTurn(+5);
    return;
  }

  if (distFront > 150 && distLeft > 0) {

    float wallErr = WALL_TARGET_MM - distLeft;
    if (abs(wallErr) < WALL_DEADBAND) wallErr = 0;

    float bias =
      WALL_KP * wallErr +
      YAW_KP * (yawRobot - headingHold);

    bias = constrain(bias, -MAX_BIAS, MAX_BIAS);

    digitalWrite(fwdL, HIGH);
    digitalWrite(revL, LOW);
    digitalWrite(fwdR, HIGH);
    digitalWrite(revR, LOW);

    setPWM(basePWM - bias, basePWM + bias);
    motion = 1;
    return;
  }

  if (distFront < 150) {
    startTurn(-50);
  }
}

// bool motorStalled() {
//   // Example: use front TOF distance to detect stall
//   // If moving forward and front distance doesn't change much, consider stalled
//   static int16_t lastDistFront = -1;
//   static unsigned long lastCheck = 0;

//   unsigned long now = millis();
//   if (now - lastCheck < 200) return false; // check every 200 ms
//   lastCheck = now;

//   if (motion != 1) return false; // only check forward motion
//   if (distFront < 0) return false; // invalid reading

//   if (lastDistFront >= 0 && abs(distFront - lastDistFront) < 2) { // <2mm change = stalled
//     lastDistFront = distFront;
//     return true;
//   }

//   lastDistFront = distFront;
//   return false;
// }

// ================= WEB =================
extern const char Slider[];

void Homepage() { html.sendhtml(Slider); }

void Forward() {
  packets++;
  autoMode = false;
  motion = 1;
  yawOffset = yawRaw;
  digitalWrite(fwdL, HIGH);
  digitalWrite(revL, LOW);
  digitalWrite(fwdR, HIGH);
  digitalWrite(revR, LOW);
  setPWM(35, 35);
}

void Backward() {
  packets++;
  autoMode = false;
  motion = 2;
  digitalWrite(fwdL, LOW);
  digitalWrite(revL, HIGH);
  digitalWrite(fwdR, LOW);
  digitalWrite(revR, HIGH);
  setPWM(basePWM, basePWM);
}

void Left() {
  packets++;
  autoMode = false;
  motion = 3;
  digitalWrite(fwdL, LOW);
  digitalWrite(revL, HIGH);
  digitalWrite(fwdR, HIGH);
  digitalWrite(revR, LOW);
  setPWM(25, 25);
}

void Right() {
  packets++;
  autoMode = false;
  motion = 4;
  digitalWrite(fwdL, HIGH);
  digitalWrite(revL, LOW);
  digitalWrite(fwdR, LOW);
  digitalWrite(revR, HIGH);
  setPWM(25, 25);
}

void StopMotor() {
  autoMode = false;
  motion = 0;
  stopMotors();
}

void AutoOn() {
  autoMode = true;
  motion = 1;
  yawOffset = yawRaw;
  yawRobot = 0;
}

void AutoOff() {
  autoMode = false;
  StopMotor();
}

void Zero() {
  yawOffset = yawRaw;
  yawRobot = 0;
}

void Status() {
  char msg[160];
  snprintf(
    msg, sizeof(msg),
    "%.1f,%.1f,%.1f,%.1f,%d,%d,%d,%s",
    yawRobot, targetYaw, lastLPWM, lastRPWM,
    distLeft, distFront, distRight,
    autoMode ? "AUTO" : "MANUAL"
  );
  html.sendplain(msg);
}

// ================= SETUP =================
void setup() {

  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN, 40000);

  WiFi.softAP(ssid);
  html.begin(80);

  html.attachHandler("/", Homepage);
  html.attachHandler("/FORWARD", Forward);
  html.attachHandler("/BACKWARD", Backward);
  html.attachHandler("/LEFT", Left);
  html.attachHandler("/RIGHT", Right);
  html.attachHandler("/STOPMOTOR", StopMotor);
  html.attachHandler("/AUTOON", AutoOn);
  html.attachHandler("/AUTOOFF", AutoOff);
  html.attachHandler("/ZERO", Zero);
  html.attachHandler("/STATUS", Status);

  bno.begin();
  bno.setExtCrystalUse(true);

  pinMode(fwdL, OUTPUT);
  pinMode(revL, OUTPUT);
  pinMode(fwdR, OUTPUT);
  pinMode(revR, OUTPUT);

  ledcAttach(pwmL, 500, PWM_RES);
  ledcAttach(pwmR, 500, PWM_RES);

  pinMode(XSHUT_LEFT, OUTPUT);
  pinMode(XSHUT_FRONT, OUTPUT);
  pinMode(XSHUT_RIGHT, OUTPUT);

  digitalWrite(XSHUT_LEFT, LOW);
  digitalWrite(XSHUT_FRONT, LOW);
  digitalWrite(XSHUT_RIGHT, LOW);
  delay(100);

  digitalWrite(XSHUT_LEFT, HIGH);
  delay(100);
  tofLeft.begin(0x30, &Wire);
  tofLeft.startRanging();

  digitalWrite(XSHUT_FRONT, HIGH);
  delay(100);
  tofFront.begin(0x31, &Wire);
  tofFront.startRanging();

  // digitalWrite(XSHUT_RIGHT, HIGH);
  // delay(100);
  // tofRight.begin(0x32, &Wire);
  // tofRight.startRanging();

  pinMode(SERVO_PIN, OUTPUT);
  digitalWrite(SERVO_PIN, LOW);


  Serial.println("=== ROBOT READY ===");
}

// ================= LOOP =================
void loop() {

  html.serve();
  updateTOF();

  imu::Quaternion q = bno.getQuat();
  yawRaw = wrap180(quatYaw(q.w(), q.x(), q.y(), q.z()));

  if (!yawZeroed) {
    yawOffset = yawRaw;
    yawZeroed = true;
  }

  yawRobot = wrap180(yawRaw - yawOffset);

  if (autoMode) wallFollow();

  unsigned long now = millis();

  if (now - lastPacketMillis >= PACKET_INTERVAL_MS) {
    lastPacketMillis = now;

    sendWiFiPacketsByte(uint8_t (packets / 4));
    Serial.printf("Sent data: %u\n", uint8_t (packets / 4));

    packets = 0;   // reset after send
  }

  updateServo();        // generates pulses (MUST be called often)
  updateServoMotion(); // updates target position (slow)

//   // --- Adaptive PWM if motor stalls ---
// if (!autoMode && motion > 0) {  // only in manual mode when moving
//   if (motorStalled()) {
//     Serial.println("Motor stall detected! Increasing PWM...");
//     basePWM = constrain(basePWM + 5, 0, 60);  // increase by 5%
//     setPWM(basePWM, basePWM);
//   }
// }

}