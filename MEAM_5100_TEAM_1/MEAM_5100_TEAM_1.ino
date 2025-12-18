// =======================================================================================
//  ADVANCED WALL FOLLOWING + 2-WHEEL PID DRIVE + TOF + 9-DOF IMU + VIVE + SERVO + TOP HAT
//  ESP32-S3  (VL53L1X + BNO055 + Encoders + PID + VIVE + MG996 + Hat + Ramp Acceleration)
// =======================================================================================

#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <utility/imumaths.h>
#include <WiFi.h>
#include "Adafruit_VL53L1X.h"
#include "html510.h"
#include "vive510.h"

/* ========================================================
   WIFI AND WEBSITE
   ======================================================== */

const char *ssid = "Team_1_WiFi";
const char *password = "12345678";

IPAddress myIP(192, 168, 1, 147);
HTML510Server webServer(80);

// Graded Task Booleans
bool navTOFActive = false;
bool navAxisActive = false;
bool lowerTowerActive = false;
bool higherTowerActive = false;
bool nexusTowerActive = false;
bool allTowersActive = false;

// Graded Task States
int allTowersPhase = 0;
unsigned long allTowersTimer = 0;
int lowerTowerPhase = 0;
unsigned long lowerTowerTimer = 0;
int higherTowerPhase = 0;
unsigned long higherTowerTimer = 0;
int nexusTowerPhase = 0;
unsigned long nexusTowerTimer = 0;
int nexusTowerHitCount = 0;

// HTML
String pageHtml() {
  return R"rawliteral(
    <!DOCTYPE html>
    <html>
        <head>
            <meta charset="utf-8"/>
            <title>ESP32 PWM Control</title>
            <style>
                body { font-family: Arial, sans-serif; text-align: center; background: #f9f9f9; }
                .slider { width: 80%; margin: 8px 0; }
                .box { background: white; padding: 16px; margin: 30px auto; width: 90%; max-width: 600px; border-radius: 8px; box-shadow: 0 2px 6px rgba(0, 0, 0, 0.1); }
                p { margin: 6px 0; }

                /* Toggle switch styling */
                .switch {
                    position: relative;
                    display: inline-block;
                    width: 60px;
                    height: 34px;
                }
                .switch input {
                    opacity: 0;
                    width: 0;
                    height: 0;
                }
                .slider-toggle {
                    position: absolute;
                    cursor: pointer;
                    top: 0; left: 0;
                    right: 0; bottom: 0;
                    background-color: #ccc;
                    transition: .4s;
                    border-radius: 34px;
                }
                .slider-toggle:before {
                    position: absolute;
                    content: "";
                    height: 26px; width: 26px;
                    left: 4px; bottom: 4px;
                    background-color: white;
                    transition: .4s;
                    border-radius: 50%;
                }
                input:checked + .slider-toggle {
                    background-color: #4CAF50;
                }
                input:checked + .slider-toggle:before {
                    transform: translateX(26px);
                }

                /* Button styles */
                .btn {
                    padding: 10px 20px;
                    margin: 8px;
                    border-radius: 8px;
                    background-color: #4CAF50;
                    color: white;
                    border: none;
                    cursor: pointer;
                    font-size: 16px;
                }
                .btn-red {
                    background-color: #f44336;
                }
                .btn:hover {
                    opacity: 0.9;
                }
                .btn-group {
                    display: flex;
                    justify-content: center;
                    margin: 10px 0;
                }

                /* Input and label for coordinates */
                .coord-input {
                    font-size: 16px;
                    margin: 8px;
                }
                .coord-group {
                    display: inline-block;
                    margin: 10px 0;
                }
            </style>
        </head>
        <body>
            <div class="box">
                <h2>ESP32 Autonomous Car</h2>
            </div>

            <div class="box">
                <h3>Maneuver Controls</h3>
                <p>Direction: <span id="dirTxt">Reverse</span></p>
                <label class="switch">
                    <input id="dirSwitch" type="checkbox" onchange="toggleDirection(this)">
                    <span class="slider-toggle"></span>
                </label>
                <p>Speed: <span id="speedTxt">0</span>%</p>
                <input id="speedSlider" class="slider" type="range" min="0" max="100" step="1" value="0" 
                    onchange="setSpeed(this.value)" 
                    oninput="document.getElementById('speedTxt').innerText=this.value">
                <div class="btn-group">
                    <button class="btn" onclick="spin('left')">Spin Left</button>
                    <button class="btn" onclick="spin('right')">Spin Right</button>
                </div>
                <div class="btn-group">
                    <button class="btn" onclick="turn(90, 'left')">90° Left</button>
                    <button class="btn" onclick="turn(90, 'right')">90° Right</button>
                </div>
                <div class="btn-group">
                    <button class="btn" onclick="turn(180, 'left')">180° Left</button>
                    <button class="btn" onclick="turn(180, 'right')">180° Right</button>
                </div>
                <button class="btn btn-red" onclick="stop()">Stop</button>
            </div>

            <div class="box">
                <h3>TOF Graded Tasks</h3>
                <button class="btn" onclick="gradedTask('left_wall')">Full Circuit (Left)</button>
                <button class="btn" onclick="gradedTask('right_wall')">Full Circuit (Right)</button>
                <h3>VIVE Graded Tasks</h3>
                <button class="btn" onclick="gradedTask('blue_higher_tower')">Blue Higher Tower</button>
                <button class="btn" onclick="gradedTask('blue_lower_tower')">Blue Lower Tower</button>
                <h3>MIX Graded Tasks</h3>
                <button class="btn" onclick="gradedTask('blue_nexus')">Blue Nexus</button>
                <button class="btn" onclick="gradedTask('attack_everything')">Attack Everything</button>
            </div>

            <div class="box">
                <h3>VIVE Location</h3>
                <div class="coord-group">
                    <label for="xCoord" class="coord-input">X:</label>
                    <input id="xCoord" type="number" class="coord-input" placeholder="X" />
                </div>
                <div class="coord-group">
                    <label for="yCoord" class="coord-input">Y:</label>
                    <input id="yCoord" type="number" class="coord-input" placeholder="Y" />
                </div>
                <button class="btn" onclick="sendViveCoordinates()">Go</button>
            </div>

            <script>
                // Initialize direction as Reverse (unchecked)
                let direction = "reverse";

                function toggleDirection(el) {
                    direction = el.checked ? "forward" : "reverse";
                    document.getElementById('dirTxt').innerText = direction.charAt(0).toUpperCase() + direction.slice(1);
                    setDirection(direction);
                }

                function setDirection(dir) {
                    fetch('/setDirection?value=' + dir)
                        .catch((e) => console.log('err', e));
                }

                function setSpeed(v) {
                    fetch('/setDutyCycle?value=' + v)
                        .catch((e) => console.log('err', e));
                }

                function spin(direction) {
                    fetch(`/spin?direction=${direction}`)
                        .catch((e) => console.log('err', e));
                }

                function turn(degree, direction) {
                    fetch(`/turn?degree=${degree}&direction=${direction}`)
                        .catch((e) => console.log('err', e));
                }

                function stop() {
                    fetch('/stop')
                        .catch((e) => console.log('err', e));
                }

                function gradedTask(taskType) {
                    fetch(`/gradedTask?type=${taskType}`)
                        .catch((e) => console.log('err', e));
                }

                function sendViveCoordinates() {
                    const xCoord = document.getElementById('xCoord').value;
                    const yCoord = document.getElementById('yCoord').value;

                    fetch(`/viveCoordinates?x=${xCoord}&y=${yCoord}`)
                        .catch((e) => console.log('err', e));
                }
            </script>
        </body>
    </html>
  )rawliteral";
}

/* ========================================================
   TOP HAT DEFINITIONS
   ======================================================== */
#define I2C_TOP_HAT_ADDR 0x28
#define SDA_PIN 36
#define SCL_PIN 37

uint8_t packets = 0;
uint8_t robotHealth = 100;
unsigned long previousMillisTopHat = 0;  // Store the time of the last update
const unsigned long topHatPacketPeriod = 500;

/* ========================================================
   TOF AND IMU SENSOR DEFINITIONS
   ======================================================== */

Adafruit_VL53L1X tofLeft = Adafruit_VL53L1X();
Adafruit_VL53L1X tofFront = Adafruit_VL53L1X();
Adafruit_VL53L1X tofRight = Adafruit_VL53L1X();

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29, &Wire);

#define XSHUT_LEFT 12
#define XSHUT_FRONT 13
#define XSHUT_RIGHT 14

#define WALL_DIST_MM 100   // Desired wall following distance (10 cm)
#define FRONT_STOP_MM 150  // Stop/turn if front < 15 cm
#define CLEAR_DIST_MM 500  // “Open space” threshold

/* ========================================================
   SERVO DEFINITIONS
   ======================================================== */
const int servoPin = 21;
const int servoFreq = 50;        // 50 Hz for servo
const int servoResolution = 16;  // 16-bit resolution
const uint32_t servoMaxDuty = (1 << servoResolution) - 1;

const int minUs = 1200;
const int maxUs = 2400;

unsigned long previousMillisServo = 0;
bool movingForward = true;
const float frequency = 1;  // 1 Hz movement
unsigned long halfPeriodMs = (1.0 / frequency) * 500.0;

/* ========================================================
   VIVE DEFINITIONS
   ======================================================== */
#define VIVE1_SIGNAL_PIN 47
#define VIVE2_SIGNAL_PIN 48

Vive510 vive1(VIVE1_SIGNAL_PIN);
Vive510 vive2(VIVE2_SIGNAL_PIN);

// Smoothing factor (0.2 = smooth, 0.5 = fast)
const float ALPHA = 0.38;

// Filter state variables
float x1_f = 0, y1_f = 0;
float x2_f = 0, y2_f = 0;
bool first1 = true, first2 = true;

// Navigation related variables
float targetViveX = 0;
float targetViveY = 0;
double viveOrientation = 0;
int navPhase = 0;    // 0 = move Y, 1 = turn X, 2 = move X, 3 = done
float posErr = 300;  // acceptable Vive unit error

/* ========================================================
   MOTOR + ENCODER DEFINITIONS
   ======================================================== */
const int pwmFreq = 2000;
const int pwmRes = 8;
const int ledcRes = ((1 << pwmRes) - 1);
int BASE_DUTY_CYCLE = 80;
int SLOW_DUTY_CYCLE = 50;

// LEFT MOTOR
const int dirL1 = 6;
const int dirL2 = 7;
const int pwmL = 15;

// RIGHT MOTOR
const int dirR1 = 42;
const int dirR2 = 41;
const int pwmR = 40;

// LEFT ENCODERS
const int encLA = 16;
const int encLB = 17;

// RIGHT ENCODERS
const int encRA = 39;
const int encRB = 38;

// Encoder counters
volatile long posL = 0;
volatile long posR = 0;
volatile uint8_t lastL = 0;
volatile uint8_t lastR = 0;

/* ========================================================
   SPEED PID VARIABLES
   ======================================================== */
double Kp = 1.2, Ki = 0.4, Kd = 0.05;

double inputL, inputR;
double outputL, outputR;

double goalDutyCycleL = 0, goalDutyCycleR = 0;
double prevErrorL = 0, prevErrorR = 0;
double integralL = 0, integralR = 0;

long prevPosL = 0;
long prevPosR = 0;

double speedL = 0;
double speedR = 0;

// Ticks per loop at ~100% duty
const double TICKS_AT_100_DUTY = 30.0;

/* ========================================================
   RAMPING SYSTEM (SMOOTH ACCEL/DECEL)
   ======================================================== */
double rampRate = 5;  // Units per loop (50ms), tune for smooth accel

double rampTo(double currentDutyCycle, double targetDutyCycle) {
  if (currentDutyCycle - targetDutyCycle < rampRate) currentDutyCycle = targetDutyCycle;
  if (currentDutyCycle < targetDutyCycle) currentDutyCycle += rampRate;
  if (currentDutyCycle > targetDutyCycle) currentDutyCycle -= rampRate;
  if (currentDutyCycle < 0) currentDutyCycle = 0;
  return currentDutyCycle;
}

/* ========================================================
   QUADRATURE DECODER
   ======================================================== */
void IRAM_ATTR updateEncoder(volatile long &pos, volatile uint8_t &last, int pinA, int pinB) {
  uint8_t state = (digitalRead(pinA) << 1) | digitalRead(pinB);
  uint8_t combined = (last << 2) | state;

  static const int8_t table[16] = {
    0, -1, +1, 0,
    +1, 0, 0, -1,
    -1, 0, 0, +1,
    0, +1, -1, 0
  };

  pos += table[combined];
  last = state;
}

void IRAM_ATTR isrL() {
  updateEncoder(posL, lastL, encLA, encLB);
}
void IRAM_ATTR isrR() {
  updateEncoder(posR, lastR, encRA, encRB);
}

/* ========================================================
   PID
   ======================================================== */
void computePID(double &input, double &output, double setpoint,
                double &prevError, double &integral) {
  double error = setpoint - input;
  integral += error;
  double derivative = error - prevError;

  output = Kp * error + Ki * integral + Kd * derivative;

  prevError = error;

  if (output > 100) output = 100;
  if (output < 0) output = 0;
}

/* ========================================================
   MOTOR CONTROL
   ======================================================== */
void setMotor(int pwmPin, double pwmDutyCycleValue) {
  pwmDutyCycleValue = constrain(pwmDutyCycleValue, 0, 100);
  uint16_t duty = (ledcRes * pwmDutyCycleValue) / 100;
  ledcWrite(pwmPin, (int)duty);
}

/* ========================================================
   MOVEMENT COMMANDS
   ======================================================== */
void resetPID() {
  prevPosL = posL;
  prevPosR = posR;
  integralL = integralR = 0;
  prevErrorL = prevErrorR = 0;
  posL = posR = 0;
}

void goForward(int dutyCycleL, int dutyCycleR) {
  digitalWrite(dirL1, HIGH);
  digitalWrite(dirL2, LOW);

  digitalWrite(dirR1, HIGH);
  digitalWrite(dirR2, LOW);

  goalDutyCycleL = dutyCycleL;
  goalDutyCycleR = dutyCycleR;
  resetPID();
}

void goReverse(int dutyCycleL, int dutyCycleR) {
  digitalWrite(dirL1, LOW);
  digitalWrite(dirL2, HIGH);

  digitalWrite(dirR1, LOW);
  digitalWrite(dirR2, HIGH);

  goalDutyCycleL = dutyCycleL;
  goalDutyCycleR = dutyCycleR;
  resetPID();
}

void turnLeft(int dutyCycleL, int dutyCycleR) {
  digitalWrite(dirL1, LOW);
  digitalWrite(dirL2, HIGH);

  digitalWrite(dirR1, HIGH);
  digitalWrite(dirR2, LOW);

  goalDutyCycleL = dutyCycleL;
  goalDutyCycleR = dutyCycleR;
  resetPID();
}

void turnRight(int dutyCycleL, int dutyCycleR) {
  digitalWrite(dirL1, HIGH);
  digitalWrite(dirL2, LOW);

  digitalWrite(dirR1, LOW);
  digitalWrite(dirR2, HIGH);

  goalDutyCycleL = dutyCycleL;
  goalDutyCycleR = dutyCycleR;
  resetPID();
}

void stop() {
  digitalWrite(dirL1, HIGH);
  digitalWrite(dirL2, HIGH);

  digitalWrite(dirR1, HIGH);
  digitalWrite(dirR2, HIGH);

  goalDutyCycleL = 0;
  goalDutyCycleR = 0;
  resetPID();

  // Hard stop right away
  setMotor(pwmL, goalDutyCycleL);
  setMotor(pwmR, goalDutyCycleR);
  delay(200);
}

/* ========================================================
   BNO055 HEADING AND TURNING
   ======================================================== */
double getHeading() {
  sensors_event_t event;
  bno.getEvent(&event);
  double h = event.orientation.x;
  if (h < 0) h += 360;
  return h;
}

void turnToAngle(double target) {
  double KpTurn = 2.0;
  double KdTurn = 0.4;
  double prevErr = 0;

  while (true) {
    double heading = getHeading();

    double error = target - heading;
    if (error > 180) error -= 360;
    if (error < -180) error += 360;

    // Scale this down to make it more accurate
    if (abs(error) < 5) {
      stop();
      break;
    }

    double derivative = error - prevErr;
    prevErr = error;

    double correction = KpTurn * error + KdTurn * derivative;
    correction = constrain(correction, -50, 50);

    if (correction > 0) turnLeft(abs(correction), abs(correction));
    else turnRight(abs(correction), abs(correction));

    setMotor(pwmL, abs(correction));
    setMotor(pwmR, abs(correction));

    delay(10);
  }
}

/* ========================================================
   VIVE FUNCTIONS
   ======================================================== */
bool readLPF(Vive510 &vive, float &xf, float &yf, bool &first) {
  if (vive.status() != VIVE_RECEIVING) {
    Serial.println("VIVE not working as expected, needs to resync!");
    return false;
  }

  uint16_t xr = vive.xCoord();
  uint16_t yr = vive.yCoord();

  // Basic validity gate
  if (xr < 1000 || xr > 8000 || yr < 1000 || yr > 8000) {
    Serial.println("Invalid sensor data, skipping...");
    return false;
  }

  // Apply low-pass filter
  if (first) {
    xf = xr;
    yf = yr;
    first = false;
  } else {
    xf = ALPHA * xr + (1.0 - ALPHA) * xf;
    yf = ALPHA * yr + (1.0 - ALPHA) * yf;
  }

  return true;
}

int decideAxisMovement(float targetY) {
  // If target is inside obstacle Y-strip, move X first
  if (targetY >= 3700 && targetY <= 4600) {
    return 0;  // Move X first
  }
  return 1;  // Default: Move Y first
}

float targetHeadingForPoint(float currentX, float targetX) {
  if (targetX > currentX) return 90.0;  // + direction
  else return 270.0;                    // - direction
}

/* ========================================================
   TOP HAT FUNCTIONS
   ======================================================== */

//--------------------------------------------------------------
// Send number of WiFi packets (1 byte) to the top hat
//--------------------------------------------------------------
bool sendWiFiPacketsByte(uint8_t value) {
  // Send data to the top hat
  Wire.beginTransmission(I2C_TOP_HAT_ADDR);
  Wire.write(value);
  uint8_t error = Wire.endTransmission();  // 0 = success

  if (error == 0) {
    Serial.printf("[TX] Sent: %u\n", value);
    return true;
  } else {
    Serial.printf("[TX ERROR] Code: %d\n", error);
    return false;
  }
}

//--------------------------------------------------------------
// Receive health value (1 byte) from the top hat
//--------------------------------------------------------------
int receiveHealthValueByte() {
  // Stored across calls
  static int lastHealth = 100;

  // Request up to 1 byte, but allow more if the top hat responds late
  int bytes = Wire.requestFrom(I2C_TOP_HAT_ADDR, (uint8_t)1);

  if (bytes > 0) {
    // Read ALL available bytes (safety for timing drift / lag)
    uint8_t health;
    while (Wire.available()) {
      // Last byte becomes the health
      health = Wire.read();
    }

    lastHealth = health;
    Serial.printf("[RX] Health: %u (bytes=%d)\n", lastHealth, bytes);
  } else {
    // No data → keep the last known value
    Serial.printf("[RX] No data received, keeping last health: %u\n", lastHealth);
  }

  return lastHealth;
}

/* ========================================================
   SERVO FUNCTIONS
   ======================================================== */
uint32_t servoDutyFromUs(uint32_t pulseUs) {
  uint32_t periodUs = 1000000UL / servoFreq;  // 20,000 us at 50 Hz
  return (pulseUs * servoMaxDuty) / periodUs;
}

/* ========================================================
   WEBSITE COMMANDS
   ======================================================== */
// Handle root
void handleRoot() {
  webServer.sendhtml(pageHtml());
}

// Handle direction updates
void handleSetDirection() {
  // Get new direction
  String txt = webServer.getText();
  bool isForward = txt.equals("forward");

  // Update motor direction pins
  isForward ? goForward(BASE_DUTY_CYCLE, BASE_DUTY_CYCLE) : goReverse(SLOW_DUTY_CYCLE, SLOW_DUTY_CYCLE);

  // Reply
  webServer.sendplain(String("OK direction=" + String(txt)));
  Serial.printf("Direction set to %s\n", txt);
  packets++;
}

// Handle duty cycle updates
void handleSetDutyCycle() {
  // Get new duty cycle
  String txt = webServer.getText();
  int dutyPct = txt.toInt();
  if (dutyPct < 0) dutyPct = 0;
  if (dutyPct > 100) dutyPct = 100;

  // Update current duty cycle
  BASE_DUTY_CYCLE = dutyPct;
  BASE_DUTY_CYCLE = dutyPct;

  // Reply
  webServer.sendplain(String("OK duty=" + String(dutyPct)));
  Serial.printf("Duty set to %d%%\n", dutyPct);
  packets++;
}

// Handle spin direction updates
void handleSpinDirection() {
  // Get new direction
  String txt = webServer.getText();
  bool isLeft = txt.equals("left");

  // Update motor direction pins
  isLeft ? turnLeft(SLOW_DUTY_CYCLE, SLOW_DUTY_CYCLE) : turnRight(SLOW_DUTY_CYCLE, SLOW_DUTY_CYCLE);

  // Reply
  webServer.sendplain(String("OK direction=" + String(txt)));
  Serial.printf("Direction set to %s\n", txt);
  packets++;
}

// Handle turn direction updates
void handleTurnDirection() {
  // Read entire query string (e.g. "90&direction=left")
  String txt = webServer.getText();

  // Parse degree
  int ampIndex = txt.indexOf('&');  // find '&'
  String degreeStr = txt.substring(0, ampIndex);
  int degree = degreeStr.toInt();

  // Parse direction ("left" or "right")
  int eqIndex = txt.indexOf('=', ampIndex);
  String direction = txt.substring(eqIndex + 1);

  // --- 4 conditions ---
  if (degree == 90 && direction == "left") {
    double target = getHeading() + 90;
    if (target >= 360) target -= 360;
    turnToAngle(target);
  } else if (degree == 90 && direction == "right") {
    double target = getHeading() - 90;
    if (target < 0) target += 360;
    turnToAngle(target);
  } else if (degree == 180 && direction == "left") {
    double target = getHeading() + 180;
    if (target >= 360) target -= 360;
    turnToAngle(target);
  } else if (degree == 180 && direction == "right") {
    double target = getHeading() - 180;
    if (target < 0) target += 360;
    turnToAngle(target);
  } else {
    Serial.println("Unknown turn command.");
  }

  webServer.sendplain(String("OK direction=" + String(txt)));
  Serial.printf("TURN COMPLETED: degree=%d direction=%s\n", degree, direction.c_str());
  packets++;
}

// Handle stop command updates
void handleStop() {
  // Stop the car
  stop();

  // Reply
  webServer.sendplain("OK stopped the car");
  Serial.printf("Stopped the car");
  packets++;
}

// Handle graded task updates
void handleGradedTask() {
  String txt = webServer.getText();
  Serial.printf("Graded Task Requested: %s\n", txt.c_str());

  if (txt == "blue_lower_tower") {
    navAxisActive = false;
    navTOFActive = false;
    lowerTowerActive = true;
    higherTowerActive = false;
    nexusTowerActive = false;
    allTowersActive = false;
    lowerTowerPhase = 0;
  } else if (txt == "blue_higher_tower") {
    navAxisActive = false;
    navTOFActive = false;
    lowerTowerActive = false;
    higherTowerActive = true;
    nexusTowerActive = false;
    allTowersActive = false;
    higherTowerPhase = 0;
  } else if (txt == "blue_nexus") {
    navAxisActive = false;
    navTOFActive = false;
    lowerTowerActive = false;
    higherTowerActive = false;
    nexusTowerActive = true;
    allTowersActive = false;
    nexusTowerPhase = 0;
    nexusTowerHitCount = 0;
  } else if (txt == "attack_everything") {
    navAxisActive = false;
    navTOFActive = false;
    lowerTowerActive = false;
    higherTowerActive = false;
    nexusTowerActive = false;
    allTowersActive = true;
    allTowersPhase = 0;
  } else if (txt == "left_wall" || txt == "right_wall") {
    navTOFActive = true;
    navAxisActive = false;
    lowerTowerActive = false;
    higherTowerActive = false;
    nexusTowerActive = false;
    allTowersActive = false;
  } else {
    Serial.println("Unknown graded task.");
  }

  webServer.sendplain("OK graded task");
  packets++;
}


// Handle vive coordinate updates
void handleVive() {
  // full string looks like: "123&y=456"
  String txt = webServer.getText();

  // Get index of the differentiator
  int ampIndex = txt.indexOf('&');

  // Extract X
  targetViveX = txt.substring(0, ampIndex).toFloat();

  // Extract Y
  int eqIndex = txt.indexOf('=', ampIndex);
  targetViveY = txt.substring(eqIndex + 1).toFloat();

  // Initiate navigation mode
  navAxisActive = true;
  navTOFActive = false;
  lowerTowerActive = false;
  higherTowerActive = false;
  nexusTowerActive = false;
  allTowersActive = false;
  navPhase = 0;

  Serial.printf("VIVE COORDINATES:  X=%.2f   Y=%.2f\n", targetViveX, targetViveY);

  webServer.sendplain("OK vive received");
  packets++;
}

/* ========================================================
   SETUP
   ======================================================== */
void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Wire.begin(SDA_PIN, SCL_PIN);
  Serial.println("Set the SDA and SCL pins");

  pinMode(XSHUT_LEFT, OUTPUT);
  pinMode(XSHUT_FRONT, OUTPUT);
  pinMode(XSHUT_RIGHT, OUTPUT);

  digitalWrite(XSHUT_LEFT, LOW);
  digitalWrite(XSHUT_FRONT, LOW);
  digitalWrite(XSHUT_RIGHT, LOW);
  delay(50);

  digitalWrite(XSHUT_LEFT, HIGH);
  delay(100);
  if (!tofLeft.begin(0x40, &Wire)) {
    Serial.println("Left TOF Fail");
    while (1)
      ;
  }
  Serial.println("Initialized left TOF!");

  digitalWrite(XSHUT_FRONT, HIGH);
  delay(100);
  if (!tofFront.begin(0x41, &Wire)) {
    Serial.println("Front TOF Fail");
    while (1)
      ;
  }
  Serial.println("Initialized front TOF!");

  digitalWrite(XSHUT_RIGHT, HIGH);
  delay(100);
  if (!tofRight.begin(0x42, &Wire)) {
    Serial.println("Right TOF Fail");
    while (1)
      ;
  }
  Serial.println("Initialized right TOF!");

  // Initialize IMU
  Serial.println("Initializing BNO055...");
  if (!bno.begin()) {
    Serial.println("BNO055 init failed!");
    while (1)
      ;
  }
  bno.setExtCrystalUse(true);
  Serial.println("BNO055 setup OK");

  // Initialize VIVE
  vive1.begin();
  vive2.begin();
  Serial.println("Dual Vive Initialized");

  // Initialize SERVO
  ledcAttach(servoPin, servoFreq, servoResolution);
  Serial.println("Servo Initialized");

  pinMode(dirL1, OUTPUT);
  pinMode(dirL2, OUTPUT);
  pinMode(dirR1, OUTPUT);
  pinMode(dirR2, OUTPUT);

  if (!ledcAttach(pwmL, pwmFreq, pwmRes))
    Serial.println("Error: LEDC attach failed (LEFT)");

  if (!ledcAttach(pwmR, pwmFreq, pwmRes))
    Serial.println("Error: LEDC attach failed (RIGHT)");

  pinMode(encLA, INPUT_PULLUP);
  pinMode(encLB, INPUT_PULLUP);
  pinMode(encRA, INPUT_PULLUP);
  pinMode(encRB, INPUT_PULLUP);

  attachInterrupt(encLA, isrL, CHANGE);
  attachInterrupt(encLB, isrL, CHANGE);
  attachInterrupt(encRA, isrR, CHANGE);
  attachInterrupt(encRB, isrR, CHANGE);

  Serial.println("Motor Initialized");

  // Start WiFi in AP mode
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  WiFi.softAPConfig(myIP,                          // Device IP address
                    IPAddress(192, 168, 1, 1),     // gateway (not important for 5100)
                    IPAddress(255, 255, 255, 0));  // net mask
  webServer.begin(80);
  Serial.printf("Access Point started. Connect to %s\n", ssid);

  // Attach web handlers using your helper
  webServer.attachHandler("/", handleRoot);                             // Matches "GET / "
  webServer.attachHandler("/setDirection?value=", handleSetDirection);  // Matches "GET /setDirection?value="
  webServer.attachHandler("/setDutyCycle?value=", handleSetDutyCycle);  // Matches "GET /setDutyCycle?value="
  webServer.attachHandler("/spin?direction=", handleSpinDirection);     // Matches "GET /spin?direction="
  webServer.attachHandler("/turn?degree=", handleTurnDirection);        // Matches "GET /turn?degree="
  webServer.attachHandler("/stop", handleStop);                         // Matches "GET /stop"
  webServer.attachHandler("/gradedTask?type=", handleGradedTask);       // Matches "GET /gradedTask?type="
  webServer.attachHandler("/viveCoordinates?x=", handleVive);           // Matches "GET /viveCoordinates?x="
  Serial.println("Web server started. Open browser to http://" + myIP.toString());

  Serial.println("Setup complete.");
}

/* ========================================================
   MAIN LOOP
   ======================================================== */
void loop() {
  // Serve the web server
  webServer.serve();

  // ======================================================
  // READ VIVE COORDINATES
  // ======================================================
  if (readLPF(vive1, x1_f, y1_f, first1)) {
    Serial.printf("Vive Sensor 1: %.1f, %.1f\n", x1_f, y1_f);
  } else {
    vive1.sync(15);
  }

  if (readLPF(vive2, x2_f, y2_f, first2)) {
    Serial.printf("Vive Senson 2: %.1f, %.1f\n", x2_f, y2_f);
  } else {
    vive2.sync(15);
  }

  // Get VIVE based heading
  float dx = x2_f - x1_f;
  float dy = y2_f - y1_f;
  viveOrientation = atan2(dy, dx) * 180.0 / PI;
  Serial.printf("Vive Orientation: %.1f\n", viveOrientation);

  // =====================================
  // AUTONOMOUS ATTACK LOWER TOWER
  // =====================================
  if (lowerTowerActive) {
    float currX = x1_f;
    float currY = y1_f;

    switch (lowerTowerPhase) {
      // =====================================
      // PHASE 0 — GO TO (50XX, 33XX)
      // =====================================
      case 0:
        if (abs(currX - 5000) < posErr && abs(currY - 3300) < posErr) {
          stop();
          lowerTowerPhase = 1;
        } else {
          goForward(SLOW_DUTY_CYCLE, SLOW_DUTY_CYCLE);
        }
        break;

      // =====================================
      // PHASE 1 — TURN RIGHT 90°
      // =====================================
      case 1:
        {
          double target = getHeading() - 90;
          if (target < 0) target += 360;
          turnToAngle(target);
          lowerTowerPhase = 2;
        }
        break;

      // =====================================
      // PHASE 2 — GO TO (45XX, 33XX)
      // =====================================
      case 2:
        if (abs(currX - 4500) < posErr && abs(currY - 3300) < posErr) {
          stop();
          lowerTowerPhase = 3;
        } else {
          goForward(SLOW_DUTY_CYCLE, SLOW_DUTY_CYCLE);
        }
        break;

      // =====================================
      // PHASE 3 — TURN RIGHT 90°
      // =====================================
      case 3:
        {
          double target = getHeading() - 90;
          if (target < 0) target += 360;
          turnToAngle(target);
          lowerTowerPhase = 4;
          lowerTowerTimer = millis();
        }
        break;

      // =====================================
      // PHASE 4 — DRIVE STRAIGHT 20s
      // =====================================
      case 4:
        if (millis() - lowerTowerTimer > 20000) {
          stop();
          lowerTowerPhase = 5;
        } else {
          goForward(SLOW_DUTY_CYCLE, SLOW_DUTY_CYCLE);
        }
        break;

      // =====================================
      // PHASE 5 — DONE
      // =====================================
      case 5:
        stop();
        lowerTowerActive = false;
        Serial.println("Low tower captured ✔");
        break;
    }
  }

  // =====================================
  // AUTONOMOUS ATTACK HIGHER TOWER
  // =====================================
  if (higherTowerActive) {
    float currX = x1_f;
    float currY = y1_f;

    switch (higherTowerPhase) {
      // =====================================
      // PHASE 0 — GO TO (30XX, 35XX)
      // =====================================
      case 0:
        if (abs(currX - 3000) < posErr && abs(currY - 3500) < posErr) {
          stop();
          higherTowerPhase = 1;
        } else {
          goForward(BASE_DUTY_CYCLE, BASE_DUTY_CYCLE);
        }
        break;

      // =====================================
      // PHASE 1 — TURN RIGHT 90°
      // =====================================
      case 1:
        {
          double target = getHeading() - 90;
          if (target < 0) target += 360;
          turnToAngle(target);
          higherTowerPhase = 2;
          higherTowerTimer = millis();
        }
        break;

      // =====================================
      // PHASE 2 — DRIVE STRAIGHT 20s
      // =====================================
      case 2:
        if (millis() - higherTowerTimer > 20000) {
          stop();
          higherTowerPhase = 3;
        } else {
          goForward(SLOW_DUTY_CYCLE, SLOW_DUTY_CYCLE);
        }
        break;

      // =====================================
      // PHASE 3 — DONE
      // =====================================
      case 3:
        stop();
        higherTowerActive = false;
        Serial.println("High tower captured ✔");
        break;
    }
  }

  // =====================================
  // AUTONOMOUS ATTACK NEXUS TOWER
  // =====================================
  if (nexusTowerActive) {
    // Read FRONT TOF only
    int16_t frontDist = -1;
    if (tofFront.dataReady()) {
      frontDist = tofFront.distance();
      tofFront.clearInterrupt();
    }

    switch (nexusTowerPhase) {

      // =====================================
      // PHASE 0 — DRIVE FORWARD UNTIL WALL
      // =====================================
      case 0:
        if (frontDist > 0 && frontDist < 50) {
          stop();
          nexusTowerPhase = 1;
        } else {
          goForward(SLOW_DUTY_CYCLE, SLOW_DUTY_CYCLE);
        }
        break;

      // =====================================
      // PHASE 1 — TURN RIGHT 90°
      // =====================================
      case 1:
        {
          double target = getHeading() - 90;
          if (target < 0) target += 360;
          turnToAngle(target);
          nexusTowerPhase = 2;
          nexusTowerTimer = millis();
        }
        break;

      // =====================================
      // PHASE 2 — FORWARD (2s)
      // =====================================
      case 2:
        if (millis() - nexusTowerTimer >= 2000) {
          nexusTowerPhase = 3;
          nexusTowerTimer = millis();
        } else {
          goForward(SLOW_DUTY_CYCLE, SLOW_DUTY_CYCLE);
        }
        break;

      // =====================================
      // PHASE 3 — BACKWARD (1s)
      // =====================================
      case 3:
        if (millis() - nexusTowerTimer >= 1000) {
          nexusTowerHitCount++;
          if (nexusTowerHitCount >= 4) {
            nexusTowerPhase = 4;
          } else {
            nexusTowerPhase = 2;
          }
          nexusTowerTimer = millis();
        } else {
          goReverse(SLOW_DUTY_CYCLE, SLOW_DUTY_CYCLE);
        }
        break;

      // =====================================
      // PHASE 4 — DONE
      // =====================================
      case 4:
        stop();
        nexusTowerActive = false;
        Serial.println("Blue Nexus captured ✔");
        break;
    }
  }

  // =====================================
  // AUTONOMOUS ATTACK ALL 3 TOWERS
  // =====================================
  if (allTowersActive) {
    float currX = x1_f;
    float currY = y1_f;

    // Read FRONT TOF only
    int16_t frontDist = -1;
    if (tofFront.dataReady()) {
      frontDist = tofFront.distance();
      tofFront.clearInterrupt();
    }

    switch (allTowersPhase) {
      // =================================================
      // PHASE 0 — GO TO HIGH TOWER (30XX, 35XX)
      // =================================================
      case 0:
        if (abs(currX - 3000) < posErr && abs(currY - 3500) < posErr) {
          stop();
          allTowersPhase = 1;
        } else {
          goForward(BASE_DUTY_CYCLE, BASE_DUTY_CYCLE);
        }
        break;

      // =================================================
      // PHASE 1 — TURN RIGHT 90°
      // =================================================
      case 1:
        {
          double target = getHeading() - 90;
          if (target < 0) target += 360;
          turnToAngle(target);
          allTowersPhase = 2;
          allTowersTimer = millis();
        }
        break;

      // =================================================
      // PHASE 2 — FORWARD 2s (HIT HIGH TOWER)
      // =================================================
      case 2:
        if (millis() - allTowersTimer >= 2000) {
          allTowersPhase = 3;
          allTowersTimer = millis();
        } else {
          goForward(SLOW_DUTY_CYCLE, SLOW_DUTY_CYCLE);
        }
        break;

      // =================================================
      // PHASE 3 — REVERSE 1s
      // =================================================
      case 3:
        if (millis() - allTowersTimer >= 1000) {
          stop();
          allTowersPhase = 4;
        } else {
          goReverse(SLOW_DUTY_CYCLE, SLOW_DUTY_CYCLE);
        }
        break;

      // =================================================
      // PHASE 4 — TURN LEFT 90°
      // =================================================
      case 4:
        {
          double target = getHeading() + 90;
          if (target >= 360) target -= 360;
          turnToAngle(target);
          allTowersPhase = 5;
        }
        break;

      // =================================================
      // PHASE 5 — GO FORWARD UNTIL END OF TRACK (TOF)
      // =================================================
      case 5:
        if (frontDist > 0 && frontDist < 50) {
          stop();
          allTowersPhase = 6;
        } else {
          goForward(SLOW_DUTY_CYCLE, SLOW_DUTY_CYCLE);
        }
        break;

      // =================================================
      // PHASE 6 — TURN LEFT 90°
      // =================================================
      case 6:
        {
          double target = getHeading() + 90;
          if (target >= 360) target -= 360;
          turnToAngle(target);
          allTowersPhase = 7;
          allTowersTimer = millis();
        }
        break;

      // =================================================
      // PHASE 7 — FORWARD 5s (HIT NEXUS TOWER)
      // =================================================
      case 7:
        if (millis() - allTowersTimer >= 5000) {
          allTowersPhase = 8;
        } else {
          goForward(SLOW_DUTY_CYCLE, SLOW_DUTY_CYCLE);
        }
        break;

      // =================================================
      // PHASE 8 — TURN LEFT 90°
      // =================================================
      case 8:
        {
          double target = getHeading() + 90;
          if (target >= 360) target -= 360;
          turnToAngle(target);
          allTowersPhase = 9;
          allTowersTimer = millis();
        }
        break;

      // =================================================
      // PHASE 9 — FORWARD 10s (HIT LOW TOWER)
      // =================================================
      case 9:
        if (millis() - allTowersTimer >= 10000) {
          allTowersPhase = 10;
        } else {
          goForward(SLOW_DUTY_CYCLE, SLOW_DUTY_CYCLE);
        }
        break;

      // =================================================
      // PHASE 10 — DONE
      // =================================================
      case 10:
        stop();
        allTowersActive = false;
        Serial.println("Attack all towers complete ✔");
        break;
    }
    // This was our implementation for the ideal case. However, as our robot was damaged, we had to
    // hardcode (re: dead-reckon) certain parts of the above functionality. We noticed that the robot
    // always spun 180 degrees when it tried to go down the ramp, so we made sure to use that knowledge
    // to implement the "realistic" version of this task. However, we decided to keep the ideal case for
    // the submission since the below case was for a broken robot. The changes were:
    // case 3: reverse for only 0.5 seconds
    // case 4: turn left 90 degrees (save IMU orientation)
    // case 5: go forward for only 0.5 seconds (get to the edge of the ramp)
    // case 6: make sure orientation of IMU is same as before
    // case 7: go forward down the ramp for 2 seconds (fall and spin out)
    // case 8: turn 180 degrees (so faces forward again)
    // case 9: go forward for 4 seconds (hits the nexus tower's middle button)
    // case 10: go reverse for 0.5 seconds
    // case 11: turn 180 degrees (it should face the lower tower)
    // case 12: go forward for 5 seconds
  }

  if (navAxisActive) {
    float currX = x1_f;
    float currY = y1_f;

    switch (navPhase) {
      // ============================================================
      //  PHASE 0 — MOVE ALONG Y-AXIS FIRST
      // ============================================================
      case 0:
        {
          float dy = targetViveY - currY;

          if (abs(dy) < posErr) {
            stop();
            navPhase = 1;
          } else {
            goForward(SLOW_DUTY_CYCLE, SLOW_DUTY_CYCLE);
          }
          break;
        }

      // ============================================================
      //  PHASE 1 — TURN FOR X-AXIS NAVIGATION
      // ============================================================
      case 1:
        {
          float targetHeading = targetHeadingForPoint(currX, targetViveX);
          turnToAngle(getHeading() + targetHeading);
          navPhase = 2;
          break;
        }

      // ============================================================
      //  PHASE 2 — MOVE ALONG X-AXIS SECOND
      // ============================================================
      case 2:
        {
          float dx = targetViveX - currX;

          if (abs(dx) < posErr) {
            stop();
            navPhase = 3;
          } else {
            goForward(SLOW_DUTY_CYCLE, SLOW_DUTY_CYCLE);
          }
          break;
        }

      // ============================================================
      //  PHASE 3 — DONE
      // ============================================================
      case 3:
        {
          stop();
          navAxisActive = false;
          Serial.println("Axis nav complete ✔");
          break;
        }
    }
  }

  if (navTOFActive) {
    // ======================================================
    // READ TOF SENSORS
    // ======================================================
    int16_t distLeft = -1;
    int16_t distFront = -1;
    int16_t distRight = -1;

    // --- Read LEFT Sensor ---
    if (tofLeft.dataReady()) {
      distLeft = tofLeft.distance();
      if (distLeft == -1) {
        Serial.println(F("Left sensor error."));
      }
      tofLeft.clearInterrupt();
    }

    // --- Read FRONT Sensor ---
    if (tofFront.dataReady()) {
      distFront = tofFront.distance();
      if (distFront == -1) {
        Serial.println(F("Front sensor error."));
      }
      tofFront.clearInterrupt();
    }

    // --- Read RIGHT Sensor ---
    if (tofRight.dataReady()) {
      distRight = tofRight.distance();
      if (distRight == -1) {
        Serial.println(F("Right sensor error."));
      }
      tofRight.clearInterrupt();
    }

    // Decide sensor availability
    bool leftOpen = (distLeft == -1 || distLeft > CLEAR_DIST_MM);
    bool frontOpen = (distFront == -1 || distFront > FRONT_STOP_MM);
    bool rightOpen = (distRight == -1 || distRight > CLEAR_DIST_MM);

    // ======================================
    // RULE 1: Left wall gone → turn left
    // ======================================
    if (leftOpen) {
      // Always choose left if available
      Serial.println("TURNING LEFT");
      double target = getHeading() + 90;
      if (target >= 360) target -= 360;

      turnToAngle(target);
    }

    // ======================================
    // RULE 2: Front open → forward with correction
    // ======================================
    else if (frontOpen) {
      // Go straight if you can
      Serial.println("GOING FORWARD");

      // ==================================================
      // LOW-LEVEL SMOOTH WALL FOLLOWING (LEFT WALL)
      // ==================================================

      // Positive: far, Negative: close
      int error = distLeft - WALL_DIST_MM;

      // Calculate the duty cycle correction term
      int correction = error * 0.05;

      // Update the motor duty cycles based on correction term
      int leftDutyCycle = BASE_DUTY_CYCLE - correction;
      int rightDutyCycle = BASE_DUTY_CYCLE + correction;
      Serial.printf("Forward L=%d R=%d\n", leftDutyCycle, rightDutyCycle);

      // Set new forward duty cycle for both motors
      goForward(leftDutyCycle, rightDutyCycle);
    }

    // ======================================
    // RULE 3: Front blocked & left blocked → turn right
    // ======================================
    else if (rightOpen) {
      // Last resort option before backing up
      Serial.println("TURNING RIGHT");
      double target = getHeading() - 90;
      if (target < 0) target += 360;

      turnToAngle(target);
    }

    // ======================================
    // RULE 4: Everything blocked → go reverse
    // ======================================
    else {
      // Dead end
      Serial.println("GOING REVERSE");
      goReverse(SLOW_DUTY_CYCLE, SLOW_DUTY_CYCLE);
    }
  }

  // ======================================================
  // PID + SMOOTH RAMP
  // ======================================================

  // Calculate the target duty cycle for each motor
  double targetDutyCycleL = rampTo(outputL, goalDutyCycleL);
  double targetDutyCycleR = rampTo(outputR, goalDutyCycleR);

  // Get encoder position reading
  long currPosL = posL;
  long currPosR = posR;

  // Get speed from encoder value
  speedL = ((currPosL - prevPosL) / TICKS_AT_100_DUTY) * 100.0;
  speedR = ((currPosR - prevPosR) / TICKS_AT_100_DUTY) * 100.0;

  // Update previous position value
  prevPosL = currPosL;
  prevPosR = currPosR;

  // Clamp speed duty cycle
  speedL = constrain(speedL, 0, 100);
  speedR = constrain(speedR, 0, 100);

  // Set input for PID to be current speed
  inputL = speedL;
  inputR = speedR;

  // Compute PID for both motors
  computePID(inputL, outputL, targetDutyCycleL, prevErrorL, integralL);
  computePID(inputR, outputR, targetDutyCycleR, prevErrorR, integralR);

  // Set required PWM for both motors
  setMotor(pwmL, outputL);
  setMotor(pwmR, outputR);

  // ======================================================
  // TOP HAT + SERVO UPDATE
  // ======================================================
  unsigned long currentMillis = millis();

  // Update Servo Location
  if (currentMillis - previousMillisServo >= halfPeriodMs) {
    previousMillisServo = currentMillis;

    if (movingForward) {
      ledcWrite(servoPin, servoDutyFromUs(maxUs));
      Serial.println("Turning Servo to 180 degrees");
    } else {
      ledcWrite(servoPin, servoDutyFromUs(minUs));
      Serial.println("Turning Servo to 90 degrees");
    }

    movingForward = !movingForward;
  }

  // Update Top Hat Health
  if (currentMillis - previousMillisTopHat >= topHatPacketPeriod) {
    // Update previous time
    previousMillisTopHat = currentMillis;

    // Send data to top hat
    sendWiFiPacketsByte(packets);

    // Get data from top hat
    robotHealth = receiveHealthValueByte();
    if (robotHealth == 0) stop();

    // Reset packets count
    packets = 0;
  }

  // Wait 50ms before running again
  delay(50);
}
