// ==========================================================================
//  ADVANCED WALL FOLLOWING + 2-WHEEL PID DRIVE + TOF + 9-DOF IMU + VIVE
//  ESP32-S3  (VL53L1X + BNO055 + Encoders + PID + VIVE + Ramp Acceleration)
// ==========================================================================

#include <Wire.h>
#include "Adafruit_VL53L1X.h"
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <utility/imumaths.h>
#include <WiFi.h>
#include "html510.h"
#include "vive510.h"

/* ========================================================
   WIFI AND WEBSITE
   ======================================================== */

const char *ssid = "Ani_WiFi";
const char *password = "12345678";

IPAddress myIP(192, 168, 1, 147);  // change to your IP
HTML510Server webServer(80);

double headingDisplay = 0;
double viveHeadingDisplay = 0;
double pwmDisplayL = 0;
double pwmDisplayR = 0;

String currentGradedTask = "";
bool gradedTaskActive = false;
bool toggleTOF = false;
int gradedPhase = 0;
unsigned long phaseStart = 0;

// HTML
String pageHtml() {
  return R"rawliteral(
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
                <p><b>Vive X:</b> <span id="viveX">0</span></p>
                <p><b>Vive Y:</b> <span id="viveY">0</span></p>
                <p><b>Heading:</b> <span id="headingVal">0</span>°</p>
                <p><b>Vive Heading:</b> <span id="viveHeading">0</span>°</p>
                <p><b>PWM Left:</b> <span id="pwmL">0</span>%</p>
                <p><b>PWM Right:</b> <span id="pwmR">0</span>%</p>
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
                <h3>Attack Controls</h3>
                <button class="btn" onclick="attack('lower_tower')">Attack Lower Tower</button>
                <button class="btn" onclick="attack('nexus')">Attack Nexus</button>
                <button class="btn" onclick="attack('higher_tower')">Attack Higher Tower</button>
            </div>

            <div class="box">
                <h3>Defend Controls</h3>
                <button class="btn" onclick="defend('lower_tower')">Defend Lower Tower</button>
                <button class="btn" onclick="defend('nexus')">Defend Nexus</button>
                <button class="btn" onclick="defend('higher_tower')">Defend Higher Tower</button>
            </div>

            <div class="box">
                <h3>Graded Tasks</h3>
                <button class="btn" onclick="gradedTask('left_wall')">Full Circuit (Left Wall Follow)</button>
                <button class="btn" onclick="gradedTask('right_wall')">Full Circuit (Right Wall Follow)</button>
                <button class="btn" onclick="gradedTask('blue_high_tower')">Blue High Tower</button>
                <button class="btn" onclick="gradedTask('blue_middle_tower')">Blue Middle Tower</button>
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
                    fetch('/setDuty?value=' + v)
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

                function attack(target) {
                    fetch(`/attack?target=${target}`)
                        .catch((e) => console.log('err', e));
                }

                function defend(target) {
                    fetch(`/defend?target=${target}`)
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

                // Telemetry auto-refresh
                setInterval(() => {
                    fetch('/telemetry')
                        .then(r => r.json())
                        .then(data => {
                            document.getElementById('viveX').innerText = data.viveX;
                            document.getElementById('viveY').innerText = data.viveY;
                            document.getElementById('headingVal').innerText = data.heading.toFixed(1);
                            document.getElementById('viveHeading').innerText = data.viveHeading.toFixed(1);
                            document.getElementById('pwmL').innerText = data.pwmL.toFixed(1);
                            document.getElementById('pwmR').innerText = data.pwmR.toFixed(1);
                        })
                        .catch(e => console.log('Telemetry error', e));
                }, 500);
            </script>
        </body>
    </html>
  )rawliteral";
}

/* ========================================================
   TOF AND IMU SENSOR DEFINITIONS
   ======================================================== */
#define SDA_PIN 36
#define SCL_PIN 37

Adafruit_VL53L1X tofLeft  = Adafruit_VL53L1X();
Adafruit_VL53L1X tofFront = Adafruit_VL53L1X();
Adafruit_VL53L1X tofRight = Adafruit_VL53L1X();

// Need to update the address to be something else later on
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

#define XSHUT_LEFT 12
#define XSHUT_FRONT 13
#define XSHUT_RIGHT 14

#define WALL_DIST_MM 100   // Desired wall following distance (10 cm)
#define FRONT_STOP_MM 150  // Stop/turn if front < 15 cm
#define CLEAR_DIST_MM 500  // “Open space” threshold

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

float targetViveX = 0;
float targetViveY = 0;

bool navAxisActive = false;
int navPhase = 0;  // 0 = move X, 1 = turn, 2 = move Y, 3 = done

float posTol = 300;  // acceptable Vive unit error (~15 cm)

/* ========================================================
   MOTOR + ENCODER DEFINITIONS
   ======================================================== */
const int pwmFreq = 2000;
const int pwmRes = 8;
const int ledcRes = ((1 << pwmRes) - 1);
int BASE_DUTY_CYCLE = 80;
int TURN_DUTY_CYCLE = 50;

// LEFT MOTOR
const int dirL1 = 6;
const int dirL2 = 7;
const int pwmL = 15;

// RIGHT MOTOR
const int dirR1 = 42;
const int dirR2 = 41;
const int pwmR = 40;

// ENCODERS
const int encLA = 16;
const int encLB = 17;

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

  Serial.print("PWM Motor: ");
  Serial.print(pwmPin == 15 ? "Left" : "Right");
  Serial.print(" PWM Duty Dycle: ");
  Serial.println(pwmDutyCycleValue);
}

/* ========================================================
   MOVEMENT COMMANDS
   ======================================================== */
void resetPID() {
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

void goBack(int dutyCycleL, int dutyCycleR) {
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

    // Scale this down if we can
    if (abs(error) < 5) {
      stop();
      delay(100);
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

void turnToAngleSlow(double target) {
  double Kp = 1.0;
  double Kd = 0.2;
  double prevErr = 0;

  while (true) {
    double heading = getHeading();

    double error = target - heading;
    if (error > 180) error -= 360;
    if (error < -180) error += 360;

    if (abs(error) < 5) {  // close enough
      stop();
      break;
    }

    double derivative = error - prevErr;
    prevErr = error;

    double correction = Kp * error + Kd * derivative;
    correction = constrain(correction, -40, 40);

    if (correction > 0) turnLeft(abs(correction), abs(correction));
    else turnRight(abs(correction), abs(correction));

    setMotor(pwmL, abs(correction));
    setMotor(pwmR, abs(correction));

    delay(20);
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

float targetHeadingForX(float currentX, float targetX) {
  if (targetX > currentX) return 0.0;  // +X direction
  else return 180.0;                   // -X direction
}

float targetHeadingForY(float currentY, float targetY) {
  if (targetY > currentY) return 90.0;  // +Y direction
  else return 270.0;                    // -Y direction
}

/* ========================================================
   GRADED TASK STATE MACHINES
   ======================================================== */
void graded_BlueHighTower() {
  Serial.println("[GRADED] Attack High Tower activated");
  // TODO: implement:
  // big path with multiple waypoints & turns
}

void graded_BlueMiddleTower() {
  Serial.println("[GRADED] Attack Middle Tower activated");
  // TODO: implement:
  // big path with multiple waypoints & turns
}

void graded_BlueNexus() {
  Serial.println("[GRADED] Attack Blue Nexus activated");
  // TODO: implement:
  // big path with multiple waypoints & turns
}

void graded_AttackEverything() {
  Serial.println("[GRADED] Attack Everything activated");
  // TODO: implement:
  // big path with multiple waypoints & turns
}

/* ========================================================
   WEBSITE COMMANDS
   ======================================================== */
// Handle root
void handleRoot() {
  webServer.sendhtml(pageHtml());
}

void handleTelemetry() {
  String json = "{";
  json += "\"viveX\":" + String(targetViveX, 2) + ",";
  json += "\"viveY\":" + String(targetViveY, 2) + ",";
  json += "\"heading\":" + String(headingDisplay, 2) + ",";
  json += "\"viveHeading\":" + String(viveHeadingDisplay, 2) + ",";
  json += "\"pwmL\":" + String(pwmDisplayL, 1) + ",";
  json += "\"pwmR\":" + String(pwmDisplayR, 1);
  json += "}";

  webServer.sendplain(json);
}

// Handle direction updates
void handleSetDirection() {
  // Get new direction
  String txt = webServer.getText();
  bool isForward = txt.equals("forward");

  // Update motor direction pins
  isForward ? goForward(BASE_DUTY_CYCLE, BASE_DUTY_CYCLE) : goBack(TURN_DUTY_CYCLE, TURN_DUTY_CYCLE);

  // Reply
  webServer.sendplain(String("OK direction=" + String(txt)));
  Serial.printf("Direction set to %s\n", txt);
}

// Handle duty cycle updates
void handleSetDuty() {
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
}w

// Handle spin direction updates
void handleSpinDirection() {
  // Get new direction
  String txt = webServer.getText();
  bool isLeft = txt.equals("left");

  // Update motor direction pins
  isLeft ? turnLeft(TURN_DUTY_CYCLE, TURN_DUTY_CYCLE) : turnRight(TURN_DUTY_CYCLE, TURN_DUTY_CYCLE);

  // Reply
  webServer.sendplain(String("OK direction=" + String(txt)));
  Serial.printf("Direction set to %s\n", txt);
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
}

// Handle stop command updates
void handleStop() {
  // Stop the car
  stop();

  // Reply
  webServer.sendplain("OK stopped the car");
  Serial.printf("Stopped the car");
}

// Handle attack target updates
void handleAttack() {
  // Get new target
  String txt = webServer.getText();

  // Reply
  webServer.sendplain(String("OK target=" + String(txt)));
  Serial.printf("Target set to %s\n", txt);
}

// Handle defend target updates
void handleDefend() {
  // Get new target
  String txt = webServer.getText();

  // Reply
  webServer.sendplain(String("OK target=" + String(txt)));
  Serial.printf("Target set to %s\n", txt);
}

// Handle graded task updates
void handleGradedTask() {
  String txt = webServer.getText();
  Serial.printf("Graded Task Requested: %s\n", txt.c_str());

  if (txt == "blue_high_tower") {
    navAxisActive = false;
    toggleTOF = false;
    gradedTaskActive = true;
    currentGradedTask = txt;
  }
  else if (txt == "blue_middle_tower") {
    navAxisActive = false;
    toggleTOF = false;
    gradedTaskActive = true;
    currentGradedTask = txt;
  }
  else if (txt == "blue_nexus") {
    navAxisActive = false;
    toggleTOF = false;
    gradedTaskActive = true;
    currentGradedTask = txt;
  }
  else if (txt == "attack_everything") {
    navAxisActive = false;
    toggleTOF = false;
    gradedTaskActive = true;
    currentGradedTask = txt;
  }
  else if (txt == "left_wall" || txt == "right_wall") {
    toggleTOF = true;
    navAxisActive = false;
    gradedTaskActive = false;
  }
  else {
    Serial.println("Unknown graded task.");
  }

  webServer.sendplain("OK graded task");
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
  toggleTOF = false;
  gradedTaskActive = false;
  navPhase = 0;

  Serial.printf("VIVE COORDINATES:  X=%.2f   Y=%.2f\n", targetViveX, targetViveY);

  webServer.sendplain("OK vive received");
}

/* ========================================================
   SETUP
   ======================================================== */
void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  // Start WiFi in AP mode
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  WiFi.softAPConfig(myIP,                          // Device IP address
                    IPAddress(192, 168, 1, 1),     // gateway (not important for 5100)
                    IPAddress(255, 255, 255, 0));  // net mask
  Serial.printf("Access Point started. Connect to %s\n", ssid);

  // Attach web handlers using your helper
  webServer.attachHandler("/", handleRoot);                             // Matches "GET / "
  webServer.attachHandler("/setDirection?value=", handleSetDirection);  // Matches "GET /setDirection?value="
  webServer.attachHandler("/setDuty?value=", handleSetDuty);            // Matches "GET /setDuty?value="
  webServer.attachHandler("/spin?direction=", handleSpinDirection);     // Matches "GET /spin?direction="
  webServer.attachHandler("/turn?degree=", handleTurnDirection);        // Matches "GET /turn?degree="
  webServer.attachHandler("/stop", handleStop);                         // Matches "GET /stop"
  webServer.attachHandler("/attack?target=", handleAttack);             // Matches "GET /attack?target="
  webServer.attachHandler("/defend?target=", handleDefend);             // Matches "GET /defend?target="
  webServer.attachHandler("/gradedTask?type=", handleGradedTask);       // Matches "GET /gradedTask?type="
  webServer.attachHandler("/viveCoordinates?x=", handleVive);           // Matches "GET /viveCoordinates?x="
  webServer.attachHandler("/telemetry", handleTelemetry);               // Matches "GET /telemetry"
  webServer.begin(80);
  Serial.println("Web server started. Open browser to http://" + myIP.toString());

  Wire.begin(SDA_PIN, SCL_PIN);

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

  digitalWrite(XSHUT_FRONT, HIGH);
  delay(100);
  if (!tofFront.begin(0x41, &Wire)) {
    Serial.println("Front TOF Fail");
    while (1)
      ;
  }

  digitalWrite(XSHUT_RIGHT, HIGH);
  delay(100);
  if (!tofRight.begin(0x42, &Wire)) {
    Serial.println("Right TOF Fail");
    while (1)
      ;
  }

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
    Serial.printf("S1: %.1f, %.1f\n", x1_f, y1_f);
  } else {
    vive1.sync(15);
  }

  if (readLPF(vive2, x2_f, y2_f, first2)) {
    Serial.printf("S2: %.1f, %.1f\n", x2_f, y2_f);
  } else {
    vive2.sync(15);
  }

  // Get VIVE based heading
  float dx = x2_f - x1_f;
  float dy = y2_f - y1_f;
  viveHeadingDisplay = atan2(dy, dx) * 180.0 / PI;

  if (navAxisActive) {
    float currX = x1_f;
    float currY = y1_f;

    switch (navPhase) {

      // ============================================================
      //  PHASE 0 — CHOOSE ORDER
      // ============================================================
      case 0:
        {
          int firstAxis = decideAxisMovement(currY);

          if (firstAxis == 1) {
            Serial.println("Routing: Y first, then X");
            navPhase = 1;
          } else {
            Serial.println("Routing: X first, then Y");
            navPhase = 5;
          }
          break;
        }

      // ============================================================
      //  PHASE 1 — MOVE Y FIRST
      // ============================================================
      case 1:
        {
          float targetH = targetHeadingForY(currY, targetViveY);
          double currH = getHeading();
          float err = targetH - currH;
          if (err > 180) err -= 360;
          if (err < -180) err += 360;

          if (abs(err) > 4) {
            turnToAngleSlow(targetH);
            break;
          }

          float dy = targetViveY - currY;
          if (abs(dy) < posTol) {
            stop();
            navPhase = 2;  // now turn for X
            delay(200);
          } else {
            goForward(BASE_DUTY_CYCLE, BASE_DUTY_CYCLE);
          }
          break;
        }

      // ============================================================
      //  PHASE 2 — TURN FOR X
      // ============================================================
      case 2:
        {
          float targetH = targetHeadingForX(currX, targetViveX);
          turnToAngleSlow(targetH);
          navPhase = 3;
          delay(200);
          break;
        }

      // ============================================================
      //  PHASE 3 — MOVE IN X
      // ============================================================
      case 3:
        {
          float targetH = targetHeadingForX(currX, targetViveX);
          double currH = getHeading();

          float err = targetH - currH;
          if (err > 180) err -= 360;
          if (err < -180) err += 360;

          if (abs(err) > 4) {
            turnToAngleSlow(targetH);
            break;
          }

          float dx = targetViveX - currX;
          if (abs(dx) < posTol) {
            stop();
            navPhase = 4;
            delay(200);
          } else {
            goForward(BASE_DUTY_CYCLE, BASE_DUTY_CYCLE);
          }
          break;
        }

      // ============================================================
      //  PHASE 4 — DONE
      // ============================================================
      case 4:
        stop();
        navAxisActive = false;
        Serial.println("Axis nav complete ✔");
        break;


      // ============================================================
      //  PHASE 5 — MOVE X FIRST (obstacle avoidance mode)
      // ============================================================
      case 5:
        {
          float targetH = targetHeadingForX(currX, targetViveX);
          double currH = getHeading();
          float err = targetH - currH;
          if (err > 180) err -= 360;
          if (err < -180) err += 360;

          if (abs(err) > 4) {
            turnToAngleSlow(targetH);
            break;
          }

          float dx = targetViveX - currX;
          if (abs(dx) < posTol) {
            stop();
            navPhase = 6;  // now turn for Y
            delay(200);
          } else {
            goForward(BASE_DUTY_CYCLE, BASE_DUTY_CYCLE);
          }
          break;
        }

      // ============================================================
      //  PHASE 6 — TURN FOR Y
      // ============================================================
      case 6:
        {
          float targetH = targetHeadingForY(currY, targetViveY);
          turnToAngleSlow(targetH);
          navPhase = 7;
          delay(200);
          break;
        }

      // ============================================================
      //  PHASE 7 — MOVE Y LAST
      // ============================================================
      case 7:
        {
          float targetH = targetHeadingForY(currY, targetViveY);
          double currH = getHeading();

          float err = targetH - currH;
          if (err > 180) err -= 360;
          if (err < -180) err += 360;

          if (abs(err) > 4) {
            turnToAngleSlow(targetH);
            break;
          }

          float dy = targetViveY - currY;
          if (abs(dy) < posTol) {
            stop();
            navPhase = 4;  // finish
            delay(200);
          } else {
            goForward(BASE_DUTY_CYCLE, BASE_DUTY_CYCLE);
          }
          break;
        }
    }
  }

  if (toggleTOF) {
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
      // Clear the interrupt status for the next reading
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
    bool leftOpen  = (distLeft == -1 || distLeft > CLEAR_DIST_MM);
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
    // RULE 4: Everything blocked → reverse
    // ======================================
    else {
      // Dead end
      Serial.println("Maze logic: DEAD END → reverse");
      goBack(TURN_DUTY_CYCLE, TURN_DUTY_CYCLE);
    }
  }

  if (gradedTaskActive) {
    if (currentGradedTask == "blue_high_tower") {
        graded_BlueHighTower();
    }
    else if (currentGradedTask == "blue_middle_tower") {
        graded_BlueMiddleTower();
    }
    else if (currentGradedTask == "blue_nexus") {
        graded_BlueNexus();
    }
    else if (currentGradedTask == "attack_everything") {
        graded_AttackEverything();
    }
  }

  // ======================================================
  // PID + SMOOTH RAMP
  // ======================================================

  // Calculate the target duty cycle for each motor
  double targetDutyCycleL = rampTo(outputL, goalDutyCycleL);
  double targetDutyCycleR = rampTo(outputR, goalDutyCycleR);

  // Get encoder position reading
  inputL = posL;
  inputR = posR;

  // Compute PID for both motors
  computePID(inputL, outputL, targetDutyCycleL, prevErrorL, integralL);
  computePID(inputR, outputR, targetDutyCycleR, prevErrorR, integralR);

  // Set required PWM for both motors
  setMotor(pwmL, outputL);
  setMotor(pwmR, outputR);

  // Get display details
  headingDisplay = getHeading();
  pwmDisplayL = outputL;
  pwmDisplayR = outputR;

  // Wait 50ms before running again
  delay(50);
}
