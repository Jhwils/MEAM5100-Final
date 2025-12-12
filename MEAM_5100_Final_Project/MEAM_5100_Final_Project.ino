// /************************************************************
//  *  ESP32 AUTONOMOUS CAR SYSTEM
//  *  Organized Codebase (cleaned but logically identical)
//  ************************************************************/

// // ==========================================================
// // ====================== INCLUDES ===========================
// // ==========================================================
// #include <WiFi.h>
// #include "html510.h"
// #include <ESP32Servo.h>

// #include "vive510.h"
// #include <VL53L0X.h>
// #include <Wire.h>

// // ==========================================================
// // ====================== CONSTANTS ==========================
// // ==========================================================

// #define TOPHAT_I2C_ADDR 0x28   // Modify if needed

// // WiFi AP settings
// const char* ssid     = "Ani_WiFi";
// const char* password = "12345678";
// IPAddress myIP(192, 168, 1, 147);

// // Vive pins
// Vive510 viveX(34);
// Vive510 viveY(35);

// // TOF sensors
// VL53L0X tofLeft;
// VL53L0X tofFront;
// VL53L0X tofRight;

// // Global flags
// bool wifiCommandReceived = false;
// unsigned long lastTopHatSend = 0;

// // Wall-follow defaults
// int followSide = 0;          // 0 = left, 1 = right
// int targetDist = 150;        // mm

// // Web server
// HTML510Server webServer(80);

// // ==========================================================
// // ===================== HTML PAGE ===========================
// // ==========================================================
// String pageHtml() {
//     return R"rawliteral(
//       <html>
//           <head>
//               <meta charset="utf-8"/>
//               <title>ESP32 PWM Control</title>
//               <style>
//                   body { font-family: Arial, sans-serif; text-align: center; background: #f9f9f9; }
//                   .slider { width: 80%; margin: 8px 0; }
//                   .box { background: white; padding: 16px; margin: 30px auto; width: 90%; max-width: 600px; border-radius: 8px; box-shadow: 0 2px 6px rgba(0, 0, 0, 0.1); }
//                   p { margin: 6px 0; }

//                   .switch { position: relative; display: inline-block; width: 60px; height: 34px; }
//                   .switch input { opacity: 0; width: 0; height: 0; }
//                   .slider-toggle {
//                       position: absolute; cursor: pointer; top: 0; left: 0;
//                       right: 0; bottom: 0; background-color: #ccc;
//                       transition: .4s; border-radius: 34px;
//                   }
//                   .slider-toggle:before {
//                       position: absolute; content: "";
//                       height: 26px; width: 26px; left: 4px; bottom: 4px;
//                       background-color: white; transition: .4s; border-radius: 50%;
//                   }
//                   input:checked + .slider-toggle { background-color: #4CAF50; }
//                   input:checked + .slider-toggle:before { transform: translateX(26px); }

//                   .btn {
//                       padding: 10px 20px; margin: 8px; border-radius: 8px;
//                       background-color: #4CAF50; color: white; border: none;
//                       cursor: pointer; font-size: 16px;
//                   }
//                   .btn-red { background-color: #f44336; }
//                   .btn:hover { opacity: 0.9; }
//                   .btn-group { display: flex; justify-content: center; margin: 10px 0; }

//                   .coord-input { font-size: 16px; margin: 8px; }
//                   .coord-group { display: inline-block; margin: 10px 0; }
//               </style>
//           </head>

//           <body>
//               <div class="box">
//                   <h2>ESP32 Autonomous Car</h2>
//               </div>

//               <div class="box">
//                   <h3>Maneuver Controls</h3>

//                   <p>Direction: <span id="dirTxt">Reverse</span></p>
//                   <label class="switch">
//                       <input id="dirSwitch" type="checkbox" onchange="toggleDirection(this)">
//                       <span class="slider-toggle"></span>
//                   </label>

//                   <p>Speed: <span id="speedTxt">0</span>%</p>
//                   <input id="speedSlider" class="slider" type="range" min="0" max="100" step="1" value="0"
//                       onchange="setSpeed(this.value)"
//                       oninput="document.getElementById('speedTxt').innerText=this.value">

//                   <div class="btn-group">
//                       <button class="btn" onclick="turn(90, 'left')">90° Left</button>
//                       <button class="btn" onclick="turn(90, 'right')">90° Right</button>
//                   </div>

//                   <div class="btn-group">
//                       <button class="btn" onclick="turn(180, 'left')">180° Left</button>
//                       <button class="btn" onclick="turn(180, 'right')">180° Right</button>
//                   </div>

//                   <button class="btn btn-red" onclick="stop()">Stop</button>
//               </div>

//               <div class="box">
//                   <h3>Attack Controls</h3>
//                   <button class="btn" onclick="attack('lower_tower')">Attack Lower Tower</button>
//                   <button class="btn" onclick="attack('nexus')">Attack Nexus</button>
//                   <button class="btn" onclick="attack('higher_tower')">Attack Higher Tower</button>
//               </div>

//               <div class="box">
//                   <h3>Defend Controls</h3>
//                   <button class="btn" onclick="defend('lower_tower')">Defend Lower Tower</button>
//                   <button class="btn" onclick="defend('nexus')">Defend Nexus</button>
//                   <button class="btn" onclick="defend('higher_tower')">Defend Higher Tower</button>
//               </div>

//               <div class="box">
//                   <h3>Graded Tasks</h3>
//                   <button class="btn" onclick="gradedTask('left_wall')">Full Circuit (Left Wall Follow)</button>
//                   <button class="btn" onclick="gradedTask('right_wall')">Full Circuit (Right Wall Follow)</button>
//               </div>

//               <div class="box">
//                   <h3>VIVE Location</h3>
//                   <div class="coord-group">
//                       <label class="coord-input">X:</label>
//                       <input id="xCoord" type="number" class="coord-input" placeholder="X"/>
//                   </div>
//                   <div class="coord-group">
//                       <label class="coord-input">Y:</label>
//                       <input id="yCoord" type="number" class="coord-input" placeholder="Y"/>
//                   </div>
//                   <button class="btn" onclick="sendViveCoordinates()">Go</button>
//               </div>

//               <script>
//                   let direction = "reverse";

//                   function toggleDirection(el) {
//                       direction = el.checked ? "forward" : "reverse";
//                       document.getElementById('dirTxt').innerText = direction.charAt(0).toUpperCase() + direction.slice(1);
//                       setDirection(direction);
//                   }

//                   function setDirection(dir) { fetch('/setDirection?value=' + dir); }
//                   function setSpeed(v) { fetch('/setDuty?value=' + v); }

//                   function turn(degree, direction) {
//                       fetch(`/turn?degree=${degree}&direction=${direction}`);
//                   }

//                   function stop() { fetch('/stop'); }
//                   function attack(target) { fetch(`/attack?target=${target}`); }
//                   function defend(target) { fetch(`/defend?target=${target}`); }
//                   function gradedTask(taskType) { fetch(`/gradedTask?type=${taskType}`); }

//                   function sendViveCoordinates() {
//                       const x = document.getElementById('xCoord').value;
//                       const y = document.getElementById('yCoord').value;
//                       fetch(`/viveCoordinates?x=${x}&y=${y}`);
//                   }
//               </script>
//           </body>
//       </html>
//     )rawliteral";
// }

// // ==========================================================
// // ===================== SERVO CONTROL =======================
// // ==========================================================

// // Swing servo (90°)
// Servo servoSwing;
// int servoSwingPin = 9;

// unsigned long lastSwingUpdate = 0;
// int swingPos = 0;
// int swingDirection = 1;
// int swingMin = 0;
// int swingMax = 180;
// int swingStep = 10;
// int swingInterval = 30;

// // 360° servo
// Servo servoRotate;
// int servoRotatePin = 10;

// bool rotateTriggered = false;
// int rotateTargetAngle = 0;
// unsigned long lastRotateUpdate = 0;
// int rotateInterval = 80;
// int rotateCurrentAngle = 0;

// // ==========================================================
// // ===================== MOTOR SYSTEM ========================
// // ==========================================================

// struct MotorPins {
//     int pwm;
//     int forward;
//     int reverse;
// };

// MotorPins motorPins[4] = {
//     {4, 18, 19},    // LF
//     {25, 26, 27},   // RF
//     {14, 12, 13},   // LR
//     {32, 33, 15}    // RR
// };

// int encoderPins[4] = {5, 17, 16, 21};

// volatile long encoderCounts[4] = {0, 0, 0, 0};

// float wheelRPM[4] = {0, 0, 0, 0};
// float targetRPM[4] = {0, 0, 0, 0};

// float Kp = 0.8, Ki = 0.15, Kd = 0.05;
// float pidIntegral[4] = {0,0,0,0};
// float lastError[4] = {0,0,0,0};

// int pwmRes = 8;
// int pwmMax = (1 << pwmRes) - 1;
// int pwmFreq = 200;

// int edgesPerRev = 20;

// // ==========================================================
// // ================= ENCODER INTERRUPTS ======================
// // ==========================================================
// void IRAM_ATTR enc0() { encoderCounts[0]++; }
// void IRAM_ATTR enc1() { encoderCounts[1]++; }
// void IRAM_ATTR enc2() { encoderCounts[2]++; }
// void IRAM_ATTR enc3() { encoderCounts[3]++; }

// // ==========================================================
// // ================= MOTOR CONTROL HELPERS ===================
// // ==========================================================

// void setMotorDirection(int id, bool forward) {
//     digitalWrite(motorPins[id].forward, forward ? HIGH : LOW);
//     digitalWrite(motorPins[id].reverse, forward ? LOW : HIGH);
// }

// void setMotorPWM(int id, int val) {
//     val = constrain(val, 0, pwmMax);
//     ledcWrite(motorPins[id].pwm, val);
// }

// void updateWheelRPM() {
//     static long lastCounts[4] = {0,0,0,0};

//     for (int i = 0; i < 4; i++) {
//         long diff = encoderCounts[i] - lastCounts[i];
//         lastCounts[i] = encoderCounts[i];

//         wheelRPM[i] = (diff * 50.0 * 60.0) / edgesPerRev; // 50Hz update
//     }
// }

// void updatePID() {
//     for (int i = 0; i < 4; i++) {
//         float error = targetRPM[i] - wheelRPM[i];
//         pidIntegral[i] += error * 0.02;
//         float derivative = (error - lastError[i]) / 0.02;
//         lastError[i] = error;

//         float out = Kp * error + Ki * pidIntegral[i] + Kd * derivative;

//         setMotorDirection(i, out >= 0);
//         setMotorPWM(i, abs(out));
//     }
// }

// // ==========================================================
// // ================= HIGH LEVEL MOTION API ==================
// // ==========================================================

// float baseRPM = 0;

// void applyBaseSpeed() {
//     for (int i = 0; i < 4; i++) targetRPM[i] = baseRPM;
// }

// void fullStop() {
//     for (int i = 0; i < 4; i++) targetRPM[i] = 0;
// }

// void turnInPlace(int degree, String dir) {
//     float turnRPM = 80;

//     if (dir == "left") {
//         targetRPM[0] = -turnRPM;
//         targetRPM[2] = -turnRPM;
//         targetRPM[1] =  turnRPM;
//         targetRPM[3] =  turnRPM;
//     } else {
//         targetRPM[0] =  turnRPM;
//         targetRPM[2] =  turnRPM;
//         targetRPM[1] = -turnRPM;
//         targetRPM[3] = -turnRPM;
//     }
// }

// // ==========================================================
// // ===================== WEB HANDLERS ========================
// // ==========================================================

// void handleRoot() { webServer.sendhtml(pageHtml()); }

// void handleSetDirection() {
//     wifiCommandReceived = true;
//     bool forward = webServer.getText().equals("forward");

//     baseRPM = forward ? abs(baseRPM) : -abs(baseRPM);
//     applyBaseSpeed();

//     webServer.sendplain("OK");
// }

// void handleSetDuty() {
//     wifiCommandReceived = true;
//     int pct = constrain(webServer.getText().toInt(), 0, 100);

//     baseRPM = pct * 1.5;
//     applyBaseSpeed();

//     webServer.sendplain("OK");
// }

// void handleTurn() {
//     wifiCommandReceived = true;
//     turnInPlace(90, "left");   // Placeholder
//     webServer.sendplain("OK turn");
// }

// void handleStop() {
//     wifiCommandReceived = true;
//     fullStop();
//     webServer.sendplain("OK stop");
// }

// void handleSafetyServo() {
//     wifiCommandReceived = true;
//     int angle = constrain(webServer.getText().toInt(), 0, 360);
//     triggerSafetyServo(angle);
//     webServer.sendplain("OK safety");
// }

// void handleAttack() { webServer.sendplain("OK attack"); }
// void handleDefend() { webServer.sendplain("OK defend"); }
// void handleGradedTask() { webServer.sendplain("OK graded"); }
// void handleViveCoordinates() { webServer.sendplain("OK vive"); }

// void handleGetCounts() {
//     webServer.sendplain(String(encoderCounts[0]));
// }

// // ==========================================================
// // ==================== SETUP ===============================
// // ==========================================================

// void setup() {
//     Serial.begin(115200);

//     // WiFi AP
//     WiFi.mode(WIFI_AP);
//     WiFi.softAP(ssid, password);
//     WiFi.softAPConfig(myIP, IPAddress(192,168,1,1), IPAddress(255,255,255,0));

//     // Vive
//     viveX.begin();
//     viveY.begin();

//     // TOF sensors
//     tofLeft.init();  tofLeft.setTimeout(100);
//     tofFront.init(); tofFront.setTimeout(100);
//     tofRight.init(); tofRight.setTimeout(100);

//     // Servos
//     servoSwing.attach(servoSwingPin);
//     servoRotate.attach(servoRotatePin);

//     servoSwing.write(90);
//     servoRotate.write(0);

//     // I2C
//     Wire.begin();

//     // Motors
//     for (int i = 0; i < 4; i++) {
//         pinMode(motorPins[i].forward, OUTPUT);
//         pinMode(motorPins[i].reverse, OUTPUT);
//         digitalWrite(motorPins[i].forward, LOW);
//         digitalWrite(motorPins[i].reverse, HIGH);
//         ledcAttach(motorPins[i].pwm, pwmFreq, pwmRes);
//     }

//     // Encoders
//     for (int i = 0; i < 4; i++) {
//         pinMode(encoderPins[i], INPUT_PULLUP);
//     }

//     attachInterrupt(digitalPinToInterrupt(encoderPins[0]), enc0, FALLING);
//     attachInterrupt(digitalPinToInterrupt(encoderPins[1]), enc1, FALLING);
//     attachInterrupt(digitalPinToInterrupt(encoderPins[2]), enc2, FALLING);
//     attachInterrupt(digitalPinToInterrupt(encoderPins[3]), enc3, FALLING);

//     // Web Handlers
//     webServer.attachHandler("/", handleRoot);
//     webServer.attachHandler("/setDirection?value=", handleSetDirection);
//     webServer.attachHandler("/setDuty?value=", handleSetDuty);
//     webServer.attachHandler("/turn?degree=", handleTurn);
//     webServer.attachHandler("/stop", handleStop);
//     webServer.attachHandler("/getCounts", handleGetCounts);
//     webServer.attachHandler("/attack?target=", handleAttack);
//     webServer.attachHandler("/defend?target=", handleDefend);
//     webServer.attachHandler("/gradedTask?type=", handleGradedTask);
//     webServer.attachHandler("/viveCoordinates?x=", handleViveCoordinates);

//     webServer.begin(80);
// }

// // ==========================================================
// // ==================== LOOP ================================
// // ==========================================================

// void loop() {
//     webServer.serve();

//     static unsigned long lastUpdate = 0;
//     if (millis() - lastUpdate >= 20) {
//         lastUpdate = millis();
//         updateWheelRPM();
//         updatePID();
//     }

//     static unsigned long lastVivePrint = 0;
//     if (millis() - lastVivePrint > 100) {
//         lastVivePrint = millis();
//         Serial.printf("Vive X: %d   Vive Y: %d\n", viveX.xCoord(), viveY.yCoord());
//     }

//     if (millis() - lastTopHatSend >= 500) {
//         lastTopHatSend = millis();
//         sendTopHatStatus();
//     }

//     if (millis() - lastUpdate >= 500) {
//         updateAttackServo();
//         updateSafetyServo();
//     }
// }

// // ==========================================================
// // ===================== UTILITIES ==========================
// // ==========================================================

// int leftDist = 0, frontDist = 0, rightDist = 0;

// void updateToF() {
//     leftDist  = tofLeft.readRangeSingleMillimeters();
//     frontDist = tofFront.readRangeSingleMillimeters();
//     rightDist = tofRight.readRangeSingleMillimeters();
// }

// void wallFollowControl() {
//     updateToF();
//     int sideDist = (followSide == 0) ? leftDist : rightDist;
//     int error = targetDist - sideDist;

//     float K = 0.6;
//     float adjust = K * error;

//     if (followSide == 0) {
//         targetRPM[0] = baseRPM - adjust;
//         targetRPM[2] = baseRPM - adjust;
//         targetRPM[1] = baseRPM + adjust;
//         targetRPM[3] = baseRPM + adjust;
//     } else {
//         targetRPM[0] = baseRPM + adjust;
//         targetRPM[2] = baseRPM + adjust;
//         targetRPM[1] = baseRPM - adjust;
//         targetRPM[3] = baseRPM - adjust;
//     }

//     if (frontDist < 120) {
//         fullStop();
//     }
// }

// void sendTopHatStatus() {
//     uint8_t status = wifiCommandReceived ? 1 : 0;

//     Wire.beginTransmission(TOPHAT_I2C_ADDR);
//     Wire.write(status);
//     Wire.endTransmission();

//     wifiCommandReceived = false;
// }

// void updateAttackServo() {
//     if (millis() - lastSwingUpdate < swingInterval) return;
//     lastSwingUpdate = millis();

//     swingPos += swingDirection * swingStep;

//     if (swingPos >= swingMax) { swingPos = swingMax; swingDirection = -1; }
//     if (swingPos <= swingMin) { swingPos = swingMin; swingDirection = +1; }

//     servoSwing.write(swingPos);
// }

// void triggerSafetyServo(int angle) {
//     rotateTargetAngle = constrain(angle, 0, 360);
//     rotateTriggered = true;
// }

// void updateSafetyServo() {
//     if (!rotateTriggered) return;
//     if (millis() - lastRotateUpdate < rotateInterval) return;

//     lastRotateUpdate = millis();

//     if (rotateCurrentAngle < rotateTargetAngle) rotateCurrentAngle += 5;
//     else if (rotateCurrentAngle > rotateTargetAngle) rotateCurrentAngle -= 5;

//     if (abs(rotateCurrentAngle - rotateTargetAngle) <= 3) {
//         rotateCurrentAngle = rotateTargetAngle;
//         rotateTriggered = false;
//     }

//     servoRotate.write(rotateCurrentAngle);
// }
