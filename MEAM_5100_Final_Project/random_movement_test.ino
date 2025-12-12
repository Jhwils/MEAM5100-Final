// #include <WiFi.h>
// #include "html510.h"

// /* ========================================================
//    WIFI AND WEBSITE
//    ======================================================== */

// const char *ssid = "Ani_WiFi";

// IPAddress myIP(192, 168, 1, 147);  // change to your IP
// HTML510Server webServer(80);

// double pwmDisplayL = 0;
// double pwmDisplayR = 0;
// String currentDirection = "STOP";

// String controlPage() {
//   return R"rawliteral(
//   <html>
//     <head>
//       <meta charset="utf-8"/>
//       <title>Robot PID + PWM Dashboard</title>
//       <style>
//         body { font-family: Arial; background:#f2f2f2; text-align:center; }
//         .box { background:white; padding:20px; margin:20px auto; width:90%; max-width:600px; 
//                border-radius:10px; box-shadow:0 0 10px rgba(0,0,0,0.15); }
//         .slider { width:80%; }
//         h2 { margin-bottom:5px; }
//       </style>
//     </head>

//     <body>
//       <div class="box">
//         <h2>Robot Telemetry</h2>
//         <p><b>Left PWM:</b> <span id="pwmL">0</span>%</p>
//         <p><b>Right PWM:</b> <span id="pwmR">0</span>%</p>
//         <p><b>Direction:</b> <span id="dir">STOP</span></p>
//       </div>

//       <div class="box">
//         <h2>PID Tuning</h2>
//         <p>Kp: <span id="kpTxt">0</span></p>
//         <input type="range" min="0" max="5" step="0.01" id="kp" class="slider"
//                oninput="updatePID()">
//         <p>Ki: <span id="kiTxt">0</span></p>
//         <input type="range" min="0" max="5" step="0.01" id="ki" class="slider"
//                oninput="updatePID()">
//         <p>Kd: <span id="kdTxt">0</span></p>
//         <input type="range" min="0" max="5" step="0.01" id="kd" class="slider"
//                oninput="updatePID()">
//       </div>

//       <script>
//         // Update PID on the server
//         function updatePID() {
//           let kp = document.getElementById("kp").value;
//           let ki = document.getElementById("ki").value;
//           let kd = document.getElementById("kd").value;

//           document.getElementById("kpTxt").innerText = kp;
//           document.getElementById("kiTxt").innerText = ki;
//           document.getElementById("kdTxt").innerText = kd;

//           fetch(`/setPID?kp=${kp}&ki=${ki}&kd=${kd}`)
//             .catch(err => console.log(err));
//         }

//         // Telemetry updater
//         setInterval(() => {
//           fetch('/telemetry')
//             .then(res => res.json())
//             .then(data => {
//               document.getElementById("pwmL").innerText = data.pwmL.toFixed(1);
//               document.getElementById("pwmR").innerText = data.pwmR.toFixed(1);
//               document.getElementById("dir").innerText = data.direction;

//               document.getElementById("kp").value = data.kp;
//               document.getElementById("ki").value = data.ki;
//               document.getElementById("kd").value = data.kd;

//               document.getElementById("kpTxt").innerText = data.kp;
//               document.getElementById("kiTxt").innerText = data.ki;
//               document.getElementById("kdTxt").innerText = data.kd;
//             });
//         }, 300);
//       </script>
//     </body>

//   </html>
//   )rawliteral";
// }

// /* ==========================================
//    MOTOR PIN DEFINITIONS (2-Wheel Diff Drive)
//    ========================================== */

// // PWM settings
// const int pwmFreq = 2000;                 // PWM frequency in Hz
// const int pwmRes = 8;                     // 8-bit resolution (0–255)
// const int ledcRes = ((1 << pwmRes) - 1);  // Maximum PWM value (255)

// // LEFT MOTOR (H-Bridge controlled)
// const int dirL1 = 6;  // Direction pin 1 for LEFT motor
// const int dirL2 = 7;  // Direction pin 2 for LEFT motor
// const int pwmL = 15;  // PWM pin for LEFT motor

// // RIGHT MOTOR (H-Bridge controlled)
// const int dirR1 = 42;  // Direction pin 1 for RIGHT motor
// const int dirR2 = 41;  // Direction pin 2 for RIGHT motor
// const int pwmR = 40;   // PWM pin for RIGHT motor

// /* =================================
//    ENCODER PIN DEFINITIONS
//    =================================
//    Each wheel uses one quadrature
//    encoder with A and B signals.
//    ================================= */

// const int encLA = 16;  // LEFT encoder channel A
// const int encLB = 17;  // LEFT encoder channel B

// const int encRA = 39;  // RIGHT encoder channel A
// const int encRB = 38;  // RIGHT encoder channel B

// /* =================================
//    ENCODER COUNTERS (volatile)
//    =================================
//    These variables store the
//    accumulated tick count.
//    'volatile' means ISR can modify them.
//    ================================= */

// volatile long posL = 0;  // Left wheel encoder count
// volatile long posR = 0;  // Right wheel encoder count

// /* =================================
//    LAST ENCODER STATES
//    =================================
//    Required for full quadrature
//    decoding — stores previous AB state.
//    ================================= */

// volatile uint8_t lastL = 0;  // Last left wheel encoder count
// volatile uint8_t lastR = 0;  // Last right wheel encoder count

// /* =================================
//    PID VARIABLES
//    ================================= */

// double Kp = 1.2, Ki = 0.4, Kd = 0.05;  // PID tuning parameters

// double inputL, inputR;    // Current measured wheel speeds
// double outputL, outputR;  // PID outputs (PWM afterwards)

// double setpointL = 0;  // Target speed left wheel
// double setpointR = 0;  // Target speed right wheel

// double prevErrorL = 0, prevErrorR = 0;  // Previous cycle error for derivative
// double integralL = 0, integralR = 0;    // Accumulated error for integral term

// /* ====================================
//    QUADRATURE DECODER (INTERRUPTS)
//    ====================================
//    Reads A/B channels and determines
//    rotation direction using a 16-entry
//    full quadrature decode table.
//    ==================================== */

// void IRAM_ATTR updateEncoder(volatile long &pos, volatile uint8_t &last, int pinA, int pinB) {
//   uint8_t state = (digitalRead(pinA) << 1) | digitalRead(pinB);
//   uint8_t combined = (last << 2) | state;

//   // Quadrature decode table:
//   // Determines movement direction & magnitude
//   static const int8_t table[16] = {
//     0, -1, +1, 0,
//     +1, 0, 0, -1,
//     -1, 0, 0, +1,
//     0, +1, -1, 0
//   };

//   pos += table[combined];
//   last = state;
// }

// /* =================================
//    INTERRUPT SERVICE ROUTINES
//    =================================
//    Called automatically when A or B
//    channels change state.
//    ================================= */

// void IRAM_ATTR isrL() {
//   updateEncoder(posL, lastL, encLA, encLB);
// }
// void IRAM_ATTR isrR() {
//   updateEncoder(posR, lastR, encRA, encRB);
// }

// /* ===============================
//    PID CONTROLLER FUNCTION
//    ===============================
//    input: measured value (ticks)
//    setpoint: desired ticks
//    output: PWM signal (0–255)
//    =============================== */

// void computePID(double &input, double &output, double setpoint,
//                 double &prevError, double &integral) {

//   double error = setpoint - input;
//   integral += error;                      // Accumulate integral
//   double derivative = error - prevError;  // Estimate derivative

//   // PID equation
//   output = Kp * error + Ki * integral + Kd * derivative;

//   prevError = error;

//   // Limit output (only positive for your PWM logic)
//   if (output > ledcRes) output = ledcRes;
//   if (output < 0) output = 0;
// }

// /* ===============================
//    MOTOR CONTROL (PWM ONLY)
//    =============================== */

// void setMotor(int pwmPin, double pwmValue) {
//   pwmValue = constrain(pwmValue, 0, 100);
//   uint16_t duty = (ledcRes * pwmValue) / 100;
//   ledcWrite(pwmPin, (int)duty);

//   Serial.print("PWM Motor: ");
//   Serial.print(pwmPin == 15 ? "Left" : "Right");
//   Serial.print(" PWM Duty Dycle: ");
//   Serial.println(pwmValue);
// }

// /* ===============================
//    DRIVE FUNCTIONS
//    =============================== */

// void goForward(int speed) {
//   currentDirection = "FORWARD";
//   Serial.println("FORWARD");

//   // LEFT side forward
//   digitalWrite(dirL1, HIGH);
//   digitalWrite(dirL2, LOW);

//   // RIGHT side forward
//   digitalWrite(dirR1, HIGH);
//   digitalWrite(dirR2, LOW);

//   // Set PID speed setpoints
//   setpointL = speed;
//   setpointR = speed;
//   setMotor(pwmL, setpointL);
//   setMotor(pwmR, setpointL);

//   // Reset PID values
//   integralL = integralR = 0;
//   prevErrorL = prevErrorR = 0;
// }

// void goReverse(int speed) {
//   currentDirection = "REVERSE";
//   Serial.println("REVERSE");

//   // LEFT side reverse
//   digitalWrite(dirL1, LOW);
//   digitalWrite(dirL2, HIGH);

//   // RIGHT side reverse
//   digitalWrite(dirR1, LOW);
//   digitalWrite(dirR2, HIGH);

//   // Set PID speed setpoints
//   setpointL = speed;
//   setpointR = speed;
//   setMotor(pwmL, setpointL);
//   setMotor(pwmR, setpointL);

//   // Reset PID values
//   integralL = integralR = 0;
//   prevErrorL = prevErrorR = 0;
// }

// void turnLeft(int speed) {
//   currentDirection = "LEFT";
//   Serial.println("LEFT");

//   // LEFT wheel backward
//   digitalWrite(dirL1, LOW);
//   digitalWrite(dirL2, HIGH);

//   // RIGHT wheel forward
//   digitalWrite(dirR1, HIGH);
//   digitalWrite(dirR2, LOW);

//   // Set PID speed setpoints
//   setpointL = speed;
//   setpointR = speed;
//   setMotor(pwmL, setpointL);
//   setMotor(pwmR, setpointL);

//   // Reset PID values
//   integralL = integralR = 0;
//   prevErrorL = prevErrorR = 0;
// }

// void turnRight(int speed) {
//   currentDirection = "RIGHT";
//   Serial.println("RIGHT");

//   // LEFT wheel forward
//   digitalWrite(dirL1, HIGH);
//   digitalWrite(dirL2, LOW);

//   // RIGHT wheel backward
//   digitalWrite(dirR1, LOW);
//   digitalWrite(dirR2, HIGH);

//   // Set PID speed setpoints
//   setpointL = speed;
//   setpointR = speed;
//   setMotor(pwmL, setpointL);
//   setMotor(pwmR, setpointL);

//   // Reset PID values
//   integralL = integralR = 0;
//   prevErrorL = prevErrorR = 0;
// }

// void stop() {
//   currentDirection = "STOP";
//   Serial.println("STOP");

//   // Apply BRAKE mode on both motors
//   digitalWrite(dirL1, HIGH);
//   digitalWrite(dirL2, HIGH);

//   digitalWrite(dirR1, HIGH);
//   digitalWrite(dirR2, HIGH);

//   // Set PID speed setpoints
//   setpointL = 0;
//   setpointR = 0;
//   setMotor(pwmL, setpointL);
//   setMotor(pwmR, setpointL);

//   // Reset PID values
//   outputL = outputR = 0;
//   integralL = integralR = 0;
//   prevErrorL = prevErrorR = 0;
// }

// /* ========================================================
//    WEBSITE COMMANDS
//    ======================================================== */
// // Handle root
// void handleRoot() {
//   webServer.sendhtml(controlPage());
// }

// void handleTelemetry() {
//   String json = "{";
//   json += "\"pwmL\":" + String(outputL, 1) + ",";
//   json += "\"pwmR\":" + String(outputR, 1) + ",";
//   json += "\"direction\":\"" + currentDirection + "\",";
//   json += "\"kp\":" + String(Kp, 3) + ",";
//   json += "\"ki\":" + String(Ki, 3) + ",";
//   json += "\"kd\":" + String(Kd, 3);
//   json += "}";

//   webServer.sendplain(json);
// }

// void handleSetPID() {
//   String txt = webServer.getText();  // example: "1.2&ki=0.4&kd=0.05"

//   int a = txt.indexOf('&');
//   int b = txt.indexOf('&', a + 1);

//   Kp = txt.substring(0, a).toFloat();

//   int eq1 = txt.indexOf('=', a);
//   Ki = txt.substring(eq1 + 1, b).toFloat();

//   int eq2 = txt.indexOf('=', b);
//   Kd = txt.substring(eq2 + 1).toFloat();

//   webServer.sendplain("OK PID updated");
// }

// /* ===============================
//    SETUP
//    =============================== */

// void setup() {
//   // Start the serial monitor
//   Serial.begin(115200);
//   while (!Serial) {
//     delay(10);  // Wait for the serial connection to establish
//   }
//   Serial.println("Initializing pins!");

//   // Set direction pins to output
//   pinMode(dirL1, OUTPUT);
//   pinMode(dirL2, OUTPUT);
//   pinMode(dirR1, OUTPUT);
//   pinMode(dirR2, OUTPUT);
//   Serial.println("Initialized direction pins!");

//   // Set PWM pins with LEDC
//   if (!ledcAttach(pwmL, pwmFreq, pwmRes)) {
//     Serial.println("Error: LEDC attach for pwmL failed!");
//   }
//   if (!ledcAttach(pwmR, pwmFreq, pwmRes)) {
//     Serial.println("Error: LEDC attach for pwmR failed!");
//   }
//   ledcWrite(pwmL, 50);
//   ledcWrite(pwmR, 50);
//   Serial.println("Initialized PWM pins!");

//   // Set encoder pins to input with pullup
//   pinMode(encLA, INPUT_PULLUP);
//   pinMode(encLB, INPUT_PULLUP);
//   pinMode(encRA, INPUT_PULLUP);
//   pinMode(encRB, INPUT_PULLUP);
//   Serial.println("Initialized encoder pins!");

//   // Attach interrupts to encoder pins
//   attachInterrupt(encLA, isrL, CHANGE);
//   attachInterrupt(encLB, isrL, CHANGE);
//   attachInterrupt(encRA, isrR, CHANGE);
//   attachInterrupt(encRB, isrR, CHANGE);
//   Serial.println("Attached encoder pins to interrupt!");

//   // Start WiFi in AP mode
//   WiFi.mode(WIFI_AP);
//   WiFi.softAP(ssid, "");
//   WiFi.softAPConfig(myIP,                          // Device IP address
//                     IPAddress(192, 168, 1, 1),     // gateway (not important for 5100)
//                     IPAddress(255, 255, 255, 0));  // net mask
//   Serial.printf("Access Point started. Connect to %s\n", ssid);

//   // Attach web handlers using your helper
//   webServer.attachHandler("/", handleRoot);
//   webServer.attachHandler("/telemetry", handleTelemetry);
//   webServer.attachHandler("/setPID", handleSetPID);
//   webServer.begin(80);
//   Serial.println("Web server started. Open browser to http://" + myIP.toString());

//   Serial.println("All pins initialized successfully!");
// }

// /* ===============================
//    MAIN LOOP
//    =============================== */

// void loop() {
//   // Serve the web server
//   webServer.serve();

//   static unsigned long lastActionTime = 0;
//   const unsigned long interval = 5000;  // 5 seconds
//   int speed = 80;                       // Hard-coded test speed 1
//   int turnSpeed = 60;                   // Hard-coded test speed 2

//   // Every second, choose a random movement
//   if (millis() - lastActionTime >= interval) {
//     lastActionTime = millis();

//     int choice = random(0, 3);

//     switch (choice) {
//       Serial.println("Decision time!");
//       Serial.print("Case: ");
//       Serial.println(choice);

//       case 0: goForward(speed); break;
//       case 1: goReverse(speed); break;
//       case 2: turnLeft(turnSpeed); break;
//       case 3: turnRight(turnSpeed); break;
//     }
//   }

//   // Read encoder counts
//   inputL = posL;
//   inputR = posR;

//   // Compute PID outputs
//   computePID(inputL, outputL, setpointL, prevErrorL, integralL);
//   computePID(inputR, outputR, setpointR, prevErrorR, integralR);

//   // Apply PWM to motors
//   setMotor(pwmL, outputL);
//   setMotor(pwmR, outputR);
// }
