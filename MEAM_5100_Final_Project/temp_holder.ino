// #include <Wire.h>

// // -------------------------
// // Slave input pins
// // -------------------------
// #define I2C_SLAVE_ADDR 0x28
// #define SDA_PIN 36
// #define SCL_PIN 37

// //--------------------------------------------------------------
// // Send number of WiFi packets (1 byte) to the slave
// //--------------------------------------------------------------
// bool sendWiFiPacketsByte(uint8_t value) {
//   // Send data to slave
//   Wire.beginTransmission(I2C_SLAVE_ADDR);
//   Wire.write(value);
//   uint8_t error = Wire.endTransmission();   // 0 = success

//   if (error == 0) {
//     Serial.printf("[TX] Sent: %u\n", value);
//     return true;
//   } else {
//     Serial.printf("[TX ERROR] Code: %d\n", error);
//     return false;
//   }
// }

// //--------------------------------------------------------------
// // Receive health value (1 byte) from the slave
// //--------------------------------------------------------------
// int receiveHealthValueByte() {
//   // Stored across calls
//   static int lastHealth = 100;

//   // Request up to 1 byte, but allow more if the slave responds late
//   int bytes = Wire.requestFrom(I2C_SLAVE_ADDR, (uint8_t)1);

//   if (bytes > 0) {
//     // Read ALL available bytes (safety for timing drift / slave lag)
//     uint8_t health;
//     while (Wire.available()) {
//       // Last byte becomes the health
//       health = Wire.read();
//     }

//     lastHealth = health;
//     Serial.printf("[RX] Health: %u (bytes=%d)\n", lastHealth, bytes);
//   } else {
//     // No data → keep the last known value
//     Serial.printf("[RX] No data received, keeping last health: %u\n", lastHealth);
//   }

//   return lastHealth;
// }

// void setup() {
//   // Initialize Serial at 115200 BAUD
//   Serial.begin(115200);
//   delay(1000);
//   Serial.println("Starting the setup!");

//   // Initialize I2C master
//   Wire.begin(SDA_PIN, SCL_PIN, 40000);
//   Serial.println("ESP32-S3 I2C Master started");
//   Serial.printf("SDA: %d, SCL: %d\n", SDA_PIN, SCL_PIN);

//   // For simulating WiFi packets
//   randomSeed(esp_random());  // for random 0/1
//   Serial.println("Finished the setup!");
// }

// void loop() {
//   // Create random 0/1 value
//   uint8_t cmd = random(0, 2);

//   // Send 1-byte packet size to TopHat
//   sendWiFiPacketsByte(cmd);

//   // // Retrieve 1-byte health value from TopHat
//   receiveHealthValueByte();

//   // Repeat every 500 ms (2 Hz)
//   delay(500);
// }

// // VIVE
// // Dual Vive + Low-Pass Filtering + Orientation
// #include "vive510.h"

// #define SIGNALPIN1 47
// #define SIGNALPIN2 48

// Vive510 vive1(SIGNALPIN1);
// Vive510 vive2(SIGNALPIN2);

// // -------------------------
// // Low-pass filter constant
// // -------------------------
// const float ALPHA = 0.38;   // smoothing factor (0.2 = smooth, 0.5 = fast)

// // Filter state variables
// float x1_f = 0, y1_f = 0;
// float x2_f = 0, y2_f = 0;

// bool first1 = true, first2 = true;

// // --------------------------
// // Low-pass filter function
// // --------------------------
// bool readLPF(Vive510 &vive, float &xf, float &yf, bool &first)
// {
//   if (vive.status() != VIVE_RECEIVING) {
//     Serial.println("VIVE not working as expected, needs to resync!");
//     return false;
//   }

//   uint16_t xr = vive.xCoord();
//   uint16_t yr = vive.yCoord();

//   // Basic validity gate
//   if (xr < 1000 || xr > 8000 || yr < 1000 || yr > 8000) {
//     Serial.println("Invalid sensor data, skipping...");
//     return false;
//   }

//   // Apply low-pass filter
//   if (first) {
//     xf = xr;
//     yf = yr;
//     first = false;
//   } else {
//     xf = ALPHA * xr + (1.0 - ALPHA) * xf;
//     yf = ALPHA * yr + (1.0 - ALPHA) * yf;
//   }

//   return true;
// }

// void setup() {
//   Serial.begin(115200);
//   pinMode(LED_BUILTIN, OUTPUT);

//   vive1.begin();
//   vive2.begin();

//   Serial.println("Dual Vive with Low-Pass Filtering and Orientation Initialized");
// }

// void loop() {
//   // Read and filter data from both Vive devices
//   if (readLPF(vive1, x1_f, y1_f, first1)) {
//     // Print filtered sensor data for vive1
//     Serial.print("S1 Filtered: x: ");
//     Serial.print(x1_f);
//     Serial.print(" y: ");
//     Serial.println(y1_f);
//   } else {
//     vive1.sync(15); // Attempt resync if needed
//   }

//   if (readLPF(vive2, x2_f, y2_f, first2)) {
//     // Print filtered sensor data for vive2
//     Serial.print("S2 Filtered: x: ");
//     Serial.print(x2_f);
//     Serial.print(" y: ");
//     Serial.println(y2_f);
//   } else {
//     vive2.sync(15); // Attempt resync if needed
//   }

//   // Compute orientation from filtered values
//   float dx = x2_f - x1_f;
//   float dy = y2_f - y1_f;
//   float headingDeg = atan2(dy, dx) * 180.0 / PI;

//   // Print heading
//   Serial.print("Heading: ");
//   Serial.println(headingDeg);

//   delay(10);
// }

// // TOF
// // ESP32-S3 TOF with 3 sensors and wall decision
// #include <Wire.h>
// #include "Adafruit_VL53L0X.h"

// // Define the SDA and SCL pins
// #define SDA_PIN 36
// #define SCL_PIN 37

// // Create sensor objects
// Adafruit_VL53L0X tofLeft  = Adafruit_VL53L0X();
// Adafruit_VL53L0X tofFront = Adafruit_VL53L0X();
// Adafruit_VL53L0X tofRight = Adafruit_VL53L0X();

// #define XSHUT_LEFT  12
// #define XSHUT_FRONT 13
// #define XSHUT_RIGHT 14

// // Distance threshold (5 cm)
// #define WALL_DIST_MM 50

// void setup() {
//   // Start the serial monitor
//   Serial.begin(115200);
//   while (!Serial) { 
//     delay(10); // Wait for the serial connection to establish
//   }

//   Serial.println("Initializing sensors!");

//   // Start I2C on chosen pins
//   Wire.begin(SDA_PIN, SCL_PIN);

//   // Set XSHUT pins
//   pinMode(XSHUT_LEFT, OUTPUT);
//   pinMode(XSHUT_FRONT, OUTPUT);
//   pinMode(XSHUT_RIGHT, OUTPUT);

//   // Shutdown all sensors
//   digitalWrite(XSHUT_LEFT, LOW);
//   digitalWrite(XSHUT_FRONT, LOW);
//   digitalWrite(XSHUT_RIGHT, LOW);

//   // Start LEFT sensor
//   digitalWrite(XSHUT_LEFT, HIGH);
//   delay(100);
//   if (!tofLeft.begin(0x40)) {
//     Serial.println("Failed to init LEFT sensor!");
//     // Attempt to check the sensor with I2C communication
//     byte error = 0;
//     Wire.beginTransmission(0x33);
//     error = Wire.endTransmission();
//     if (error == 0) {
//       Serial.println("LEFT sensor is responding!");
//     } else {
//       Serial.print("I2C communication failed with error code: ");
//       Serial.println(error);
//     }
//     // while (1);  // Stop execution
//   }
//   Serial.println("Left sensor initialized successfully!");

//   // Start FRONT sensor
//   digitalWrite(XSHUT_FRONT, HIGH);
//   delay(100);
//   if (!tofFront.begin(0x34)) {
//     Serial.println("Failed to init FRONT sensor!");
//     // Attempt to check the sensor with I2C communication
//     byte error = 0;
//     Wire.beginTransmission(0x41);
//     error = Wire.endTransmission();
//     if (error == 0) {
//       Serial.println("FRONT sensor is responding!");
//     } else {
//       Serial.print("I2C communication failed with error code: ");
//       Serial.println(error);
//     }
//     // while (1);  // Stop execution
//   }
//   Serial.println("Front sensor initialized successfully!");

//   // Start RIGHT sensor
//   digitalWrite(XSHUT_RIGHT, HIGH);
//   delay(100);
//   if (!tofRight.begin(0x35)) {
//     Serial.println("Failed to init RIGHT sensor!");
//     // Attempt to check the sensor with I2C communication
//     byte error = 0;
//     Wire.beginTransmission(0x42);
//     error = Wire.endTransmission();
//     if (error == 0) {
//       Serial.println("RIGHT sensor is responding!");
//     } else {
//       Serial.print("I2C communication failed with error code: ");
//       Serial.println(error);
//     }
//     // while (1);  // Stop execution
//   }
//   Serial.println("Right sensor initialized successfully!");

//   Serial.println("All sensors initialized successfully!");
// }

// void loop() {
//   // Initialize measurement data for VL53L0X
//   VL53L0X_RangingMeasurementData_t measureLeft;
//   VL53L0X_RangingMeasurementData_t measureFront;
//   VL53L0X_RangingMeasurementData_t measureRight;

//   // Read left (VL53L0X)
//   tofLeft.rangingTest(&measureLeft, false);
//   int distLeft = (measureLeft.RangeStatus != 4) ? measureLeft.RangeMilliMeter : -1;

//   // Read front (VL53L0X)
//   tofFront.rangingTest(&measureFront, false);
//   int distFront = (measureFront.RangeStatus != 4) ? measureFront.RangeMilliMeter : -1;

//   // Read right (VL53L0X)
//   tofRight.rangingTest(&measureRight, false);
//   int distRight = (measureRight.RangeStatus != 4) ? measureRight.RangeMilliMeter : -1;

//   // Check if the measurement is valid
//   if (tofLeft.timeoutOccurred() || tofFront.timeoutOccurred() || tofRight.timeoutOccurred()) {
//     Serial.print("Error: Sensor timeout!");
//     Serial.print(" Left TOF: ");
//     Serial.print(tofLeft.timeoutOccurred());
//     Serial.print(" Front TOF: ");
//     Serial.print(tofFront.timeoutOccurred());
//     Serial.print(" Right TOF: ");
//     Serial.print(tofRight.timeoutOccurred());
//     Serial.println(" (1 is error)");
//   } else {
//     // Print readings
//     Serial.print("Left: ");
//     Serial.print(distLeft);
//     Serial.print(" mm   Front: ");
//     Serial.print(distFront);
//     Serial.print(" mm   Right: ");
//     Serial.print(distRight);
//     Serial.println(" mm");
//   }

//   // Decision Logic
//   bool leftClear  = (distLeft  > WALL_DIST_MM || distLeft  == -1);
//   bool frontClear = (distFront > WALL_DIST_MM || distFront == -1);
//   bool rightClear = (distRight > WALL_DIST_MM || distRight == -1);

//   if (leftClear) {
//     Serial.println("TURN LEFT");
//   } else if (!leftClear && frontClear) {
//     Serial.println("GO FRONT");
//   } else if (!leftClear && !frontClear && rightClear) {
//     Serial.println("TURN RIGHT");
//   } else if (!leftClear && !frontClear && !rightClear) {
//     Serial.println("GO BACK");
//   } else {
//     Serial.println("SHOULD NEVER GET HERE");
//   }

//   delay(100);
// }

// // TOF
// // ESP32-S3 TOF with 3 VL53L1X sensors and wall decision
// #include <Wire.h>
// #include "Adafruit_VL53L1X.h"

// // Define the SDA and SCL pins
// #define SDA_PIN 36
// #define SCL_PIN 37

// // Create sensor objects
// Adafruit_VL53L1X tofLeft  = Adafruit_VL53L1X();
// Adafruit_VL53L1X tofFront = Adafruit_VL53L1X();
// Adafruit_VL53L1X tofRight = Adafruit_VL53L1X();

// #define XSHUT_LEFT  12
// #define XSHUT_FRONT 13
// #define XSHUT_RIGHT 14

// // Distance threshold (5 cm)
// #define WALL_DIST_MM 50

// void setup() {
//   // Start the serial monitor
//   Serial.begin(115200);
//   while (!Serial) { 
//     delay(10); // Wait for the serial connection to establish
//   }

//   Serial.println("Initializing sensors!");

//   // Start I2C on chosen pins
//   Wire.begin(SDA_PIN, SCL_PIN);

//   // Set XSHUT pins
//   pinMode(XSHUT_LEFT, OUTPUT);
//   pinMode(XSHUT_FRONT, OUTPUT);
//   pinMode(XSHUT_RIGHT, OUTPUT);

//   // Shutdown all sensors
//   digitalWrite(XSHUT_LEFT, LOW);
//   digitalWrite(XSHUT_FRONT, LOW);
//   digitalWrite(XSHUT_RIGHT, LOW);

//   // Start LEFT sensor
//   digitalWrite(XSHUT_LEFT, HIGH);
//   delay(100);  // wait for the sensor to power up
//   if (!tofLeft.begin(0x30, &Wire)) {
//     Serial.println("Failed to initialize LEFT sensor!");
//     while (1);  // halt if initialization fails
//   }
//   Serial.println("Left sensor initialized successfully!");

//   // Start FRONT sensor
//   digitalWrite(XSHUT_FRONT, HIGH);
//   delay(100);  // wait for the sensor to power up
//   if (!tofFront.begin(0x31, &Wire)) {
//     Serial.println("Failed to initialize FRONT sensor!");
//     while (1);  // halt if initialization fails
//   }
//   Serial.println("Front sensor initialized successfully!");

//   // Start RIGHT sensor
//   digitalWrite(XSHUT_RIGHT, HIGH);
//   delay(100);  // wait for the sensor to power up
//   if (!tofRight.begin(0x32, &Wire)) {
//     Serial.println("Failed to initialize RIGHT sensor!");
//     while (1);  // halt if initialization fails
//   }
//   Serial.println("Right sensor initialized successfully!");

//   // Initialize the sensors for continuous ranging
//   tofLeft.startRanging();
//   tofFront.startRanging();
//   tofRight.startRanging();
  
//   Serial.println("All sensors initialized and ranging started!");
// }

// void loop() {
//   int16_t distLeft = -1;
//   int16_t distFront = -1;
//   int16_t distRight = -1;

//   // --- Read LEFT Sensor ---
//   if (tofLeft.dataReady()) {
//     distLeft = tofLeft.distance();
//     if (distLeft == -1) {
//       Serial.println(F("Left sensor error."));
//     }
//     // Clear the interrupt status for the next reading
//     tofLeft.clearInterrupt();
//   }

//   // --- Read FRONT Sensor ---
//   if (tofFront.dataReady()) {
//     distFront = tofFront.distance();
//     if (distFront == -1) {
//       Serial.println(F("Front sensor error."));
//     }
//     tofFront.clearInterrupt();
//   }

//   // --- Read RIGHT Sensor ---
//   if (tofRight.dataReady()) {
//     distRight = tofRight.distance();
//     if (distRight == -1) {
//       Serial.println(F("Right sensor error."));
//     }
//     tofRight.clearInterrupt();
//   }

//   // Print values or timeout/errors
//   if (distLeft == -1 || distFront == -1 || distRight == -1) {
//     Serial.print("Error/Invalid Measurement -> ");
//     Serial.print("Left: ");  Serial.print(distLeft);
//     Serial.print(" Front: "); Serial.print(distFront);
//     Serial.print(" Right: "); Serial.println(distRight);
//   } else {
//     Serial.print("Left: ");
//     Serial.print(distLeft);
//     Serial.print(" mm   Front: ");
//     Serial.print(distFront);
//     Serial.print(" mm   Right: ");
//     Serial.print(distRight);
//     Serial.println(" mm");
//   }

//   // Decision Logic
//   bool leftClear  = (distLeft  > WALL_DIST_MM || distLeft  == -1);
//   bool frontClear = (distFront > WALL_DIST_MM || distFront == -1);
//   bool rightClear = (distRight > WALL_DIST_MM || distRight == -1);

//   if (leftClear) {
//     Serial.println("TURN LEFT");
//   } else if (!leftClear && frontClear) {
//     Serial.println("GO FRONT");
//   } else if (!leftClear && !frontClear && rightClear) {
//     Serial.println("TURN RIGHT");
//   } else if (!leftClear && !frontClear && !rightClear) {
//     Serial.println("GO BACK");
//   } else {
//     Serial.println("SHOULD NEVER GET HERE");
//   }

//   delay(100);
// }

// // SERVO
// // Servo that keeps continuously rotating 180 degrees
// #include <ESP32Servo.h>

// Servo myServo;

// // --- USER SETTINGS ---
// const int servoPin = 18;        // Choose any PWM-capable pin
// const int minUs    = 500;       // Servo pulse minimum
// const int maxUs    = 2400;      // Servo pulse maximum

// // Motion parameters
// const int angleA = 0;           // Start angle
// const int angleB = 180;         // End angle
// const float frequency = 0.5;    // Hz
// // For 2Hz: period = 1/f = 0.5 sec, half-period = 250ms
// unsigned long halfPeriodMs = (1.0 / frequency) * 500.0; 

// void setup() {
//   myServo.attach(servoPin, minUs, maxUs);
//   myServo.writeMicroseconds(minUs);  // Set the servo to its minimum position initially
// }

// void loop() {
//   unsigned long startTime = millis();  // Get the current time

//   // Move to angleB (180 degrees) quickly in the given time
//   while (millis() - startTime < halfPeriodMs) {
//     int elapsedTime = millis() - startTime;
//     int position = map(elapsedTime, 0, halfPeriodMs, minUs, maxUs);
//     myServo.writeMicroseconds(position);  // Gradually move the servo
//   }

//   // Reverse motion to angleA (0 degrees) in the next half-period
//   startTime = millis();  // Reset start time for reverse motion

//   while (millis() - startTime < halfPeriodMs) {
//     int elapsedTime = millis() - startTime;
//     int position = map(elapsedTime, 0, halfPeriodMs, maxUs, minUs);
//     myServo.writeMicroseconds(position);  // Gradually move back to position A
//   }
// }



























// // ========================================================
// //  ADVANCED WALL FOLLOWING + 2-WHEEL PID DRIVE + TOF
// //  ESP32-S3  (VL53L1X + Encoders + PID + Ramp Acceleration)
// // ========================================================

// #include <Wire.h>
// #include "Adafruit_VL53L1X.h"

// /* ========================================================
//    TOF SENSOR DEFINITIONS
//    ======================================================== */
// #define SDA_PIN 36
// #define SCL_PIN 37

// Adafruit_VL53L1X tofLeft  = Adafruit_VL53L1X();
// Adafruit_VL53L1X tofFront = Adafruit_VL53L1X();
// Adafruit_VL53L1X tofRight = Adafruit_VL53L1X();

// #define XSHUT_LEFT  12
// #define XSHUT_FRONT 13
// #define XSHUT_RIGHT 14

// #define WALL_DIST_MM 100     // Desired wall following distance (10 cm)
// #define FRONT_STOP_MM 150    // Stop/turn if front < 15 cm
// #define CLEAR_DIST_MM 500    // “Open space” threshold

// /* ========================================================
//    MOTOR + ENCODER DEFINITIONS
//    ======================================================== */
// const int BASE_SPEED = 80;
// const int TURN_SPEED = 60;
// const int pwmFreq = 2000;
// const int pwmRes  = 8;
// const int ledcRes = ((1 << pwmRes) - 1);

// // LEFT MOTOR
// const int dirL1 = 6;
// const int dirL2 = 7;
// const int pwmL  = 15;

// // RIGHT MOTOR
// const int dirR1 = 42;
// const int dirR2 = 41;
// const int pwmR  = 40;

// // ENCODERS
// const int encLA = 16;    // LEFT encoder channel A
// const int encLB = 17;    // LEFT encoder channel B

// const int encRA = 39;    // RIGHT encoder channel A
// const int encRB = 38;    // RIGHT encoder channel B

// // Encoder counters
// volatile long posL = 0;
// volatile long posR = 0;

// volatile uint8_t lastL = 0;
// volatile uint8_t lastR = 0;

// /* ========================================================
//    PID VARIABLES
//    ======================================================== */
// double Kp = 1.2, Ki = 0.4, Kd = 0.05;

// double inputL, inputR;
// double outputL, outputR;

// double goalSpeedL = 0, goalSpeedR = 0;
// double prevErrorL = 0, prevErrorR = 0;
// double integralL  = 0, integralR  = 0;

// /* ========================================================
//    RAMPING SYSTEM (SMOOTH ACCEL/DECEL)
//    ======================================================== */
// double rampRate = 5;   // Units per loop (50ms), tune for smooth accel

// double rampTo(double currentSpeed, double target) {
//   if (currentSpeed - target < rampRate) currentSpeed = target;
//   if (currentSpeed < target) currentSpeed += rampRate;
//   if (currentSpeed > target) currentSpeed -= rampRate;
//   if (currentSpeed < 0) currentSpeed = 0;
//   return currentSpeed;
// }

// /* ========================================================
//    QUADRATURE DECODER
//    ======================================================== */
// void IRAM_ATTR updateEncoder(volatile long &pos, volatile uint8_t &last, int pinA, int pinB) {
//   uint8_t state = (digitalRead(pinA) << 1) | digitalRead(pinB);
//   uint8_t combined = (last << 2) | state;

//   static const int8_t table[16] = {
//     0, -1, +1, 0,
//     +1, 0, 0, -1,
//     -1, 0, 0, +1,
//     0, +1, -1, 0
//   };

//   pos += table[combined];
//   last = state;
// }

// void IRAM_ATTR isrL() { updateEncoder(posL, lastL, encLA, encLB); }
// void IRAM_ATTR isrR() { updateEncoder(posR, lastR, encRA, encRB); }

// /* ========================================================
//    PID
//    ======================================================== */
// void computePID(double &input, double &output, double setpoint,
//                 double &prevError, double &integral)
// {
//   double error = setpoint - input;
//   integral += error;
//   double derivative = error - prevError;

//   output = Kp * error + Ki * integral + Kd * derivative;

//   prevError = error;

//   if (output > 100) output = 100;
//   if (output < 0)       output = 0;
// }

// /* ========================================================
//    MOTOR CONTROL
//    ======================================================== */
// void setMotor(int pwmPin, double pwmValue) {
//   pwmValue  = constrain(pwmValue,  0, 100);
//   uint16_t duty  = (ledcRes * pwmValue) / 100;
//   ledcWrite(pwmPin, (int) duty);

//   Serial.print("PWM Motor: ");
//   Serial.print(pwmPin == 15? "Left" : "Right");
//   Serial.print(" PWM Duty Dycle: ");
//   Serial.print(pwmValue);
// }

// /* ========================================================
//    MOVEMENT COMMANDS
//    ======================================================== */
// void resetPID() {
//   integralL = integralR = 0;
//   prevErrorL = prevErrorR = 0;
//   posL = posR = 0;
// }

// void goForward(int speedL, int speedR) {
//   digitalWrite(dirL1, HIGH);
//   digitalWrite(dirL2, LOW);

//   digitalWrite(dirR1, HIGH);
//   digitalWrite(dirR2, LOW);

//   goalSpeedL = speedL;
//   goalSpeedR = speedR;
//   resetPID();
// }

// void goBack(int speedL, int speedR) {
//   digitalWrite(dirL1, LOW);
//   digitalWrite(dirL2, HIGH);

//   digitalWrite(dirR1, LOW);
//   digitalWrite(dirR2, HIGH);

//   goalSpeedL = speedL;
//   goalSpeedR = speedR;
//   resetPID();
// }

// void turnLeft(int speedL, int speedR) {
//   digitalWrite(dirL1, LOW);
//   digitalWrite(dirL2, HIGH);

//   digitalWrite(dirR1, HIGH);
//   digitalWrite(dirR2, LOW);

//   goalSpeedL = speedL;
//   goalSpeedR = speedR;
//   resetPID();
// }

// void turnRight(int speedL, int speedR) {
//   digitalWrite(dirL1, HIGH);
//   digitalWrite(dirL2, LOW);

//   digitalWrite(dirR1, LOW);
//   digitalWrite(dirR2, HIGH);

//   goalSpeedL = speedL;
//   goalSpeedR = speedR;
//   resetPID();
// }

// void stop() {
//   digitalWrite(dirL1, HIGH);
//   digitalWrite(dirL2, HIGH);

//   digitalWrite(dirR1, HIGH);
//   digitalWrite(dirR2, HIGH);

//   goalSpeedL = 0;
//   goalSpeedR = 0;
//   resetPID();
// }

// /* ========================================================
//    SETUP
//    ======================================================== */
// void setup() {
//   Serial.begin(115200);
//   while(!Serial) delay(10);

//   Wire.begin(SDA_PIN, SCL_PIN);

//   pinMode(XSHUT_LEFT, OUTPUT);
//   pinMode(XSHUT_FRONT, OUTPUT);
//   pinMode(XSHUT_RIGHT, OUTPUT);

//   digitalWrite(XSHUT_LEFT, LOW);
//   digitalWrite(XSHUT_FRONT, LOW);
//   digitalWrite(XSHUT_RIGHT, LOW);
//   delay(50);

//   digitalWrite(XSHUT_LEFT, HIGH); delay(100);
//   if (!tofLeft.begin(0x30, &Wire)) { Serial.println("Left TOF Fail"); while(1); }

//   digitalWrite(XSHUT_FRONT, HIGH); delay(100);
//   if (!tofFront.begin(0x31, &Wire)) { Serial.println("Front TOF Fail"); while(1); }

//   digitalWrite(XSHUT_RIGHT, HIGH); delay(100);
//   if (!tofRight.begin(0x32, &Wire)) { Serial.println("Right TOF Fail"); while(1); }

//   pinMode(dirL1, OUTPUT);
//   pinMode(dirL2, OUTPUT);
//   pinMode(dirR1, OUTPUT);
//   pinMode(dirR2, OUTPUT);

//   if (!ledcAttach(pwmL, pwmFreq, pwmRes))
//       Serial.println("Error: LEDC attach failed (LEFT)");

//   if (!ledcAttach(pwmR, pwmFreq, pwmRes))
//       Serial.println("Error: LEDC attach failed (RIGHT)");

//   pinMode(encLA, INPUT_PULLUP);
//   pinMode(encLB, INPUT_PULLUP);
//   pinMode(encRA, INPUT_PULLUP);
//   pinMode(encRB, INPUT_PULLUP);

//   attachInterrupt(encLA, isrL, CHANGE);
//   attachInterrupt(encLB, isrL, CHANGE);
//   attachInterrupt(encRA, isrR, CHANGE);
//   attachInterrupt(encRB, isrR, CHANGE);

//   Serial.println("Setup complete.");
// }

// /* ========================================================
//    MAIN LOOP – ADVANCED WALL FOLLOWING
//    ======================================================== */
// void loop() {

//   // ======================================================
//   // READ TOF SENSORS
//   // ======================================================
//   int16_t distLeft = -1;
//   int16_t distFront = -1;
//   int16_t distRight = -1;

//   // --- Read LEFT Sensor ---
//   if (tofLeft.dataReady()) {
//     distLeft = tofLeft.distance();
//     if (distLeft == -1) {
//       Serial.println(F("Left sensor error."));
//     }
//     // Clear the interrupt status for the next reading
//     tofLeft.clearInterrupt();
//   }

//   // --- Read FRONT Sensor ---
//   if (tofFront.dataReady()) {
//     distFront = tofFront.distance();
//     if (distFront == -1) {
//       Serial.println(F("Front sensor error."));
//     }
//     tofFront.clearInterrupt();
//   }

//   // --- Read RIGHT Sensor ---
//   if (tofRight.dataReady()) {
//     distRight = tofRight.distance();
//     if (distRight == -1) {
//       Serial.println(F("Right sensor error."));
//     }
//     tofRight.clearInterrupt();
//   }

//   // Decide sensor availability
//   bool leftOpen  = (distLeft == -1 || distLeft > CLEAR_DIST_MM);
//   bool frontOpen = (distFront == -1 || distFront > FRONT_STOP_MM);
//   bool rightOpen = (distRight == -1 || distRight > CLEAR_DIST_MM);

//   // ======================================
//   // RULE 1: Left wall gone → turn left
//   // ======================================
//   if (leftOpen) {
//     // Always choose left if available
//     Serial.println("TURNING LEFT");
//     turnLeft(TURN_SPEED, TURN_SPEED);
//   }

//   // ======================================
//   // RULE 2: Front open → forward with correction
//   // ======================================
//   else if (frontOpen) {
//     // Go straight if you can
//     Serial.println("GOING FORWARD");

//     // ==================================================
//     // LOW-LEVEL SMOOTH WALL FOLLOWING (LEFT WALL)
//     // ==================================================

//     // Positive: far, Negative: close
//     int error = distLeft - WALL_DIST_MM;

//     // Calculate the speed correction term
//     int correction = error * 0.6;

//     // Update the motor speeds based on correction term
//     int leftSpeed  = BASE_SPEED - correction;
//     int rightSpeed = BASE_SPEED + correction;
//     Serial.printf("Forward L=%d R=%d\n", leftSpeed, rightSpeed);

//     // Set new forward speed for both motors
//     goForward(leftSpeed, rightSpeed);
//   }

//   // ======================================
//   // RULE 3: Front blocked & left blocked → turn right
//   // ======================================
//   else if (rightOpen) {
//     // Last resort option before backing up
//     Serial.println("TURNING RIGHT");
//     turnRight(TURN_SPEED, TURN_SPEED);
//   }

//   // ======================================
//   // RULE 4: Everything blocked → reverse
//   // ======================================
//   else {
//     // Dead end
//     Serial.println("Maze logic: DEAD END → reverse");
//     goBack(TURN_SPEED, TURN_SPEED);
//   }

//   // ======================================================
//   // PID + SMOOTH RAMP
//   // ======================================================

//   // Calculate the target speed for each motor
//   double targetSpeedL = rampTo(outputL, goalSpeedL);
//   double targetSpeedR = rampTo(outputR, goalSpeedR);

//   // Get encoder position reading
//   inputL = posL;
//   inputR = posR;

//   // Compute PID for both motors
//   computePID(inputL, outputL, targetSpeedL, prevErrorL, integralL);
//   computePID(inputR, outputR, targetSpeedR, prevErrorR, integralR);

//   // Set required PWM for both motors
//   setMotor(pwmL, outputL);
//   setMotor(pwmR, outputR);

//   // Wait 50ms before running again
//   delay(50);
// }
