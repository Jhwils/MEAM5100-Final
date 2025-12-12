// // =================================================================
// //  ADVANCED WALL FOLLOWING + 2-WHEEL PID DRIVE + TOF + 9-DOF IMU
// //  ESP32-S3  (VL53L1X + BNO055 + Encoders + PID + Ramp Acceleration)
// // =================================================================

// #include <Wire.h>
// #include "Adafruit_VL53L1X.h"
// #include <Adafruit_BNO055.h>
// #include <Adafruit_Sensor.h>
// #include <utility/imumaths.h>

// /* ========================================================
//    TOF AND IMU SENSOR DEFINITIONS
//    ======================================================== */
// #define SDA_PIN 36
// #define SCL_PIN 37

// Adafruit_VL53L1X tofLeft  = Adafruit_VL53L1X();
// Adafruit_VL53L1X tofFront = Adafruit_VL53L1X();
// Adafruit_VL53L1X tofRight = Adafruit_VL53L1X();

// Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

// #define XSHUT_LEFT  12
// #define XSHUT_FRONT 11
// #define XSHUT_RIGHT 10

// #define WALL_DIST_MM 100     // Desired wall following distance (10 cm)
// #define FRONT_STOP_MM 150    // Stop/turn if front < 15 cm
// #define CLEAR_DIST_MM 500    // “Open space” threshold

// /* ========================================================
//    MOTOR + ENCODER DEFINITIONS
//    ======================================================== */
// const int BASE_DUTY_CYCLE = 80;
// const int REVERSE_DUTY_CYCLE = 60;
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
// const int encLA = 16;
// const int encLB = 17;

// const int encRA = 39;
// const int encRB = 38;

// // Encoder counters
// volatile long posL = 0;
// volatile long posR = 0;

// volatile uint8_t lastL = 0;
// volatile uint8_t lastR = 0;

// /* ========================================================
//    SPEED PID VARIABLES
//    ======================================================== */
// double Kp = 1.2, Ki = 0.4, Kd = 0.05;

// double inputL, inputR;
// double outputL, outputR;

// double goalDutyCycleL = 0, goalDutyCycleR = 0;
// double prevErrorL = 0, prevErrorR = 0;
// double integralL  = 0, integralR  = 0;

// /* ========================================================
//    RAMPING SYSTEM (SMOOTH ACCEL/DECEL)
//    ======================================================== */
// double rampRate = 5;   // Units per loop (50ms), tune for smooth accel

// double rampTo(double currentDutyCycle, double targetDutyCycle) {
//   if (currentDutyCycle - targetDutyCycle < rampRate) currentDutyCycle = targetDutyCycle;
//   if (currentDutyCycle < targetDutyCycle) currentDutyCycle += rampRate;
//   if (currentDutyCycle > targetDutyCycle) currentDutyCycle -= rampRate;
//   if (currentDutyCycle < 0) currentDutyCycle = 0;
//   return currentDutyCycle;
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
// void setMotor(int pwmPin, double pwmDutyCycleValue) {
//   pwmDutyCycleValue  = constrain(pwmDutyCycleValue,  0, 100);
//   uint16_t duty  = (ledcRes * pwmDutyCycleValue) / 100;
//   ledcWrite(pwmPin, (int) duty);

//   Serial.print("PWM Motor: ");
//   Serial.print(pwmPin == 15? "Left" : "Right");
//   Serial.print(" PWM Duty Dycle: ");
//   Serial.print(pwmDutyCycleValue);
// }

// /* ========================================================
//    MOVEMENT COMMANDS
//    ======================================================== */
// void resetPID() {
//   integralL = integralR = 0;
//   prevErrorL = prevErrorR = 0;
//   posL = posR = 0;
// }

// void goForward(int dutyCycleL, int dutyCycleR) {
//   digitalWrite(dirL1, HIGH);
//   digitalWrite(dirL2, LOW);

//   digitalWrite(dirR1, HIGH);
//   digitalWrite(dirR2, LOW);

//   goalDutyCycleL = dutyCycleL;
//   goalDutyCycleR = dutyCycleR;
//   resetPID();
// }

// void goBack(int dutyCycleL, int dutyCycleR) {
//   digitalWrite(dirL1, LOW);
//   digitalWrite(dirL2, HIGH);

//   digitalWrite(dirR1, LOW);
//   digitalWrite(dirR2, HIGH);

//   goalDutyCycleL = dutyCycleL;
//   goalDutyCycleR = dutyCycleR;
//   resetPID();
// }

// void turnLeft(int dutyCycleL, int dutyCycleR) {
//   digitalWrite(dirL1, LOW);
//   digitalWrite(dirL2, HIGH);

//   digitalWrite(dirR1, HIGH);
//   digitalWrite(dirR2, LOW);

//   goalDutyCycleL = dutyCycleL;
//   goalDutyCycleR = dutyCycleR;
//   resetPID();
// }

// void turnRight(int dutyCycleL, int dutyCycleR) {
//   digitalWrite(dirL1, HIGH);
//   digitalWrite(dirL2, LOW);

//   digitalWrite(dirR1, LOW);
//   digitalWrite(dirR2, HIGH);

//   goalDutyCycleL = dutyCycleL;
//   goalDutyCycleR = dutyCycleR;
//   resetPID();
// }

// void stop() {
//   digitalWrite(dirL1, HIGH);
//   digitalWrite(dirL2, HIGH);

//   digitalWrite(dirR1, HIGH);
//   digitalWrite(dirR2, HIGH);

//   goalDutyCycleL = 0;
//   goalDutyCycleR = 0;
//   resetPID();
// }

// /* ========================================================
//    BNO055 HEADING AND TURNING
//    ======================================================== */
// double getHeading() {
//   sensors_event_t event;
//   bno.getEvent(&event);
//   double h = event.orientation.x;
//   if (h < 0) h += 360;
//   return h;
// }

// void turnToAngle(double target) {
//   double KpTurn = 2.0;
//   double KdTurn = 0.4;
//   double prevErr = 0;

//   while (true) {
//     double heading = getHeading();

//     double error = target - heading;
//     if (error > 180) error -= 360;
//     if (error < -180) error += 360;

//     // Scale this down if we can
//     if (abs(error) < 10) {
//       stop();
//       delay(100);
//       break;
//     }

//     double derivative = error - prevErr;
//     prevErr = error;

//     double correction = KpTurn * error + KdTurn * derivative;
//     correction = constrain(correction, -50, 50);

//     if (correction > 0) {
//       turnLeft(abs(correction), abs(correction));
//     } else {
//       turnRight(abs(correction), abs(correction));
//     }

//     setMotor(pwmL, abs(correction));
//     setMotor(pwmR, abs(correction));

//     delay(10);
//   }
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
//   if (!tofLeft.begin(0x40, &Wire)) { Serial.println("Left TOF Fail"); while(1); }

//   digitalWrite(XSHUT_FRONT, HIGH); delay(100);
//   if (!tofFront.begin(0x41, &Wire)) { Serial.println("Front TOF Fail"); while(1); }

//   digitalWrite(XSHUT_RIGHT, HIGH); delay(100);
//   if (!tofRight.begin(0x42, &Wire)) { Serial.println("Right TOF Fail"); while(1); }

//   Serial.println("Initializing BNO055...");
//   if (!bno.begin()) {
//       Serial.println("BNO055 init failed!");
//       while (1);
//   }
//   bno.setExtCrystalUse(true);
//   Serial.println("BNO055 setup OK");

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
//     double target = getHeading() + 90;
//     if (target >= 360) target -= 360;

//     turnToAngle(target);
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

//     // Calculate the duty cycle correction term
//     int correction = error * 0.05;

//     // Update the motor duty cycles based on correction term
//     int leftDutyCycle  = BASE_DUTY_CYCLE - correction;
//     int rightDutyCycle = BASE_DUTY_CYCLE + correction;
//     Serial.printf("Forward L=%d R=%d\n", leftDutyCycle, rightDutyCycle);

//     // Set new forward duty cycle for both motors
//     goForward(leftDutyCycle, rightDutyCycle);
//   }

//   // ======================================
//   // RULE 3: Front blocked & left blocked → turn right
//   // ======================================
//   else if (rightOpen) {
//     // Last resort option before backing up
//     Serial.println("TURNING RIGHT");
//     double target = getHeading() - 90;
//     if (target < 0) target += 360;

//     turnToAngle(target);
//   }

//   // ======================================
//   // RULE 4: Everything blocked → reverse
//   // ======================================
//   else {
//     // Dead end
//     Serial.println("Maze logic: DEAD END → reverse");
//     goBack(REVERSE_DUTY_CYCLE, REVERSE_DUTY_CYCLE);
//   }

//   // ======================================================
//   // PID + SMOOTH RAMP
//   // ======================================================

//   // Calculate the target duty cycle for each motor
//   double targetDutyCycleL = rampTo(outputL, goalDutyCycleL);
//   double targetDutyCycleR = rampTo(outputR, goalDutyCycleR);

//   // Get encoder position reading
//   inputL = posL;
//   inputR = posR;

//   // Compute PID for both motors
//   computePID(inputL, outputL, targetDutyCycleL, prevErrorL, integralL);
//   computePID(inputR, outputR, targetDutyCycleR, prevErrorR, integralR);

//   // Set required PWM for both motors
//   setMotor(pwmL, outputL);
//   setMotor(pwmR, outputR);

//   // Wait 50ms before running again
//   delay(50);
// }
