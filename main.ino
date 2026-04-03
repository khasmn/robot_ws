#include <AccelStepper.h>

// --- PIN DEFINITIONS ---
#define X_DIR_PIN 6
#define X_STEP_PIN 7
#define Y_DIR_PIN 8
#define Y_STEP_PIN 9
#define Z_DIR_PIN 10
#define Z_STEP_PIN 11

#define ENABLE_PIN 5
#define X_LIMIT_PIN 30
#define Y_LIMIT_PIN 26
#define Z_LIMIT_PIN 28

// --- OBJECTS ---
AccelStepper stepperX(AccelStepper::DRIVER, X_STEP_PIN, X_DIR_PIN);
AccelStepper stepperY(AccelStepper::DRIVER, Y_STEP_PIN, Y_DIR_PIN);
AccelStepper stepperZ(AccelStepper::DRIVER, Z_STEP_PIN, Z_DIR_PIN);

// --- SETTINGS ---
float stepsPerMeterXY = 13000.0; 
float stepsPerMeterZ = 38000.0; 
bool isHomed = false;

// --- EXACT SOFT LIMITS (From URDF) ---
float X_MAX = 0.330; 
float Y_MAX = 0.360; 
float Z_MAX = 0.150;

void setup() {
  Serial.begin(115200);

  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW);

  pinMode(X_LIMIT_PIN, INPUT_PULLUP);
  pinMode(Y_LIMIT_PIN, INPUT_PULLUP);
  pinMode(Z_LIMIT_PIN, INPUT_PULLUP);

  stepperX.setPinsInverted(true, false, false); 
  
  stepperX.setMaxSpeed(6000.0); stepperX.setAcceleration(3000.0);
  stepperY.setMaxSpeed(6000.0); stepperY.setAcceleration(3000.0);
  stepperZ.setMaxSpeed(4000.0); stepperZ.setAcceleration(2000.0);

  Serial.println("SYSTEM_READY");
}

void loop() {
  if (Serial.available() > 0) {
    char cmd = Serial.peek();
    
    if (cmd == 'S') { Serial.read(); handleStop(); }
    else if (cmd == 'H') { Serial.read(); digitalWrite(ENABLE_PIN, LOW); runFullHoming(); } 
    else if (cmd == 'T') { Serial.read(); runStepTest(); }
    else if (cmd == 'X') { 
      Serial.read(); 
      if (isHomed) handleMove(); 
    }
    else { Serial.read(); }
  }

  if (isHomed) {
    stepperX.run();
    stepperY.run();
    stepperZ.run();
  }
}

void handleMove() {
  float x = Serial.parseFloat(); 
  if (Serial.find('Y')) {
    float y = Serial.parseFloat();
    if (Serial.find('Z')) {
      float z = Serial.parseFloat();
      
      // Explicit Hard Limits checks for X, Y, and Z
      if (x < 0.0) x = 0.0;
      if (x > X_MAX) x = X_MAX;
      
      if (y < 0.0) y = 0.0;
      if (y > Y_MAX) y = Y_MAX;
      
      if (z < 0.0) z = 0.0;
      if (z > Z_MAX) z = Z_MAX;

      // Move to the strictly limited coordinates
      stepperX.moveTo(x * stepsPerMeterXY);
      stepperY.moveTo(y * stepsPerMeterXY);
      stepperZ.moveTo(z * stepsPerMeterZ);
    }
  }
}

void handleStop() {
  isHomed = false;
  stepperX.stop(); stepperY.stop(); stepperZ.stop();
  digitalWrite(ENABLE_PIN, HIGH);
  Serial.println("ALARM: EMERGENCY STOP");
}

void runFullHoming() {
  isHomed = false;
  Serial.println("STATUS: Homing Started...");
  
  // Home separately in order (Z first to avoid crashing)
  homeAxis(stepperZ, Z_LIMIT_PIN, 1, stepsPerMeterZ);  
  homeAxis(stepperX, X_LIMIT_PIN, 1, stepsPerMeterXY);   
  homeAxis(stepperY, Y_LIMIT_PIN, 1, stepsPerMeterXY);
  
  while(Serial.available() > 0) Serial.read(); 
  isHomed = true;
  Serial.println("HOMING_COMPLETE");
}

void homeAxis(AccelStepper &motor, int limitPin, int direction, float ratio) {
  motor.setMaxSpeed(2000.0);
  
  // 1. Move towards the switch until it is pressed (Goes LOW)
  motor.setSpeed(1200.0 * direction); 
  while (digitalRead(limitPin) == HIGH) { 
    motor.runSpeed(); 
  }
  
  delay(100);

  // 2. Move away slowly until the switch releases (Goes HIGH)
  motor.setSpeed(400.0 * (direction * -1)); 
  while (digitalRead(limitPin) == LOW) { 
    motor.runSpeed(); 
  }

  // 3. Stop immediately. This is the exact 0 point.
  motor.setCurrentPosition(0);
  motor.moveTo(0);
  
  // 4. Restore normal operating speeds
  motor.setMaxSpeed(6000.0);
  motor.setAcceleration(3000.0);
}

void runStepTest() {
  bool wasHomed = isHomed;
  isHomed = false; 
  digitalWrite(ENABLE_PIN, LOW); 
  Serial.println("STATUS: Running Multi-Axis Step Test...");
  
  long targetX = stepperX.currentPosition() + 1000;
  long targetY = stepperY.currentPosition() + 1000;
  long targetZ = stepperZ.currentPosition() + 1000; 
  
  stepperX.moveTo(targetX);
  stepperY.moveTo(targetY);
  stepperZ.moveTo(targetZ);
  
  while (stepperX.distanceToGo() != 0 || stepperY.distanceToGo() != 0 || stepperZ.distanceToGo() != 0) {
    stepperX.run(); stepperY.run(); stepperZ.run();
  }
  
  Serial.println("STATUS: Test Complete.");
  isHomed = wasHomed;
}