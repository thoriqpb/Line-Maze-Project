// mode PWM 70
#define MOTOR_PIN1 6    // Motor A IN1 (Right)
#define MOTOR_PIN2 5    // Motor A IN2 (Right)
#define MOTOR_PIN3 10   // Motor B IN1 (Left)
#define MOTOR_PIN4 11   // Motor B IN2 (Left)

const int selectPins[] = {2, 3, 4}; 
const int analogPin = A0;           
const int threshold = 230;          
const int SPEED_BASE = 70;         
const int TURN_SPEED = 100;        // Speed for sharp turns
const int INTERSECTION_DELAY = 250; // Delay at intersections
const int SEARCH_SPEED = 80;       // Speed for line searching
const unsigned long SEARCH_TIMEOUT = 2000; // Max search time (ms)

// PID constants
const float scaler = 17;       
const float Kp = 1.0;               
const float Ki = 0.0001;               
const float Kd = 0.01;              

float previousError = 0;            
float integral = 0;                 
unsigned long lastLineTime = 0;     // Track when line was last seen
bool searchDirection = false;       // Toggle search direction

// Motor directions
#define FORWARD 1
#define BACKWARD 0

void setup() {
  Serial.begin(9600);
  pinMode(MOTOR_PIN1, OUTPUT); pinMode(MOTOR_PIN2, OUTPUT);
  pinMode(MOTOR_PIN3, OUTPUT); pinMode(MOTOR_PIN4, OUTPUT);
  for (int i = 0; i < 3; i++) pinMode(selectPins[i], OUTPUT);
}

void loop() {
  int sensorValues[8], weightedSum = 0, sum = 0;
  bool anySensorDetectsLine = false, allSensorsDetectLine = true;
  bool isRightTurn = false, isLeftTurn = false, isIntersection = false;

  float weights[8] = {-7, -6, -1.5, -0.5, 0.5, 1.5, 2, 3};  

  // Read all sensors
  for (int i = 0; i < 8; i++) {
    selectChannel(i);
    int val = analogRead(analogPin);
    sensorValues[i] = val;
    if (val < threshold) {
      anySensorDetectsLine = true;
      lastLineTime = millis(); // Update last line seen time
    } else {
      allSensorsDetectLine = false;
    }

    float weight = weights[i];
    int sensorActive = (val < threshold) ? 1000 : 0;
    weightedSum += sensorActive * weight;
    sum += sensorActive;
  }

  // Detect turn conditions
  if (sensorValues[0] < threshold || sensorValues[1] < threshold) {
    isRightTurn = true;
  }
  if (sensorValues[6] < threshold || sensorValues[7] < threshold) {
    isLeftTurn = true;
  }
  
  // Detect 4-way intersection
  if ((sensorValues[0] < threshold || sensorValues[1] < threshold) && 
      (sensorValues[6] < threshold || sensorValues[7] < threshold)) {
    isIntersection = true;
  }

  // Print sensor values
  for (int i = 0; i < 8; i++) {
    Serial.print(sensorValues[i]);
    Serial.print("\t");
  }

  if (allSensorsDetectLine) {
    stopMotors();
    Serial.println("stop");
  }
  else if (!anySensorDetectsLine) {
    // No line detected - rotate to find it (alternating directions)
    unsigned long searchStart = millis();
    Serial.print("No line - Searching ");
    Serial.print(searchDirection ? "RIGHT" : "LEFT");
    Serial.println("...");
    
    while(millis() - searchStart < SEARCH_TIMEOUT) {
      if (searchDirection) {
        // Rotate right (clockwise)
        setMotor(MOTOR_PIN1, MOTOR_PIN2, FORWARD, SEARCH_SPEED); // Right forward
        setMotor(MOTOR_PIN3, MOTOR_PIN4, BACKWARD, SEARCH_SPEED); // Left backward
      } else {
        // Rotate left (counter-clockwise)
        setMotor(MOTOR_PIN1, MOTOR_PIN2, BACKWARD, SEARCH_SPEED); // Right backward
        setMotor(MOTOR_PIN3, MOTOR_PIN4, FORWARD, SEARCH_SPEED); // Left forward
      }
      
      // Check sensors while searching
      bool foundLine = false;
      for (int i = 0; i < 8; i++) {
        selectChannel(i);
        if (analogRead(analogPin) < threshold) {
          foundLine = true;
          break;
        }
      }
      
      if (foundLine) {
        Serial.println("Line found!");
        lastLineTime = millis();
        searchDirection = !searchDirection; // Toggle direction for next time
        break;
      }
    }
    
    if (millis() - searchStart >= SEARCH_TIMEOUT) {
      Serial.println("Search timeout");
      stopMotors();
      searchDirection = !searchDirection; // Toggle direction for next time
    }
  }
  else if (isIntersection) {
    // At intersection, always prefer right turn
    setMotor(MOTOR_PIN1, MOTOR_PIN2, BACKWARD, TURN_SPEED); // Right backward
    setMotor(MOTOR_PIN3, MOTOR_PIN4, FORWARD, TURN_SPEED);  // Left forward
    Serial.print("Intersection - Right turn\t");
    delay(INTERSECTION_DELAY);
    integral = 0;
  } 
  else if (isRightTurn) {
    // Normal right turn
    setMotor(MOTOR_PIN1, MOTOR_PIN2, BACKWARD, TURN_SPEED); // Right backward
    setMotor(MOTOR_PIN3, MOTOR_PIN4, FORWARD, TURN_SPEED-10); // Left forward
    Serial.print("Right turn\t");
    delay(150);
    integral = 0;
  }
  else if (isLeftTurn) {
    // Left turn
    setMotor(MOTOR_PIN1, MOTOR_PIN2, FORWARD, TURN_SPEED-10); // Right forward
    setMotor(MOTOR_PIN3, MOTOR_PIN4, BACKWARD, TURN_SPEED);   // Left backward
    Serial.print("Left turn\t");
    delay(150);
    integral = 0;
  } 
  else {
    // Normal PID control
    float error = (float)weightedSum / sum;
    integral += error;
    float derivative = error - previousError;
    float output = Kp * error + Ki * integral + Kd * derivative;
    previousError = error;

    int adjustment = output * scaler;
    int leftSpeed = constrain(SPEED_BASE + adjustment, 0, 255);
    int rightSpeed = constrain(SPEED_BASE - adjustment, 0, 255);

    setMotorSpeeds(leftSpeed, rightSpeed);

    Serial.print("PID: ");
    Serial.print(output);
    Serial.print("\tPWM Right: ");
    Serial.print(leftSpeed);
    Serial.print("\tPWM Left: ");
    Serial.print(rightSpeed);
  }
  Serial.println();
}

// Enhanced motor control function
void setMotor(int pin1, int pin2, int direction, int speed) {
  if (direction == FORWARD) {
    analogWrite(pin1, speed);
    analogWrite(pin2, 0);
  } else {
    analogWrite(pin1, 0);
    analogWrite(pin2, speed);
  }
}

void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  setMotor(MOTOR_PIN1, MOTOR_PIN2, FORWARD, leftSpeed);
  setMotor(MOTOR_PIN3, MOTOR_PIN4, FORWARD, rightSpeed);
}

void stopMotors() {
  analogWrite(MOTOR_PIN1, 0); analogWrite(MOTOR_PIN2, 0);
  analogWrite(MOTOR_PIN3, 0); analogWrite(MOTOR_PIN4, 0);
}

void selectChannel(int channel) {
  for (int i = 0; i < 3; i++) digitalWrite(selectPins[i], (channel >> i) & 1);
}