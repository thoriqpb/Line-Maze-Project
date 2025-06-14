#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// Motor control
#define MOTOR_PIN1 6    
#define MOTOR_PIN2 5    
#define MOTOR_PIN3 10   
#define MOTOR_PIN4 11  

#define BUZZER_PIN 8

#define FORWARD 1
#define BACKWARD 0

// Sensor and control configuration
const int selectPins[] = {2, 3, 4}; 
const int analogPin = A0;           
const int threshold = 230;          
const int SPEED_BASE = 30;         
const int TURN_SPEED = 60;          
const int INTERSECTION_DELAY = 80; 
const int SEARCH_SPEED = 40;       
const unsigned long SEARCH_TIMEOUT = 2000; 

// PID constants
const float scaler = 17;       
const float Kp = 1.0;               
const float Ki = 0.0001;               
const float Kd = 0.01;              

float previousError = 0;
float integral = 0;
unsigned long lastLineTime = 0;
unsigned long lastMemoryTime = 0;
unsigned long lastIntersectionTime = 0;
const unsigned long INTERSECTION_COOLDOWN = 1000;  // ms 

String currentDirection = "STOP";
String currentPath = "-";
String pathHistory = "";
int currentLeftPWM = 0;
int currentRightPWM = 0;

// String fixedPath = "";  // Editable path
int fixedPathIndex = 0;
bool pathCompleted = false;

bool preferRight = 1; // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< TRUE = MODE KANAN >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

void setup() {
  Serial.begin(9600);
 
  pinMode(MOTOR_PIN1, OUTPUT); pinMode(MOTOR_PIN2, OUTPUT);
  pinMode(MOTOR_PIN3, OUTPUT); pinMode(MOTOR_PIN4, OUTPUT);
  for (int i = 0; i < 3; i++) pinMode(selectPins[i], OUTPUT);

  pinMode(12, INPUT_PULLUP);  // Button for path simplification
  pinMode(BUZZER_PIN, OUTPUT);  // <<< Add this line

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("OLED failed"));
    while (true);
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Line Maze Ready");
  display.display();
  
  delay(1000);
}

void loop() {
  if (digitalRead(12) == LOW) {
    pathHistory = simplifyPath(pathHistory);
    fixedPathIndex = 0;
    pathCompleted = false;

    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Path Simplified!");
    display.println(pathHistory);
    display.display();
    delay(1000);
  }

  int sensorValues[8], weightedSum = 0, sum = 0;
  bool anySensorDetectsLine = false, frontSensorsDetectLine = false;
  bool isRightTurn = false, isLeftTurn = false;
  bool intersectionAll = false;
  bool intersectionSide = false;
  bool intersectionFrontLeft = false;
  bool intersectionFrontRight = false;

  float weights[8] = {-3, -2, -1.2, -0.5, 0.5, 1.2, 2, 3};  

  for (int i = 0; i < 8; i++) {
    selectChannel(i);
    int val = analogRead(analogPin);
    sensorValues[i] = val;
    if (val < threshold) {
      anySensorDetectsLine = true;
      lastLineTime = millis();
    }

    float weight = weights[i];
    int sensorActive = (val < threshold) ? 1000 : 0;
    weightedSum += sensorActive * weight;
    sum += sensorActive;

    Serial.print(val); 
    Serial.print("\t"); 
  }
  Serial.println();

  frontSensorsDetectLine = (sensorValues[3] < 150 || sensorValues[4] < 200);
  bool leftSide = (sensorValues[6] < 450 || sensorValues[7] < 320);
  bool rightSide = (sensorValues[0] < 400 || sensorValues[1] < 250);
  bool leftFront = frontSensorsDetectLine && leftSide;
  bool rightFront = frontSensorsDetectLine && rightSide;

  intersectionAll = frontSensorsDetectLine && leftSide && rightSide;
  intersectionSide = leftSide && rightSide;
  intersectionFrontLeft = leftFront;
  intersectionFrontRight = rightFront;
  isLeftTurn = leftSide && !frontSensorsDetectLine;
  isRightTurn = rightSide && !frontSensorsDetectLine;

  bool allSensorsBelow100 = true;
  for (int i = 0; i < 8; i++) {
    if (sensorValues[i] >= 200) {
      allSensorsBelow100 = false;
      break;
    }
  }

  // === Confirm side detection with delay ===
  if (isLeftTurn || isRightTurn || intersectionSide || intersectionFrontLeft || intersectionFrontRight) {
    delay(110);  // Confirm before acting

    // Re-read sensors
    for (int i = 0; i < 8; i++) {
      selectChannel(i);
      sensorValues[i] = analogRead(analogPin);
    }

    // Recalculate detection flags
    frontSensorsDetectLine = (sensorValues[3] < 150 || sensorValues[4] < 200);
    leftSide = (sensorValues[6] < 450 || sensorValues[7] < 320);
    rightSide = (sensorValues[0] < 400 || sensorValues[1] < 250);
    leftFront = frontSensorsDetectLine && leftSide;
    rightFront = frontSensorsDetectLine && rightSide;

    intersectionSide = leftSide && rightSide;
    intersectionFrontLeft = leftFront;
    intersectionFrontRight = rightFront;
    isLeftTurn = leftSide && !frontSensorsDetectLine;
    isRightTurn = rightSide && !frontSensorsDetectLine;
  }

  if ((sensorValues[2] < 350 && sensorValues[3] < 150) && (sensorValues[4] < 150 && sensorValues[5] < 350)) {
    stopMotors();
    currentDirection = "STOP";
    currentPath = "-";
  }
  else if (!anySensorDetectsLine) {
    unsigned long searchStart = millis();
    currentDirection = "SEARCH";
    currentPath = "U";  

    while (millis() - searchStart < SEARCH_TIMEOUT) {
      if (preferRight) {
        setMotor(MOTOR_PIN1, MOTOR_PIN2, BACKWARD, SEARCH_SPEED);
        setMotor(MOTOR_PIN3, MOTOR_PIN4, FORWARD, SEARCH_SPEED);
        currentLeftPWM = SEARCH_SPEED;
        currentRightPWM = SEARCH_SPEED;
      } else if (!preferRight) {
        setMotor(MOTOR_PIN1, MOTOR_PIN2, FORWARD, SEARCH_SPEED);
        setMotor(MOTOR_PIN3, MOTOR_PIN4, BACKWARD, SEARCH_SPEED);
        currentLeftPWM = SEARCH_SPEED;
        currentRightPWM = SEARCH_SPEED;
      }

      bool foundLine = false;
      for (int i = 0; i < 8; i++) {
        selectChannel(i);
        if (analogRead(analogPin) < threshold) {
          foundLine = true;
          break;
        }
      }
      if (foundLine) {
        lastLineTime = millis();
        break;
      }
    }
    if (millis() - searchStart >= SEARCH_TIMEOUT) {
      stopMotors();
      currentDirection = "TIMEOUT";
    }
    appendToPath(currentPath);
  }

  else if (intersectionSide) {
    // Both sides detected
    if (preferRight) {
      currentDirection = "INT-SIDE";
      currentPath = "R";
      setMotor(MOTOR_PIN1, MOTOR_PIN2, BACKWARD, TURN_SPEED);
      setMotor(MOTOR_PIN3, MOTOR_PIN4, FORWARD, TURN_SPEED - 10 + 15);
    } else {
      currentDirection = "INT-SIDE";
      currentPath = "L";
      setMotor(MOTOR_PIN1, MOTOR_PIN2, FORWARD, TURN_SPEED - 10);
      setMotor(MOTOR_PIN3, MOTOR_PIN4, BACKWARD, TURN_SPEED + 15);
    }
    appendToPath(currentPath);
    delay(INTERSECTION_DELAY);
    integral = 0;
  }

  else if ((preferRight && intersectionFrontLeft) || (!preferRight && intersectionFrontRight)) {
    // Straight path on preferred side's front
    setMotor(MOTOR_PIN1, MOTOR_PIN2, FORWARD, SPEED_BASE);
    setMotor(MOTOR_PIN3, MOTOR_PIN4, FORWARD, SPEED_BASE + 15);
    currentDirection = "STRAIGHT";
    currentPath = "S";
    appendToPath(currentPath);
    integral = 0;
  }

  else if ((preferRight && intersectionFrontRight) || (!preferRight && intersectionFrontLeft)) {
    // Turn toward preferred direction at intersection
    if (preferRight) {
      setMotor(MOTOR_PIN1, MOTOR_PIN2, BACKWARD, TURN_SPEED);
      setMotor(MOTOR_PIN3, MOTOR_PIN4, FORWARD, TURN_SPEED - 10 + 15);
      currentDirection = "RIGHT";
      currentPath = "R";
    } else {
      setMotor(MOTOR_PIN1, MOTOR_PIN2, FORWARD, TURN_SPEED - 10);
      setMotor(MOTOR_PIN3, MOTOR_PIN4, BACKWARD, TURN_SPEED + 15);
      currentDirection = "LEFT";
      currentPath = "L";
    }
    appendToPath(currentPath);  // Save only when turning at intersections
    delay(INTERSECTION_DELAY);
    integral = 0;
  }

  else if (isRightTurn) {
    // Regular right turn — move robot but DO NOT save path
    setMotor(MOTOR_PIN1, MOTOR_PIN2, BACKWARD, TURN_SPEED);
    setMotor(MOTOR_PIN3, MOTOR_PIN4, FORWARD, TURN_SPEED + 15);
    currentDirection = "RIGHT";
    // NO appendToPath here!
    delay(50);
    integral = 0;
  }
  else if (isLeftTurn) {
    // Regular left turn — move robot but DO NOT save path
    setMotor(MOTOR_PIN1, MOTOR_PIN2, FORWARD, TURN_SPEED);
    setMotor(MOTOR_PIN3, MOTOR_PIN4, BACKWARD, TURN_SPEED + 15);
    currentDirection = "LEFT";
    // NO appendToPath here!
    delay(50);
    integral = 0;
  }


  else {
    float error = (float)weightedSum / sum;
    integral += error;
    float derivative = error - previousError;
    float output = Kp * error + Ki * integral + Kd * derivative;
    previousError = error;

    int adjustment = output * scaler;
    int leftSpeed = constrain(SPEED_BASE + adjustment, 0, 255);
    int rightSpeed = constrain(SPEED_BASE - adjustment + 15, 0, 255);

    setMotorSpeeds(leftSpeed, rightSpeed);
    currentLeftPWM = leftSpeed;
    currentRightPWM = rightSpeed;
    currentDirection = "FORWARD";
    currentPath = "-";
  }
  displayStatus();
}

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
  delay(130);
  analogWrite(MOTOR_PIN1, 0);
  analogWrite(MOTOR_PIN2, 0);
  analogWrite(MOTOR_PIN3, 0);
  analogWrite(MOTOR_PIN4, 0);
  currentLeftPWM = 0;
  currentRightPWM = 0;
  Beep();
}

void selectChannel(int channel) {
  for (int i = 0; i < 3; i++) digitalWrite(selectPins[i], (channel >> i) & 1);
}

void displayStatus() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Dir: "); 
  display.println(currentDirection);
  display.print("PWM L: "); 
  display.println(currentLeftPWM);
  display.print("PWM R: "); 
  display.println(currentRightPWM);
  display.print("Path: "); 
  display.println(pathHistory);
  display.display();
}

void appendToPath(String pathChar) {
  unsigned long currentMillis = millis();
  if (currentMillis - lastMemoryTime >= 1500 && pathHistory.length() < 20) {
    pathHistory += pathChar;
    lastMemoryTime = currentMillis;
  }
}

void Beep() {
  for (int i = 0; i < 3; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(10);
    digitalWrite(BUZZER_PIN, LOW);
    delay(10);
  }
}

String simplifyPath(String path) {
  for (int i = 0; i < 5; i++) {
    if (preferRight) {
      path.replace("SUR", "L");
      path.replace("RUR", "S");
      path.replace("RUL", "U");
    } else {
      path.replace("SUL", "R");
      path.replace("LUL", "S");
      path.replace("LUR", "U");
    }
  }
  return path;
}
