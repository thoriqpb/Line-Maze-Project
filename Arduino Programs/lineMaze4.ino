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
#define FORWARD 1
#define BACKWARD 0

// Sensor and control configuration
const int selectPins[] = {2, 3, 4}; 
const int analogPin = A0;           
const int threshold = 230;          
const int SPEED_BASE = 30;         
const int TURN_SPEED = 60;        
const int INTERSECTION_DELAY = 150; 
const int SEARCH_SPEED = 50;       
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

String currentDirection = "STOP";
String currentPath = "-";
String pathHistory = "";
int currentLeftPWM = 0;
int currentRightPWM = 0;

void setup() {
  Serial.begin(9600);
  pinMode(MOTOR_PIN1, OUTPUT); pinMode(MOTOR_PIN2, OUTPUT);
  pinMode(MOTOR_PIN3, OUTPUT); pinMode(MOTOR_PIN4, OUTPUT);
  for (int i = 0; i < 3; i++) pinMode(selectPins[i], OUTPUT);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("OLED failed"));
    while (true);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("OLED Ready");
  display.display();
  delay(1000);
}

void loop() {
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

    Serial.print(val); Serial.print("\t"); 
  }
  Serial.println();

  frontSensorsDetectLine = (sensorValues[3] < 250 || sensorValues[4] < 250);
  bool leftSide = (sensorValues[6] < 380 || sensorValues[7] < 380);
  bool rightSide = (sensorValues[0] < 280 || sensorValues[1] < 280);
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
    delay(55);  // Confirm before acting

    // Re-read sensors
    for (int i = 0; i < 8; i++) {
      selectChannel(i);
      sensorValues[i] = analogRead(analogPin);
    }

    // Recalculate detection flags
    frontSensorsDetectLine = (sensorValues[3] < 250-30 || sensorValues[4] < 250-30);
    leftSide = (sensorValues[6] < 380 || sensorValues[7] < 380);
    rightSide = (sensorValues[0] < 280 || sensorValues[1] < 280);
    leftFront = frontSensorsDetectLine && leftSide;
    rightFront = frontSensorsDetectLine && rightSide;

    // intersectionAll = frontSensorsDetectLine && leftSide && rightSide;
    intersectionSide = leftSide && rightSide;
    intersectionFrontLeft = leftFront;
    intersectionFrontRight = rightFront;
    isLeftTurn = leftSide && !frontSensorsDetectLine;
    isRightTurn = rightSide && !frontSensorsDetectLine;
  }

  if ((sensorValues[2] < 380 && sensorValues[3] < 380) && (sensorValues[4] < 380 & sensorValues[5] < 380)) {
    stopMotors();
    currentDirection = "STOP";
    currentPath = "-";
  }
  else if (!anySensorDetectsLine) {
    unsigned long searchStart = millis();
    currentDirection = "SEARCH";
    currentPath = "U";  

    while (millis() - searchStart < SEARCH_TIMEOUT) {
      setMotor(MOTOR_PIN1, MOTOR_PIN2, BACKWARD, SEARCH_SPEED);
      setMotor(MOTOR_PIN3, MOTOR_PIN4, FORWARD, SEARCH_SPEED);
      currentLeftPWM = SEARCH_SPEED;
      currentRightPWM = SEARCH_SPEED;

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
    setMotor(MOTOR_PIN1, MOTOR_PIN2, BACKWARD, TURN_SPEED);
    setMotor(MOTOR_PIN3, MOTOR_PIN4, FORWARD, TURN_SPEED - 10);
    currentDirection = "INT-SIDE";
    currentPath = "R";
    appendToPath(currentPath);
    delay(INTERSECTION_DELAY);
    integral = 0;
  }
  //   else if (intersectionAll) {
  //   setMotor(MOTOR_PIN1, MOTOR_PIN2, BACKWARD, TURN_SPEED);
  //   setMotor(MOTOR_PIN3, MOTOR_PIN4, FORWARD, TURN_SPEED - 10);
  //   currentDirection = "INT-ALL";
  //   currentPath = "R";
  //   appendToPath(currentPath);
  //   delay(INTERSECTION_DELAY);
  //   integral = 0;
  // }
  else if (intersectionFrontLeft) {
    setMotor(MOTOR_PIN1, MOTOR_PIN2, FORWARD, SPEED_BASE);
    setMotor(MOTOR_PIN3, MOTOR_PIN4, FORWARD, SPEED_BASE);
    currentDirection = "INT-FL";
    currentPath = "S";
    appendToPath(currentPath);
    integral = 0;
  }
  else if (intersectionFrontRight) {
    setMotor(MOTOR_PIN1, MOTOR_PIN2, BACKWARD, TURN_SPEED);
    setMotor(MOTOR_PIN3, MOTOR_PIN4, FORWARD, TURN_SPEED - 10);
    currentDirection = "INT-FR";
    currentPath = "R";
    appendToPath(currentPath);
    delay(INTERSECTION_DELAY);
    integral = 0;
  }
  else if (isRightTurn) {
    setMotor(MOTOR_PIN1, MOTOR_PIN2, BACKWARD, TURN_SPEED);
    setMotor(MOTOR_PIN3, MOTOR_PIN4, FORWARD, TURN_SPEED - 10);
    currentDirection = "RIGHT";
    currentPath = "R";
    appendToPath(currentPath);
    delay(20);
    integral = 0;
  }
  else if (isLeftTurn) {
    setMotor(MOTOR_PIN1, MOTOR_PIN2, FORWARD, TURN_SPEED - 10);
    setMotor(MOTOR_PIN3, MOTOR_PIN4, BACKWARD, TURN_SPEED);
    currentDirection = "LEFT";
    currentPath = "L";
    appendToPath(currentPath);
    delay(100);
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
    int rightSpeed = constrain(SPEED_BASE - adjustment, 0, 255);

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
  analogWrite(MOTOR_PIN1, 0); analogWrite(MOTOR_PIN2, 0);
  analogWrite(MOTOR_PIN3, 0); analogWrite(MOTOR_PIN4, 0);
  currentLeftPWM = 0;
  currentRightPWM = 0;
}

void selectChannel(int channel) {
  for (int i = 0; i < 3; i++) digitalWrite(selectPins[i], (channel >> i) & 1);
}

void displayStatus() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Dir: "); display.println(currentDirection);
  display.print("PWM L: "); display.println(currentLeftPWM);
  display.print("PWM R: "); display.println(currentRightPWM);
  display.print("Path: "); display.println(pathHistory);
  display.display();
}

void appendToPath(String pathChar) {
  unsigned long currentMillis = millis();
  if (currentMillis - lastMemoryTime >= 800 && pathHistory.length() < 20) {
    pathHistory += pathChar;
    lastMemoryTime = currentMillis;
  }
}
