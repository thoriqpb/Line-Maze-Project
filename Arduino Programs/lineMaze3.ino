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
const int SPEED_BASE = 50;         
const int TURN_SPEED = 60;        
const int INTERSECTION_DELAY = 100; 
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
  bool isRightTurn = false, isLeftTurn = false, isIntersection = false;

  float weights[8] = {-3, -2, -1.2, -0.5, 0.5, 1.5, 2, 3};  

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

    Serial.print(val); Serial.print("\t"); // Debug output
  }

  Serial.println();

  frontSensorsDetectLine = (sensorValues[2] < threshold && 
                            sensorValues[3] < threshold && 
                            sensorValues[4] < threshold && 
                            sensorValues[5] < threshold);

  if (sensorValues[0] < 380 || sensorValues[1] < 380) {
    isRightTurn = true;
  }
  if (sensorValues[6] < threshold || sensorValues[7] < 350) {
    isLeftTurn = true;
  }
  if ((sensorValues[0] < 390 || sensorValues[1] < 390) && 
      (sensorValues[6] < 390 || sensorValues[7] < 390)) {
    isIntersection = true;
  }

  if (frontSensorsDetectLine) {
    stopMotors();
    currentDirection = "STOP";
    currentPath = "-";
  }
  else if (!anySensorDetectsLine) {
    unsigned long searchStart = millis();
    currentDirection = "SEARCH";
    currentPath = "-";

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
  }
  else if (isIntersection) {
    setMotor(MOTOR_PIN1, MOTOR_PIN2, BACKWARD, TURN_SPEED);
    setMotor(MOTOR_PIN3, MOTOR_PIN4, FORWARD, TURN_SPEED - 10);
    currentLeftPWM = TURN_SPEED - 10;
    currentRightPWM = TURN_SPEED;
    currentDirection = "RIGHT";
    currentPath = "I";
    appendToPath(currentPath);
    delay(INTERSECTION_DELAY);
    integral = 0;
  } 
  else if (isRightTurn) {
    setMotor(MOTOR_PIN1, MOTOR_PIN2, BACKWARD, TURN_SPEED);
    setMotor(MOTOR_PIN3, MOTOR_PIN4, FORWARD, TURN_SPEED - 10);
    currentLeftPWM = TURN_SPEED - 10;
    currentRightPWM = TURN_SPEED;
    currentDirection = "RIGHT";
    currentPath = "R";
    appendToPath(currentPath);
    delay(30);
    integral = 0;
  }
  else if (isLeftTurn) {
    setMotor(MOTOR_PIN1, MOTOR_PIN2, FORWARD, TURN_SPEED - 10);
    setMotor(MOTOR_PIN3, MOTOR_PIN4, BACKWARD, TURN_SPEED);
    currentLeftPWM = TURN_SPEED;
    currentRightPWM = TURN_SPEED - 10;
    currentDirection = "LEFT";
    currentPath = "L";
    appendToPath(currentPath);
    delay(30);
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
    currentPath = "U";
    appendToPath(currentPath);
    delay(30);
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
  delay(50);
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
  if (pathHistory.length() < 20) {
    pathHistory += pathChar;
  }
  delay(30); // Small delay to prevent overflow
}

