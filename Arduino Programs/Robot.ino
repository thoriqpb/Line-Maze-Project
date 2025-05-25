#define MOTOR_PIN1 6    // Motor A IN1
#define MOTOR_PIN2 5    // Motor A IN2
#define MOTOR_PIN3 11   // Motor B IN1
#define MOTOR_PIN4 10   // Motor B IN2

const int selectPins[] = {2, 3, 4}; // Multiplexer select pins: S1, S2, S3
const int analogPin = A0;           // Analog pin to read sensor values
const int threshold = 200;          // Line detection threshold

// Define motor speeds at the beginning for easy tweaking
const int SPEED_FORWARD = 150;
const int SPEED_SLOW_FORWARD = 100;
const int SPEED_TURN_FAST = 150;
const int SPEED_TURN_SLOW = 80;

void setup() {
  Serial.begin(9600);
  
  pinMode(MOTOR_PIN1, OUTPUT);
  pinMode(MOTOR_PIN2, OUTPUT);
  pinMode(MOTOR_PIN3, OUTPUT);
  pinMode(MOTOR_PIN4, OUTPUT);
  
  for (int i = 0; i < 3; i++) {
    pinMode(selectPins[i], OUTPUT);
  }
}

void loop() {
  int sensorValues[8];
  bool anySensorDetectsLine = false;
  bool allSensorsDetectLine = true;

  // Read all sensor values and store
  for (int channel = 0; channel < 8; channel++) {
    selectChannel(channel);
    int value = analogRead(analogPin);
    sensorValues[channel] = value;

    if (value < threshold) {
      anySensorDetectsLine = true;
    } else {
      allSensorsDetectLine = false;
    }
  }

  // Print sensor values with tabs
  for (int i = 0; i < 8; i++) {
    Serial.print(sensorValues[i]);
    Serial.print("\t");
  }

  // Decision based on center sensors 5,4,3,2
  bool center4 = sensorValues[4] < threshold;
  bool center3 = sensorValues[3] < threshold;
  bool center5 = sensorValues[5] < threshold;
  bool center2 = sensorValues[2] < threshold;

  if (allSensorsDetectLine) {
    // All sensors detect black block → stop
    stopMotors();
    Serial.println("stop");
  }
  else if (anySensorDetectsLine) {
    if (center4 && center3) {
      // Line exactly centered → go forward
      forwardMotors(SPEED_FORWARD);
      Serial.println("forward");
    } 
    else if (center5) {
      // Line detected more to left → slight left turn
      turnLeft();
      Serial.println("left");
    }
    else if (center2) {
      // Line detected more to right → slight right turn
      turnRight();
      Serial.println("right");
    }
    else {
      // Line detected but center sensors not triggered → slow forward
      forwardMotors(SPEED_SLOW_FORWARD);
      Serial.println("forward_slow");
    }
  }
  else {
    // No sensors detect line → stop
    stopMotors();
    Serial.println("stop");
  }

  delay(100);
}

// Helper functions to control motors
void forwardMotors(int speed) {
  analogWrite(MOTOR_PIN1, speed);
  analogWrite(MOTOR_PIN2, 0);
  analogWrite(MOTOR_PIN3, speed);
  analogWrite(MOTOR_PIN4, 0);
}

void stopMotors() {
  analogWrite(MOTOR_PIN1, 0);
  analogWrite(MOTOR_PIN2, 0);
  analogWrite(MOTOR_PIN3, 0);
  analogWrite(MOTOR_PIN4, 0);
}

void turnLeft() {
  analogWrite(MOTOR_PIN1, SPEED_TURN_SLOW);
  analogWrite(MOTOR_PIN2, 0);
  analogWrite(MOTOR_PIN3, SPEED_TURN_FAST);
  analogWrite(MOTOR_PIN4, 0);
}

void turnRight() {
  analogWrite(MOTOR_PIN1, SPEED_TURN_FAST);
  analogWrite(MOTOR_PIN2, 0);
  analogWrite(MOTOR_PIN3, SPEED_TURN_SLOW);
  analogWrite(MOTOR_PIN4, 0);
}

void selectChannel(int channel) {
  for (int i = 0; i < 3; i++) {
    digitalWrite(selectPins[i], (channel >> i) & 1);
  }
}
