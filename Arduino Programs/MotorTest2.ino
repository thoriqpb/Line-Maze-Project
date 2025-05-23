#define MOTOR_PIN1 5    // Motor A IN1
#define MOTOR_PIN2 6    // Motor A IN2
#define MOTOR_PIN3 16   // Motor B IN1
#define MOTOR_PIN4 17   // Motor B IN2

void setup() {
  pinMode(MOTOR_PIN1, OUTPUT);
  pinMode(MOTOR_PIN2, OUTPUT);
  pinMode(MOTOR_PIN3, OUTPUT);
  pinMode(MOTOR_PIN4, OUTPUT);

  analogWrite(MOTOR_PIN1, 0);
  analogWrite(MOTOR_PIN2, 0);
  analogWrite(MOTOR_PIN3, 0);
  analogWrite(MOTOR_PIN4, 0);
}

void loop() {
  // Forward
  for (int speed = 0; speed <= 255; speed++) {
    analogWrite(MOTOR_PIN1, speed);
    analogWrite(MOTOR_PIN3, speed);
    analogWrite(MOTOR_PIN2, 0);
    analogWrite(MOTOR_PIN4, 0);
    delay(20);
  }
  delay(1000);

  // Stop
  for (int speed = 255; speed >= 0; speed--) {
    analogWrite(MOTOR_PIN1, speed);
    analogWrite(MOTOR_PIN3, speed);
    delay(20);
  }
  delay(300);

  // Reverse
  for (int speed = 0; speed <= 255; speed++) {
    analogWrite(MOTOR_PIN2, speed);
    analogWrite(MOTOR_PIN4, speed);
    analogWrite(MOTOR_PIN1, 0);
    analogWrite(MOTOR_PIN3, 0);
    delay(20);
  }
  delay(1000);

  // Stop
  for (int speed = 255; speed >= 0; speed--) {
    analogWrite(MOTOR_PIN2, speed);
    analogWrite(MOTOR_PIN4, speed);
    delay(20);
  }
  delay(300);
}
