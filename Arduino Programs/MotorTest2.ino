#define MOTOR_PIN1 6    // Motor A IN1 (Right)
#define MOTOR_PIN2 5    // Motor A IN2 (Right)
#define MOTOR_PIN3 10     // Motor B IN1 (Left)
#define MOTOR_PIN4 11     // Motor B IN2 (Left)

const float maxSpeed = 80;

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
  for (int speed = 0; speed <= maxSpeed; speed++) {
    analogWrite(MOTOR_PIN1, speed);
    analogWrite(MOTOR_PIN3, speed);
    analogWrite(MOTOR_PIN2, 0);
    analogWrite(MOTOR_PIN4, 0);
    delay(20);
  }
  delay(1000);

  // Stop
  for (int speed = maxSpeed; speed >= 0; speed--) {
    analogWrite(MOTOR_PIN1, speed);
    analogWrite(MOTOR_PIN3, speed);
    delay(20);
  }
  // delay(300);

  // Reverse
  for (int speed = 0; speed <= maxSpeed; speed++) {
    analogWrite(MOTOR_PIN2, speed);
    analogWrite(MOTOR_PIN4, speed);
    analogWrite(MOTOR_PIN1, 0);
    analogWrite(MOTOR_PIN3, 0);
    delay(20);
  }
  delay(1000);

  // Stop
  for (int speed = maxSpeed; speed >= 0; speed--) {
    analogWrite(MOTOR_PIN2, speed);
    analogWrite(MOTOR_PIN4, speed);
    delay(20);
  }
  // delay(300);
}
