#define MOTOR_PIN1 6    // Motor A IN1 (Right)
#define MOTOR_PIN2 5    // Motor A IN2 (Right)
#define MOTOR_PIN3 10     // Motor B IN1 (Left)
#define MOTOR_PIN4 11     // Motor B IN2 (Left)

const float speed = 50;

void setup() {
  // Mengatur pin sebagai output
  pinMode(MOTOR_PIN1, OUTPUT);
  pinMode(MOTOR_PIN2, OUTPUT);
  pinMode(MOTOR_PIN3, OUTPUT);
  pinMode(MOTOR_PIN4, OUTPUT);
  
  // Memastikan motor berhenti saat startup
  // analogWrite(MOTOR_PIN1, speed); // 
  // analogWrite(MOTOR_PIN2, 0); // 
  // analogWrite(MOTOR_PIN3, speed); // 
  // analogWrite(MOTOR_PIN4, 0); //   

  analogWrite(MOTOR_PIN1, 0); // 
  analogWrite(MOTOR_PIN2, speed); // 
  analogWrite(MOTOR_PIN3, 0); // 
  analogWrite(MOTOR_PIN4, speed); // 
}

// 5 12 gabisa PWM

void loop() {
}