// Program untuk mengendalikan motor DC menggunakan H-Bridge pada Arduino Nano
// Pin D5 dan D6 digunakan untuk kontrol H-Bridge

#define MOTOR_PIN1 5    // Pin D5 untuk kontrol H-Bridge (IN1)
#define MOTOR_PIN2 6    // Pin D6 untuk kontrol H-Bridge (IN2)
#define MOTOR_PIN3 16    // Pin D5 untuk kontrol H-Bridge (IN1)
#define MOTOR_PIN4 17    // Pin D6 untuk kontrol H-Bridge (IN2)


void setup() {
  // Mengatur pin sebagai output
  pinMode(MOTOR_PIN1, OUTPUT);
  pinMode(MOTOR_PIN2, OUTPUT);
  pinMode(MOTOR_PIN3, OUTPUT);
  pinMode(MOTOR_PIN4, OUTPUT);
  
  // Memastikan motor berhenti saat startup
  digitalWrite(MOTOR_PIN1, 0); // 5
  digitalWrite(MOTOR_PIN2, 1); // 6
  digitalWrite(MOTOR_PIN3, 0); // 16
  digitalWrite(MOTOR_PIN4, 1); // 17 
}

void loop() {
}
