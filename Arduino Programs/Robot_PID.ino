#define MOTOR_PIN1 11    // Motor A IN1 (Left)
#define MOTOR_PIN2 10    // Motor A IN2 (Left)
#define MOTOR_PIN3 6     // Motor B IN1 (Right)
#define MOTOR_PIN4 5     // Motor B IN2 (Right)

const int selectPins[] = {2, 3, 4}; 
const int analogPin = A0;           
const int threshold = 280;          
const int SPEED_BASE = 180;         

const float Kp = 1.0;               
const float Ki = 0.0001;               
const float Kd = 0.001;              

float previousError = 0;            
float integral = 0;                 

void setup() {
  Serial.begin(9600);
  pinMode(MOTOR_PIN1, OUTPUT); pinMode(MOTOR_PIN2, OUTPUT);
  pinMode(MOTOR_PIN3, OUTPUT); pinMode(MOTOR_PIN4, OUTPUT);
  for (int i = 0; i < 3; i++) pinMode(selectPins[i], OUTPUT);
}

void loop() {
  int sensorValues[8], weightedSum = 0, sum = 0;
  bool anySensorDetectsLine = false, allSensorsDetectLine = true;

  for (int i = 0; i < 8; i++) {
    selectChannel(i);
    int val = analogRead(analogPin);
    sensorValues[i] = val;
    if (val < threshold) anySensorDetectsLine = true;
    else allSensorsDetectLine = false;

    int weight = i - 3; 
    int sensorActive = (val < threshold) ? 1000 : 0;
    weightedSum += sensorActive * weight;
    sum += sensorActive;
  }

  for (int i = 0; i < 8; i++) { Serial.print(sensorValues[i]); Serial.print("\t"); }

  if (allSensorsDetectLine) {
    stopMotors(); Serial.println("stop");
  }
  else if (anySensorDetectsLine && sum != 0) {
    float error = (float)weightedSum / sum;
    integral += error;
    float derivative = error - previousError;
    float output = Kp * error + Ki * integral + Kd * derivative;
    previousError = error;

    int adjustment = output * 40;
    int leftSpeed = constrain(SPEED_BASE + adjustment, 0, 255);
    int rightSpeed = constrain(SPEED_BASE - adjustment, 0, 255);

    setMotorSpeeds(leftSpeed, rightSpeed);
    Serial.print("PID: "); Serial.println(output);
  }
  else {
    delay(50); stopMotors(); Serial.println("stop");
  }

  // delay(10);
}

void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  analogWrite(MOTOR_PIN1, leftSpeed); analogWrite(MOTOR_PIN2, 0);
  analogWrite(MOTOR_PIN3, rightSpeed); analogWrite(MOTOR_PIN4, 0);
}

void stopMotors() {
  analogWrite(MOTOR_PIN1, 0); analogWrite(MOTOR_PIN2, 0);
  analogWrite(MOTOR_PIN3, 0); analogWrite(MOTOR_PIN4, 0);
}

void selectChannel(int channel) {
  for (int i = 0; i < 3; i++) digitalWrite(selectPins[i], (channel >> i) & 1);
}
