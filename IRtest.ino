const int sensorPin = A0; // Change this to A1–A7 for others
int sensorValue = 0;
const int threshold = 8; // Between 6 and 10

void setup() {
  Serial.begin(9600);
}

void loop() {
  sensorValue = analogRead(sensorPin);

  Serial.print("Sensor Value: ");
  Serial.print(sensorValue);

  if (sensorValue <= threshold) {
    Serial.println(" --> Obstacle detected!");
  } else {
    Serial.println(" --> Clear");
  }

  delay(100);
}
