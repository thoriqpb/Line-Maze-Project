const int selectPins[] = {4, 5, 6}; // S1, S2, S3
const int analogPin = A0;

void setup() {
  Serial.begin(9600);
  for (int i = 0; i < 3; i++) {
    pinMode(selectPins[i], OUTPUT);
  }
}

void loop() {
  for (int channel = 0; channel < 8; channel++) {
    selectChannel(channel);
    int value = analogRead(analogPin);
    Serial.print(value);
    Serial.print("\t"); 
  }
  Serial.println(); 
  // delay(50);
}

void selectChannel(int channel) {
  for (int i = 0; i < 3; i++) {
    digitalWrite(selectPins[i], (channel >> i) & 1);
  }
}
