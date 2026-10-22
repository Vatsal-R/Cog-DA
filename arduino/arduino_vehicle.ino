// Gesture-Controlled Vehicle: Arduino + L298N motor driver.
// Update pin values if your wiring is different.
const int ENA = 5;
const int IN1 = 8;
const int IN2 = 9;
const int ENB = 6;
const int IN3 = 10;
const int IN4 = 11;
const unsigned long FAILSAFE_MS = 700;
unsigned long lastCommandMs = 0;

void stopMotors() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}
void forward() { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); }
void backward() { digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); }
void left() { digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); }
void right() { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); }

void setup() {
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  analogWrite(ENA, 180); analogWrite(ENB, 180); // speed: 0 to 255
  stopMotors();
  Serial.begin(9600);
  lastCommandMs = millis();
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'F') forward();
    else if (c == 'B') backward();
    else if (c == 'L') left();
    else if (c == 'R') right();
    else if (c == 'S') stopMotors();
    else return;
    lastCommandMs = millis();
  }
  if (millis() - lastCommandMs > FAILSAFE_MS) stopMotors();
}
