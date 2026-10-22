/* Gesture-Controlled Vehicle: ESP32 + L298N + HC-SR04
   PC packets: F,180 / B,180 / L,180 / R,180 / S,0
   Upload this sketch to the ESP32. Change pins to match your wiring.
*/
#include <Arduino.h>

const int ENA = 25, IN1 = 26, IN2 = 27;
const int ENB = 33, IN3 = 14, IN4 = 12;
const int TRIG_PIN = 4, ECHO_PIN = 34;
const int PWM_FREQ = 20000, PWM_RESOLUTION = 8, PWM_A = 0, PWM_B = 1;
const int STOP_DISTANCE_CM = 20;
const unsigned long COMMAND_TIMEOUT_MS = 200;
const int RAMP_STEP = 8;

volatile char requestedDirection = 'S';
volatile int requestedSpeed = 0;
volatile bool safetyLockout = false;
volatile unsigned long lastCommandMs = 0;
int appliedSpeed = 0;
char appliedDirection = 'S';
String packet;

void setDirection(char direction) {
  bool forwardLeft = direction == 'F' || direction == 'R';
  bool forwardRight = direction == 'F' || direction == 'L';
  digitalWrite(IN1, forwardLeft);  digitalWrite(IN2, !forwardLeft);
  digitalWrite(IN3, forwardRight); digitalWrite(IN4, !forwardRight);
}

void applyMotor(char direction, int speed) {
  if (direction == 'S' || speed == 0) {
    digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
    ledcWrite(PWM_A, 0); ledcWrite(PWM_B, 0);
    return;
  }
  setDirection(direction);
  ledcWrite(PWM_A, speed); ledcWrite(PWM_B, speed);
}

long distanceCm() {
  digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 25000);
  return duration == 0 ? -1 : duration / 58;
}

void safetyTask(void*) {
  for (;;) {
    long distance = distanceCm();
    // Invalid readings do not create a false lockout; valid close objects do.
    safetyLockout = distance > 0 && distance <= STOP_DISTANCE_CM;
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void motorTask(void*) {
  for (;;) {
    bool timedOut = millis() - lastCommandMs > COMMAND_TIMEOUT_MS;
    char targetDirection = (safetyLockout || timedOut) ? 'S' : requestedDirection;
    int targetSpeed = targetDirection == 'S' ? 0 : requestedSpeed;
    if (appliedSpeed < targetSpeed) appliedSpeed = min(appliedSpeed + RAMP_STEP, targetSpeed);
    else if (appliedSpeed > targetSpeed) appliedSpeed = max(appliedSpeed - RAMP_STEP, targetSpeed);
    appliedDirection = targetDirection;
    applyMotor(appliedDirection, appliedSpeed);
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void parsePacket(String value) {
  value.trim();
  if (value.length() < 1) return;
  char direction = value.charAt(0);
  if (String("FBLRS").indexOf(direction) < 0) return;
  int comma = value.indexOf(',');
  int speed = comma >= 0 ? value.substring(comma + 1).toInt() : 180;
  requestedDirection = direction;
  requestedSpeed = direction == 'S' ? 0 : constrain(speed, 0, 255);
  lastCommandMs = millis();
}

void setup() {
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT); pinMode(ECHO_PIN, INPUT);
  ledcSetup(PWM_A, PWM_FREQ, PWM_RESOLUTION); ledcAttachPin(ENA, PWM_A);
  ledcSetup(PWM_B, PWM_FREQ, PWM_RESOLUTION); ledcAttachPin(ENB, PWM_B);
  applyMotor('S', 0);
  Serial.begin(115200);
  lastCommandMs = millis();
  xTaskCreatePinnedToCore(safetyTask, "ultrasonic_safety", 3072, nullptr, 2, nullptr, 0);
  xTaskCreatePinnedToCore(motorTask, "motor_pwm", 3072, nullptr, 2, nullptr, 1);
}

void loop() {
  while (Serial.available()) {
    char character = (char)Serial.read();
    if (character == '\n') { parsePacket(packet); packet = ""; }
    else if (character != '\r' && packet.length() < 20) packet += character;
  }
  delay(1);
}