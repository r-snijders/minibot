// Arduino Uno/Nano + TB6612FNG; both encoder A pins use hardware interrupts.
#include <Arduino.h>
#include <math.h>
#include <stdlib.h>

constexpr int LA = 2, LB = 4, RA = 3, RB = 5;
constexpr int LPWM = 9, LIN1 = 7, LIN2 = 8;
constexpr int RPWM = 10, RIN1 = 11, RIN2 = 12, STBY = 6;
constexpr float TICKS_PER_REV = 360.0f;  // calibrate wheel-side counts
constexpr float KP = 8.0f;  // PWM units per rad/s speed error; tune
volatile long leftTicks = 0, rightTicks = 0;
float targetL = 0, targetR = 0, speedL = 0, speedR = 0;
float pwmL = 0, pwmR = 0;
unsigned long lastCommand = 0, lastControl = 0;
long prevL = 0, prevR = 0;
char line[64];
byte lineLength = 0;

void leftEdge() { leftTicks += (digitalRead(LA) == digitalRead(LB)) ? 1 : -1; }
void rightEdge() { rightTicks += (digitalRead(RA) == digitalRead(RB)) ? 1 : -1; }

void motor(int pwmPin, int in1, int in2, float value) {
  int magnitude = constrain((int)fabsf(value), 0, 255);
  digitalWrite(in1, value > 0 ? HIGH : LOW);
  digitalWrite(in2, value < 0 ? HIGH : LOW);
  analogWrite(pwmPin, magnitude);
}

void setup() {
  Serial.begin(115200);
  pinMode(LA, INPUT_PULLUP); pinMode(LB, INPUT_PULLUP);
  pinMode(RA, INPUT_PULLUP); pinMode(RB, INPUT_PULLUP);
  pinMode(LPWM, OUTPUT); pinMode(LIN1, OUTPUT); pinMode(LIN2, OUTPUT);
  pinMode(RPWM, OUTPUT); pinMode(RIN1, OUTPUT); pinMode(RIN2, OUTPUT);
  pinMode(STBY, OUTPUT); digitalWrite(STBY, HIGH);
  attachInterrupt(digitalPinToInterrupt(LA), leftEdge, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RA), rightEdge, CHANGE);
  lastControl = lastCommand = millis();
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      line[lineLength] = '\0';
      char *p = line;
      bool valid = (*p++ == 'V' && *p++ == ' ');
      char *endL = p;
      float l = strtod(p, &endL);
      char *endR = endL;
      float r = strtod(endL, &endR);
      if (valid && endL != p && endR != endL && *endR == '\0' && isfinite(l) && isfinite(r)) {
        targetL = constrain(l, -12.0f, 12.0f);
        targetR = constrain(r, -12.0f, 12.0f);
        lastCommand = millis();
      }
      lineLength = 0;
    } else if (lineLength < sizeof(line) - 1) line[lineLength++] = c;
    else lineLength = 0;
  }
  unsigned long now = millis();
  if (now - lastControl < 50) return;
  float dt = (now - lastControl) / 1000.0f;
  lastControl = now;
  noInterrupts(); long l = leftTicks, r = rightTicks; interrupts();
  speedL = (l - prevL) * (2.0f * PI / TICKS_PER_REV) / dt;
  speedR = (r - prevR) * (2.0f * PI / TICKS_PER_REV) / dt;
  prevL = l; prevR = r;
  if (now - lastCommand > 500) { targetL = 0; targetR = 0; }
  if (targetL == 0) pwmL = 0;
  else pwmL = constrain(pwmL + KP * (targetL - speedL), -255.0f, 255.0f);
  if (targetR == 0) pwmR = 0;
  else pwmR = constrain(pwmR + KP * (targetR - speedR), -255.0f, 255.0f);
  motor(LPWM, LIN1, LIN2, pwmL);
  motor(RPWM, RIN1, RIN2, pwmR);
  Serial.print("T "); Serial.print(l); Serial.print(' '); Serial.println(r);
}
