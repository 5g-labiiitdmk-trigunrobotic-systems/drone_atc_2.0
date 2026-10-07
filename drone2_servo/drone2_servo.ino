#include <ESP32Servo.h>


const int triggerPin = 34;
const int servoPin   = 18;

Servo myServo;

const int restAngle    = 0;
const int triggerAngle = 90;
const int triggerThreshold = 1500;

volatile unsigned long pulseStart = 0;
volatile unsigned long pulseWidth = 0;
volatile bool newPulse = false;

void IRAM_ATTR handleInterrupt() {
  if (digitalRead(triggerPin) == HIGH) {
    pulseStart = micros();
  } else {
    unsigned long width = micros() - pulseStart;
    if (width >= 400 && width <= 2600) {
      pulseWidth = width;
      newPulse = true;
    }
  }
}

bool stableState = false;
int consistentCount = 0;
const int requiredConsistent = 5; // require 5 matching readings before acting

void setup() {
  Serial.begin(115200);

  pinMode(triggerPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(triggerPin), handleInterrupt, CHANGE);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  myServo.setPeriodHertz(50);
  myServo.attach(servoPin, 500, 2500);
  delay(500); // let servo settle before first command
  myServo.write(restAngle);

  Serial.println("Ready.");
}

void loop() {
  if (newPulse) {
    newPulse = false;
    bool reading = pulseWidth > (unsigned long)triggerThreshold;

    if (reading == stableState) {
      consistentCount = 0; // no change, reset counter
    } else {
      consistentCount++;
      if (consistentCount >= requiredConsistent) {
        stableState = reading;
        consistentCount = 0;

        if (stableState) {
          Serial.println("CONFIRMED trigger -> 90");
          myServo.write(triggerAngle);
        } else {
          Serial.println("CONFIRMED release -> rest");
          myServo.write(restAngle);
        }
      }
    }
  }
}