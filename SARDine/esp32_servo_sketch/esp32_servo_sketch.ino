#include <ESP32Servo.h>

Servo myservo;
const int servoSwitchPin = 25;   
const int ledSwitchPin = 26;
const int servoPin = 33; // Output for servo control
const int ledPin = 32;    // Output to LED

void setup() {
  Serial.begin(115200);

  // Configure Inputs
  pinMode(servoSwitchPin, INPUT);
  pinMode(ledSwitchPin, INPUT);
  
  // Configure Output
  pinMode(ledPin, OUTPUT);

  // Servo Setup
  ESP32PWM::allocateTimer(0);
  myservo.setPeriodHertz(50);
  myservo.attach(servoPin, 500, 2400); 
}

void loop() {
  // 1. Continuous Servo Logic
  if (digitalRead(servoSwitchPin) == HIGH) {
    myservo.write(0);
  } else {
    myservo.write(90);
  }

  // 2. LED Logic 
  if (digitalRead(ledSwitchPin) == HIGH) {
    digitalWrite(ledPin, HIGH); // Turn LED ON
  } else {
    digitalWrite(ledPin, LOW);  // Turn LED OFF
  }

  // Small delay for stability
  delay(20); 
}