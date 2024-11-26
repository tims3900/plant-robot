#include <Arduino.h>
#include <Servo.h>

const int sensorPin = 2;
const int servoPin = 21;

int currLight;
int maxLight = 0;
int maxAngle = 0;

const int scanStep = 5; 
const int delayTime = 100;

Servo servo;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  servo.attach(servoPin);
  servo.write(0);
  delay(1000);
}

void loop() {
  maxLight = 0;
  maxAngle = 0;

  for (int angle = 0; angle <= 180; angle += scanStep) {
    servo.write(angle);
    delay(200);
    currLight = analogRead(sensorPin);
    
    Serial.print("Angle: ");
    Serial.print(angle);
    Serial.print(" - Light: ");
    Serial.println(currLight);

    if (currLight > maxLight) {
      maxLight = currLight;
      maxAngle = angle;
    }
  }

  servo.write(maxAngle);
  Serial.print("Max Light: ");
  Serial.print(maxLight);
  Serial.print(" at Angle: ");
  Serial.println(maxAngle);

  delay(10000);
}
