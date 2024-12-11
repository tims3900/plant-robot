#include <Arduino.h>
#include <HttpClient.h>
#include <Adafruit_BusIO_Register.h>
#include <WiFi.h>
#include <inttypes.h>
#include <stdio.h>
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"

#include <Adafruit_Sensor.h>
#include <Adafruit_AHTX0.h>

char ssid[50];
char pass[50];
const int kNetworkTimeout = 30 * 1000;
const int kNetworkDelay = 1000;

const int NUM_SENSORS = 3;
const int light1 = 37;
const int light2 = 38;
const int light3 = 39; 
const float PHOTORESISTOR_ANGLES[] = {PI / 3, PI, 5 * PI / 3};

const int wheel11 = 32; // Wheel 1 forward
const int wheel12 = 33; // Wheel 1 backward
const int wheel21 = 13; // Wheel 2 forward
const int wheel22 = 12; // Wheel 2 backward
const int wheel31 = 25; // Wheel 3 forward
const int wheel32 = 26; // Wheel 3 backward

const int pwmChannel11 = 0;
const int pwmChannel12 = 1;
const int pwmChannel21 = 2;
const int pwmChannel22 = 3;
const int pwmChannel31 = 4;
const int pwmChannel32 = 5;

unsigned long previousMillis = 0;
const long interval = 1000; 

const float WHEEL_ANGLE_OFFSET[] = {0, 2 * PI / 3, 4 * PI / 3};

bool moveRobot = false;

float findBrightestLightAngle() {
  int lightValues[NUM_SENSORS] = {
    analogRead(light1),
    analogRead(light2),
    analogRead(light3)
  };

  const int lightThreshold = 300; 
  int maxIndex1 = -1, maxIndex2 = -1;
  int maxValue = -1, minValue = INT_MAX;

  for (int i = 0; i < NUM_SENSORS; i++) {
    int value = lightValues[i];
    if (value > 3000) { 
      continue;
    }
    if (value > maxValue) {
      maxValue = value;
    }
    if (value < minValue) {
      minValue = value;
    }

    if (maxIndex1 == -1 || value > lightValues[maxIndex1]) {
      maxIndex2 = maxIndex1;
      maxIndex1 = i;
    } else if (maxIndex2 == -1 || value > lightValues[maxIndex2]) {
      maxIndex2 = i;
    }
  }

  if (maxValue - minValue < lightThreshold) {
    return -1;
  }

  if (maxIndex2 == -1) {
    float angle = PHOTORESISTOR_ANGLES[maxIndex1];
  }

  float weight1 = (float)lightValues[maxIndex1];
  float weight2 = (float)lightValues[maxIndex2];

  float angle1 = PHOTORESISTOR_ANGLES[maxIndex1];
  float angle2 = PHOTORESISTOR_ANGLES[maxIndex2];

  float x1 = weight1 * cos(angle1);
  float y1 = weight1 * sin(angle1);
  float x2 = weight2 * cos(angle2);
  float y2 = weight2 * sin(angle2);

  float avgX = x1 + x2;
  float avgY = y1 + y2;
  float brightestAngle = atan2(avgY, avgX);
  
  return brightestAngle < 0 ? brightestAngle + 2 * PI : brightestAngle;
}

void vectorToWheelSpeeds(float vx, float vy, float* wheelSpeeds) {
  for (int i = 0; i < 3; i++) {
    float angle = WHEEL_ANGLE_OFFSET[i];
    wheelSpeeds[i] = -sin(angle) * vx + cos(angle) * vy;
  }
}

void setWheelSpeed(int pinForward, int pinBackward, float speed, int forwardChannel, int backwardChannel) {
  speed = constrain(speed, -1.0, 1.0);
  int pwmValue = abs(speed * 2000);

  if (speed > 0) {
    ledcWrite(forwardChannel, pwmValue);
    ledcWrite(backwardChannel, 0);
  } else if (speed < 0) {
    ledcWrite(backwardChannel, pwmValue);
    ledcWrite(forwardChannel, 0);
  } else {
    ledcWrite(forwardChannel, 0);
    ledcWrite(backwardChannel, 0);
  }
}

void stopWheels() {
  setWheelSpeed(wheel11, wheel12, 0, pwmChannel11, pwmChannel12);
  setWheelSpeed(wheel21, wheel22, 0, pwmChannel21, pwmChannel22);
  setWheelSpeed(wheel31, wheel32, 0, pwmChannel31, pwmChannel32);
}

void moveRobotTowardsAngle(float angle) {
  float vx = cos(angle);
  float vy = sin(angle);

  float wheelSpeeds[3];
  vectorToWheelSpeeds(vx, vy, wheelSpeeds);

  setWheelSpeed(wheel11, wheel12, wheelSpeeds[0], pwmChannel11, pwmChannel12);
  setWheelSpeed(wheel21, wheel22, wheelSpeeds[1], pwmChannel21, pwmChannel22);
  setWheelSpeed(wheel31, wheel32, wheelSpeeds[2], pwmChannel31, pwmChannel32);
}


void nvs_access() {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    Serial.printf("\n");
    Serial.printf("Opening Non-Volatile Storage (NVS) handle... ");
    nvs_handle_t my_handle;
    err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        Serial.printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    } else {
        Serial.printf("Done\n");
        Serial.printf("Retrieving SSID/PASSWD\n");
        size_t ssid_len;
        size_t pass_len;
        err = nvs_get_str(my_handle, "ssid", ssid, &ssid_len);
        err |= nvs_get_str(my_handle, "pass", pass, &pass_len);
        switch (err) {
            case ESP_OK:
                Serial.printf("Done\n");
                Serial.printf("SSID = %s\n", ssid);
                Serial.printf("PASSWD = %s\n", pass);
                break;
            case ESP_ERR_NVS_NOT_FOUND:
                Serial.printf("The value is not initialized yet!\n");
                break;
            default:
                Serial.printf("Error (%s) reading!\n", esp_err_to_name(err));
        }
    }
    nvs_close(my_handle);
}

void setup() {
  Serial.begin(9600);
  delay(1000);

  nvs_access();

  delay(1000);
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, pass);

  while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println("MAC address: ");
  Serial.println(WiFi.macAddress());

  Serial.println("Light-Tracking Robot Ready!");

  // Set up PWM for wheels
  ledcSetup(pwmChannel11, 5000, 8);
  ledcSetup(pwmChannel12, 5000, 8);
  ledcSetup(pwmChannel21, 5000, 8);
  ledcSetup(pwmChannel22, 5000, 8);
  ledcSetup(pwmChannel31, 5000, 8);
  ledcSetup(pwmChannel32, 5000, 8);

  ledcAttachPin(wheel11, pwmChannel11);
  ledcAttachPin(wheel12, pwmChannel12);
  ledcAttachPin(wheel21, pwmChannel21);
  ledcAttachPin(wheel22, pwmChannel22);
  ledcAttachPin(wheel31, pwmChannel31);
  ledcAttachPin(wheel32, pwmChannel32);
}

// Loop Function
void loop() {
  int err = 0;
  WiFiClient c;
  HttpClient http(c);

  int lightValues[NUM_SENSORS] = {
     analogRead(light1),
     analogRead(light2),
     analogRead(light3)
  };

  // Construct query string with light sensor values
  String values = "/?light1=" + String(lightValues[0]) + 
                  "&light2=" + String(lightValues[1]) + 
                 "&light3=" + String(lightValues[2]);
  const char* query = values.c_str();
  err = http.get("54.193.149.150", 5000, query, NULL);
    if (err == 0) {
        Serial.println("startedRequest ok");
        err = http.responseStatusCode();
        if (err >= 0) {
            Serial.print("Got status code: ");
            Serial.println(err);
            err = http.skipResponseHeaders();
            if (err >= 0) {
                int bodyLen = http.contentLength();
                Serial.print("sent");
            } else {
                Serial.print("Failed to skip response headers: ");
                Serial.println(err);
            }
        } else {
            Serial.print("Getting response failed: ");
            Serial.println(err);
        }
    } else {
        Serial.print("Connect failed: ");
        Serial.println(err);
    }
    http.stop();
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    float brightestAngle = findBrightestLightAngle();
    
    if (brightestAngle == -1) {
      Serial.println("Light levels are too uniform. Robot will not move.");
      stopWheels();
      return;
    }
    
    Serial.print("Brightest Angle: ");
    Serial.println(brightestAngle * 180 / PI);

    moveRobotTowardsAngle(brightestAngle);
    delay(250); 
    stopWheels();
  }

  Serial.print("Light Values: ");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(lightValues[i]);
    if (i < NUM_SENSORS - 1) Serial.print(" | ");
  }
  Serial.println();
  delay(50);
}
