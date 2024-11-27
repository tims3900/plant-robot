#include <Arduino.h>
#include <Servo.h>
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

char ssid[50]; // your network SSID (name)
char pass[50]; // your network password (use for WPA, or use
const int kNetworkTimeout = 30 * 1000;
const int kNetworkDelay = 1000;

const int sensorPin = 2;
const int servoPin = 21;

int currLight;
int maxLight = 0;
int maxAngle = 0;

const int scanStep = 5; 
const int delayTime = 100;

Servo servo;


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
  // put your setup code here, to run once:
  Serial.begin(9600);
  servo.attach(servoPin);
  servo.write(0);
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

}

void loop() {
  maxLight = 0;
  maxAngle = 0;
  int err = 0;

  WiFiClient c;
  HttpClient http(c);

  String values = "/?light=" + String(maxLight, 2);
  const char* query = values.c_str();

  err = http.get("54.193.149.150", 5000, query, NULL);
    if (err == 0) {
        Serial.println("startedRequest ok");
        err = http.responseStatusCode();
        if (err >= 0) {
            Serial.print("Got status code: ");
            Serial.println(err);

            // Usually you'd check that the response code is 200 or a
            // similar "success" code (200-299) before carrying on,
            // but we'll print out whatever response we get
            err = http.skipResponseHeaders();
            if (err >= 0) {
                int bodyLen = http.contentLength();
                Serial.print("Content length is: ");
                Serial.println(bodyLen);
                Serial.println();
                Serial.println("Body returned follows:");

                // Now we've got to the body, so we can print it out
                unsigned long timeoutStart = millis();
                char c;
                // Whilst we haven't timed out & haven't reached the end of the body
                while ((http.connected() || http.available()) && ((millis() - timeoutStart) < kNetworkTimeout)) {
                    if (http.available()) {
                        c = http.read();
                        // Print out this character
                        Serial.print(c);
                        bodyLen--;
                        // We read something, reset the timeout counter
                        timeoutStart = millis();
                    } else {
                        // We haven't got any data, so let's pause to allow some to
                        // arrive
                        delay(kNetworkDelay);
                    }
                }
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

    // And just stop, now that we've tried a download
    //while (1);
    delay(2000);

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
