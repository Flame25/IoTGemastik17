#include "esp32-hal-gpio.h"
#include "esp32-hal-ledc.h"
#include <WiFi.h>
#include <Arduino.h>
#include <Firebase_ESP_Client.h>
#include <LiquidCrystal_I2C.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>

#define WIFI_SSID "SSID"
#define WIFI_PASSWORD "PASS"
#define API_KEY "API_KEY"
#define DATABASE_URL "DATABASE_URL"
#define AOUT_PIN 36 // ESP32 pin GPIO36 (ADC0) that connects to AOUT pin of moisture sensor

LiquidCrystal_I2C lcd(0x27, 16, 2);
TinyGPSPlus gps;
HardwareSerial GPS_Serial(1);

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

// Global Variable
bool flagx = true;

void setup() {
  Serial.begin(115200);
  lcd.init();
  lcd.backlight();
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to ");
  Serial.print(WIFI_SSID);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  
  Serial.println();
  Serial.println("Connected");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;

  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);

  // Initialize GPS serial
  GPS_Serial.begin(9600, SERIAL_8N1, 16, 17); // RX, TX
}

void loop() {
  delay(2000);
  int humidity = analogRead(AOUT_PIN); // Read the analog value from soil moisture sensor

  Serial.print("Moisture value: ");
  Serial.println(value);

  // Read from GPS module
  while (GPS_Serial.available() > 0) {
    gps.encode(GPS_Serial.read());
  }

  delay(2000);

  float latitude = gps.location.lat();
  float longitude = gps.location.lng();

  flagx = !flagx;

  if(flagx){
  lcd.clear(); 
  lcd.setCursor(0, 0); 
  lcd.print("Hum: ");
  lcd.print(humidity);
  lcd.print("%"); // Will be add for natrium, calcium, and phospor as soon as the sensor available
}else{
  lcd.setCursor(0, 0); 
  lcd.print("Lat: ");
  lcd.print(latitude, 6);
  lcd.setCursor(0, 1); 
  lcd.print("Lng: ");
  lcd.print(longitude, 6);
}
  Serial.println("Sending data to Firebase...");

  if (Firebase.RTDB.setFloat(&fbdo, "/sensor/humidity", humidity)) {
    Serial.println("Humidity data sent to Firebase.");
  } else {
    Serial.print("Failed to send humidity data to Firebase: ");
    Serial.println(fbdo.errorReason());
  }

  if (Firebase.RTDB.setFloat(&fbdo, "/sensor/latitude", latitude)) {
    Serial.println("Latitude data sent to Firebase.");
  } else {
    Serial.print("Failed to send latitude data to Firebase: ");
    Serial.println(fbdo.errorReason());
  }

  if (Firebase.RTDB.setFloat(&fbdo, "/sensor/longitude", longitude)) {
    Serial.println("Longitude data sent to Firebase.");
  } else {
    Serial.print("Failed to send longitude data to Firebase: ");
    Serial.println(fbdo.errorReason());
  }

  // Need Id Checker for device  
}

