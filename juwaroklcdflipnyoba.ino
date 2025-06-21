#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include "time.h"
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <ESP32Servo.h>
#include <RTClib.h>

#define BLYNK_TEMPLATE_ID "TMPL6MpcgorMy"
#define BLYNK_TEMPLATE_NAME "tugas akhir"
#define BLYNK_AUTH_TOKEN "hc-Z09Nfq1AuoYO_CZk8z6nfOK9jtdSg"

char ssid[] = "Andromax-M2Y-F6BF";
char pass[] = "27385345";

// Pin
#define DHTPIN 4
#define SOIL_MOISTURE_PIN 34
#define PUMP_PIN 5
#define TDS_PIN 33
#define servoPin 12
#define PH_SENSOR_PIN 32 
#define VPIN_PUMP_STATUS V6

// Waktu
const char* ntpServer = "time.google.com";
const long gmtOffset_sec = 7 * 3600;
const int daylightOffset_sec = 0;

RTC_DS3231 rtc;
Servo myServo;
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);
LiquidCrystal_I2C lcd(0x27, 20, 4);

// pH Sensor
unsigned long int avgval;
int buffer_arr[20];
float voltage, ph_act;

const int soilDryValue = 3200;
const int soilWetValue = 1500;

float VREF = 3.3;
float adcResolution = 4095.0;
float kFactor = 1;

// Nutrisi & halaman
bool nutrisi1Diberikan = false;
bool nutrisi2Diberikan = false;
bool showNutrisiPage = false;
unsigned long lastPageSwitch = 0;
const unsigned long flipInterval = 5000;
unsigned long previousMillis = 0;
const unsigned long interval = 120000;

void updateNutrisiPage() {
  if (millis() - lastPageSwitch >= flipInterval) {
    showNutrisiPage = !showNutrisiPage;
    lastPageSwitch = millis();

    lcd.clear();
    if (showNutrisiPage) {
      lcd.setCursor(0, 0);
      lcd.print("Nutrisi 1: ");
      lcd.print(nutrisi1Diberikan ? "Sudah" : "Belum");
      lcd.setCursor(0, 1);
      lcd.print("Nutrisi 2: ");
      lcd.print(nutrisi2Diberikan ? "Sudah" : "Belum");
    } else {
      lcd.setCursor(0, 0);
      lcd.print("Suhu: ");
      lcd.print(dht.readTemperature());
      lcd.print(" C");

      lcd.setCursor(0, 1);
      int soilADC = analogRead(SOIL_MOISTURE_PIN);
      int soilMoisturePercent = map(soilADC, soilDryValue, soilWetValue, 0, 100);
      soilMoisturePercent = constrain(soilMoisturePercent, 0, 100);
      lcd.print("Soil: ");
      lcd.print(soilMoisturePercent);
      lcd.print("%");

      lcd.setCursor(0, 2);
      int analogValue = analogRead(TDS_PIN);
      float tdsValue = (analogValue / adcResolution) * VREF * 100;
      lcd.print("TDS: ");
      lcd.print(tdsValue);
      lcd.print("ppm");

      lcd.setCursor(0, 3);
      lcd.print("pH: ");
      lcd.print(ph_act);
    }
  }
}

void setup() {
  Serial.begin(115200);
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  dht.begin();
  lcd.init();
  lcd.backlight();

  pinMode(PUMP_PIN, OUTPUT);
  digitalWrite(PUMP_PIN, LOW);

  myServo.attach(servoPin);
  myServo.write(0);

  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  struct tm timeinfo;
  int retries = 10;
  while (!getLocalTime(&timeinfo) && retries-- > 0) {
    Serial.println("Menunggu waktu NTP...");
    delay(1000);
  }

  if (!rtc.begin()) {
    Serial.println("RTC tidak terdeteksi!");
    while (1);
  }

  rtc.adjust(DateTime(
    timeinfo.tm_year + 1900,
    timeinfo.tm_mon + 1,
    timeinfo.tm_mday,
    timeinfo.tm_hour,
    timeinfo.tm_min,
    timeinfo.tm_sec
  ));

  lcd.setCursor(0, 0);
  lcd.print("SmartFarm System");
  lcd.setCursor(0, 1);
  lcd.print("Initializing...");
  delay(2000);
}

void loop() {
  DateTime now = rtc.now();

  // Aktifkan servo untuk nutrisi
  if (now.hour() == 6 && now.minute() == 30 && now.second() == 0) {
    myServo.write(180);
    delay(1000);
    myServo.write(0);
    nutrisi1Diberikan = true;
  }

  if (now.hour() == 10 && now.minute() == 30 && now.second() == 0) {
    myServo.write(180);
    delay(1000);
    myServo.write(0);
    nutrisi2Diberikan = true;
  }

  Blynk.run();

  int soilADC = analogRead(SOIL_MOISTURE_PIN);
  int soilMoisturePercent = map(soilADC, soilDryValue, soilWetValue, 0, 100);
  soilMoisturePercent = constrain(soilMoisturePercent, 0, 100);

  String soilStatus = (soilMoisturePercent <= 30) ? "Kering" :
                      (soilMoisturePercent <= 60) ? "Lembab" : "Basah";

  for (int i = 0; i < 10; i++) {
    buffer_arr[i] = analogRead(PH_SENSOR_PIN);
    delay(30);
  }
  avgval = 0;
  for (int i = 0; i < 10; i++) avgval += buffer_arr[i];
  avgval /= 10;
  voltage = avgval * (3.3 / 4095.0);
  ph_act = -6.0 * voltage + 22.0;

  int analogValue = analogRead(TDS_PIN);
  float voltageTDS = (analogValue / adcResolution) * VREF;
  float tdsValue = (voltageTDS * kFactor) * 100;

  digitalWrite(PUMP_PIN, soilMoisturePercent <= 45 ? HIGH :
                          soilMoisturePercent >= 60 ? LOW : digitalRead(PUMP_PIN));

  Blynk.virtualWrite(VPIN_PUMP_STATUS, digitalRead(PUMP_PIN) == HIGH ? "Nyala" : "Mati");

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    Blynk.virtualWrite(V0, dht.readTemperature());
    Blynk.virtualWrite(V2, soilStatus);
    Blynk.virtualWrite(V3, ph_act);
    Blynk.virtualWrite(V4, tdsValue);
    Serial.println("Data berhasil dikirim ke Blynk.");
  }

  updateNutrisiPage();
  delay(2000);
}
