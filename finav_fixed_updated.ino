#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <EEPROM.h>
#include <GravityTDS.h>

#define BLYNK_TEMPLATE_ID "TMPL6MpcgorMy"
#define BLYNK_TEMPLATE_NAME "tugas akhir"
#define BLYNK_AUTH_TOKEN "hc-Z09Nfq1AuoYO_CZk8z6nfOK9jtdSg"

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <ESP32Servo.h>

#include <EEPROM.h>
#include <GravityTDS.h>

#define TdsSensorPin 33
GravityTDS gravityTds;

float temperature = 25, tdsValue = 0;

// WiFi 
char ssid[] = "Andromax-M2Y-F6BF";
char pass[] = "27385345";

// Pin 
#define DHTPIN 4
#define PUMP_PIN 5
#define servoPin 12
#define TRIG_PIN 18
#define ECHO_PIN 19
#define PH_PIN 32
#define TdsSensorPin 33
#define SOIL_MOISTURE_PIN 34
#define RELAYSELENOID_PIN 35  
#define VPIN_PUMP_STATUS V6

// Servo
Servo myServo;

// DHT11 sensor
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// LCD I2C
LiquidCrystal_I2C lcd(0x27, 20, 4);

// pH calibration
float voltagePH;
float phValue;

// Tegangan referensi larutan pH standar
float PH_4_VOLTAGE = 3.30;  // sesuaikan dengan hasil pembacaan saat pH 4
float PH_7_VOLTAGE = 2.84;  // sesuaikan dengan hasil pembacaan saat pH 7

// Kalibrasi nilai ADC soil moisture 
const int soilDryValue = 3200;   // Nilai saat tanah kering (analogRead besar)
const int soilWetValue = 1500;   // Nilai saat tanah basah (analogRead kecil)

// Interval pengiriman Blynk
unsigned long previousMillis = 0;
const unsigned long interval = 120000; // 2 menit

void setup() {
  gravityTds.setPin(TdsSensorPin);
  gravityTds.setAref(3.3);       // Tegangan referensi ADC ESP32
  gravityTds.setAdcRange(4096);  // ADC ESP32 12-bit
  gravityTds.begin();

  Serial.begin(115200);

  WiFi.begin(ssid, pass);
  unsigned long wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < 10000) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi Connected.");
    Blynk.config(BLYNK_AUTH_TOKEN);
    Blynk.connect();
  } else {
    Serial.println("WiFi Not Connected. System will work offline.");
  }

  dht.begin();
  lcd.init();
  lcd.backlight();

  pinMode(PUMP_PIN, OUTPUT);
  digitalWrite(PUMP_PIN, LOW);

  myServo.attach(servoPin);

  lcd.setCursor(0, 0);
  lcd.print("SmartFarm System");
  lcd.setCursor(0, 1);
  lcd.print("Initializing...");

  delay(2000);
}

void loop() {
  gravityTds.setTemperature(temperature);  // Jika ada sensor suhu, ganti nilainya
  gravityTds.update();                     // Sampling dan kalkulasi
  tdsValue = gravityTds.getTdsValue();     // Ambil hasil
  Serial.print("TDS: ");
  Serial.print(tdsValue, 0);
  Serial.println(" ppm");

  // Jalankan Blynk hanya jika WiFi terkoneksi!
  if (WiFi.status() == WL_CONNECTED) {
    Blynk.run();
  }

  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();

  // Baca soilMoisture
  int soilADC = analogRead(SOIL_MOISTURE_PIN);
  int soilMoisturePercent = map(soilADC, soilDryValue, soilWetValue, 0, 100);
  soilMoisturePercent = constrain(soilMoisturePercent, 0, 100);

  String soilStatus;
  if (soilMoisturePercent <= 30) {
    soilStatus = "Kering";
  } else if (soilMoisturePercent <= 60) {
    soilStatus = "Lembab";
  } else {
    soilStatus = "Basah";
  }

  // Baca pH
  int phAnalog = analogRead(PH_PIN);
  voltagePH = (phAnalog / 4095.0) * 3.3;

  // Kalibrasi linier 2 titik
  float slope = (7.00 - 4.00) / (PH_7_VOLTAGE - PH_4_VOLTAGE);
  float intercept = 7.00 - (slope * PH_7_VOLTAGE);
  phValue = slope * voltagePH + intercept;

  //kalibrasi tds
  int analogValue = analogRead(TDS_PIN); // Baca nilai analog dari sensor
  float voltage = (analogValue / adcResolution) * VREF; // Konversi nilai analog menjadi tegangan
  float tdsValue = (voltage * kFactor) * 100; // Hitung TDS dalam ppm

  // Kontrol pompa
  if (soilMoisturePercent <= 45) {
    digitalWrite(PUMP_PIN, HIGH);
  } else if (soilMoisturePercent >= 60) {
    digitalWrite(PUMP_PIN, LOW);
  }
  
  //status pompa blynk -- hanya kirim jika ada internet
  if (WiFi.status() == WL_CONNECTED) {
    int pumpState = digitalRead(PUMP_PIN);  // Baca status pin pompa
    if (pumpState == HIGH) {
      Blynk.virtualWrite(VPIN_PUMP_STATUS, "Nyala");
    } else {
      Blynk.virtualWrite(VPIN_PUMP_STATUS, "Mati");
    }
  }

  //status pH
  String statusPH;
  if (phValue < 7.0) {
    statusPH = "Asam";
  } else if (phValue == 7.0) {
    statusPH = "Netral";
  } else {
    statusPH = "Basa";
  }

  Serial.print("pH: ");
  Serial.println(phValue);
  Serial.print(" - Status: ");
  Serial.println(statusPH);

  // Hanya kirim ke Blynk jika ada internet
  if (WiFi.status() == WL_CONNECTED) {
    Blynk.virtualWrite(V7, statusPH);
  }

  // Serial Output
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.print(" %, Soil Moisture: ");
  Serial.print(soilMoisturePercent);
  Serial.print("% - ");
  Serial.print(", pH: ");
  Serial.print(phValue);
  Serial.print(" TDS: ");
  Serial.print(tdsValue);
  Serial.println(" ppm");

  // LCD Display (Tanpa pH dan TDS)
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Suhu: ");
  lcd.print(temperature);
  lcd.print(" C");

  lcd.setCursor(0, 1);
  lcd.print("Soil: ");
  lcd.print(soilMoisturePercent);
  lcd.print("% ");
  lcd.print(soilStatus);

  lcd.setCursor(0, 2);
  lcd.print("TDS: ");
  lcd.print(tdsValue);
  lcd.print(" ppm");

  lcd.setCursor(0, 3);
  lcd.print("pH: ");
  lcd.print(phValue);
  lcd.print(" ");
  lcd.print(statusPH);

  // Kirim ke Blynk setiap 2 menit jika ada internet
  unsigned long currentMillis = millis();
  if (WiFi.status() == WL_CONNECTED && (currentMillis - previousMillis >= interval)) {
    previousMillis = currentMillis;
    Blynk.virtualWrite(V0, temperature);
    Blynk.virtualWrite(V2, soilStatus);  // 
    Blynk.virtualWrite(V3, phValue);       // pH
    Blynk.virtualWrite(V4, tdsValue); // TDS
    Serial.println("Data berhasil dikirim ke Blynk.");

    if (tdsValue < 560) {
      Serial.println("TDS rendah, mengatur nutrisi...");
      myServo.write(180);   // Buka saluran nutrisi
      delay(3000);         // Tunggu 3 detik
      myServo.write(0);    // Tutup kembali
    }
  } else if (tdsValue < 560) {
    // Jika TDS rendah dan offline, tetap kontrol servo!
    Serial.println("TDS rendah (offline), mengatur nutrisi...");
    myServo.write(180);   // Buka saluran nutrisi
    delay(3000);         // Tunggu 3 detik
    myServo.write(0);    // Tutup kembali
  }

  delay(2000);
}