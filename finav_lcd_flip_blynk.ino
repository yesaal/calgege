#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#define BLYNK_TEMPLATE_ID "TMPL6MpcgorMy"
#define BLYNK_TEMPLATE_NAME "tugas akhir"
#define BLYNK_AUTH_TOKEN "hc-Z09Nfq1AuoYO_CZk8z6nfOK9jtdSg"

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <ESP32Servo.h>

// WiFi credentials
char ssid[] = "Andromax-M2Y-F6BF";
char pass[] = "27385345";

// Pin Definitions
#define DHTPIN 4
#define SOIL_MOISTURE_PIN 34
#define PUMP_PIN 5
#define TDS_PIN 33
#define PH_SENSOR_PIN 32 
#define servoPin 12 
#define VPIN_PUMP_STATUS V6

// Servo
Servo myServo;

// DHT11 sensor
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// LCD I2C
LiquidCrystal_I2C lcd(0x27, 20, 4);

// pH Sensor
unsigned long int avgval;
int buffer_arr[10];
float voltage, ph_act;

// Soil moisture calibration
const int soilDryValue = 3200;
const int soilWetValue = 1500;

// TDS sensor
float VREF = 3.3;
float adcResolution = 4095.0;
float kFactor = 1;

// LCD flip variables
int lcdPage = 1;
unsigned long lastLcdFlip = 0;
const unsigned long lcdFlipInterval = 5000;

// Status flags
bool pumpStatus = false;
bool nutrisiDiberikan = false;

void setup() {
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  Serial.begin(115200);

  dht.begin();
  lcd.init();
  lcd.backlight();

  pinMode(PUMP_PIN, OUTPUT);
  myServo.attach(servoPin);
}

void loop() {
  
  Blynk.run();

  Blynk.virtualWrite(V0, temperature);         // Temperature
  Blynk.virtualWrite(V2, soilMoisturePercent); // Soil Moisture
  Blynk.virtualWrite(V3, ph_act);              // pH
  Blynk.virtualWrite(V4, tdsValue);            // TDS

  String pumpStatusText = digitalRead(PUMP_PIN) ? "ON" : "OFF";
  Blynk.virtualWrite(V6, pumpStatusText);      // Pump status

  String phStatusText = "";
  if (ph_act < 5.5) {
    phStatusText = "Asam";
  } else if (ph_act > 7.5) {
    phStatusText = "Basa";
  } else {
    phStatusText = "Netral";
  }
  Blynk.virtualWrite(V7, phStatusText);        // pH status

Blynk.run();

  unsigned long currentMillis = millis();

  // LCD flip timer
  if (currentMillis - lastLcdFlip >= lcdFlipInterval) {
    lcdPage = (lcdPage == 1) ? 2 : 1;
    lastLcdFlip = currentMillis;
    lcd.clear();
  }

  // Sensor readings
  float h = dht.readHumidity(); // tetap dibaca walau tidak ditampilkan
  float t = dht.readTemperature();

  int soilMoistureValue = analogRead(SOIL_MOISTURE_PIN);
  int soilPercent = map(soilMoistureValue, soilDryValue, soilWetValue, 0, 100);
  soilPercent = constrain(soilPercent, 0, 100);

  int tdsValue = analogRead(TDS_PIN);
  float tdsVoltage = tdsValue * (VREF / adcResolution);
  float tds = (133.42 * pow(tdsVoltage, 3) - 255.86 * pow(tdsVoltage, 2) + 857.39 * tdsVoltage) * kFactor;

  // pH Sensor
  for (int i = 0; i < 10; i++) {
    buffer_arr[i] = analogRead(PH_SENSOR_PIN);
    delay(10);
  }
  avgval = 0;
  for (int i = 0; i < 10; i++) avgval += buffer_arr[i];
  avgval = avgval / 10;
  voltage = avgval * (VREF / adcResolution);
  ph_act = 3.5 * voltage;

  // Tentukan status pH
  String statusPH = "";
  if (ph_act < 7.0) {
    statusPH = "Asam";
  } else if (ph_act > 7.0) {
    statusPH = "Basa";
  } else {
    statusPH = "Netral";
  }

  // Kontrol pompa dengan dua ambang batas (hysteresis)
  if (soilPercent <= 45) {
    digitalWrite(PUMP_PIN, HIGH);
    pumpStatus = true;
  } else if (soilPercent >= 60) {
    digitalWrite(PUMP_PIN, LOW);
    pumpStatus = false;
  }

  // Kontrol nutrisi (servo)
  if (tds < 300) {
    myServo.write(90);
    delay(1000);
    myServo.write(0);
    nutrisiDiberikan = true;
  } else {
    nutrisiDiberikan = false;
  }

  // LCD display
  if (lcdPage == 1) {
    lcd.setCursor(0, 0);
    lcd.print("Temp: " + String(t) + " C");
    lcd.setCursor(0, 1);
    lcd.print("Soil : " + String(soilPercent) + "%");
    lcd.setCursor(0, 2);
    lcd.print("TDS : " + String(tds, 0) + " ppm");
    lcd.setCursor(0, 3);
    lcd.print("pH : " + String(ph_act, 2) + " " + statusPH);
  } else if (lcdPage == 2) {
    lcd.setCursor(0, 0);
    lcd.print("Pompa   : ");
    lcd.print(pumpStatus ? "ON " : "OFF");
    lcd.setCursor(0, 1);
    lcd.print("Nutrisi : ");
    lcd.print(nutrisiDiberikan ? "DIBERI" : "TIDAK");
  }

  // Kirim ke Blynk
  Blynk.virtualWrite(VPIN_PUMP_STATUS, pumpStatus);

  delay(1000);
}
