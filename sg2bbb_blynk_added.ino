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

// Ganti dengan SSID dan password WiFi kamu
char ssid[] = "Susharyatno";
char pass[] = "istikom4h";
// Pin Definitions
#define DHTPIN 4
#define SOIL_MOISTURE_PIN 34
#define LDR_PIN 13
#define PH_PIN 32 // Pin analog untuk sensor pH
#define PUMP_PIN 5 // Pin digital untuk relay pompa

// DHT11 sensor
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// LCD I2C
LiquidCrystal_I2C lcd(0x27, 20, 4);

// Nilai kalibrasi pH
float PH4 = 3.2;
float PH7 = 2.5;

void setup() {
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  // Initialize the serial communication
  Serial.begin(115200);
  
  // Initialize the DHT11 sensor
  dht.begin();
  
  // Initialize the LCD
  lcd.init();
  lcd.backlight();

  // Initialize the pump relay pin
  pinMode(PUMP_PIN, OUTPUT);
  digitalWrite(PUMP_PIN, LOW); // Matikan pompa pada awalnya
  
  // Display initial message
  lcd.setCursor(0, 0);
  lcd.print("SmartFarm System");
  lcd.setCursor(0, 1);
  lcd.print("Initializing...");
  delay(2000);
}

void loop() {
  Blynk.run();
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();
  int soilMoisture = analogRead(SOIL_MOISTURE_PIN);
  int phValue = analogRead(PH_PIN);
  float voltage = phValue * (3.3 / 4095.0);
  float ph = 7 + ((voltage - PH7) / (PH7 - PH4)) * 3;

  Blynk.virtualWrite(V0, temperature);
  Blynk.virtualWrite(V1, humidity);
  Blynk.virtualWrite(V2, soilMoisture);
  Blynk.virtualWrite(V3, ph);

  
  // Check if any reads failed and exit early (to try again).
  if (isnan(temperature) || isnan(humidity)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    lcd.setCursor(0, 3);
    lcd.print("DHT Error         ");
    delay(2000); // wait for 2 seconds before trying again
    return;
  }
  
  // Read soil moisture value
  int soilMoistureValue = analogRead(SOIL_MOISTURE_PIN);
  
int ldrValue = analogRead(LDR_PIN);
Serial.print("LDR Value: ");
Serial.println(ldrValue);

  
  // Read pH value
  int nilai_analog_PH = analogRead(PH_PIN);
  double TeganganPh = 3.3 / 4095.0 * nilai_analog_PH;
  float Ph_step = (PH4 - PH7) / 3.0;
  float Po = 7.00 + ((PH7 - TeganganPh) / Ph_step);

   // Control pump based on soil moisture value
  if (soilMoistureValue > 2000) {
    digitalWrite(PUMP_PIN, HIGH); // Hidupkan pompa
  } else if (soilMoistureValue <= 1700) {
    digitalWrite(PUMP_PIN, LOW); // Matikan pompa
  }

  // Print values to Serial
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.print(" *C, Humidity: ");
  Serial.print(humidity);
  Serial.print(" %, Soil Moisture: ");
  Serial.print(soilMoistureValue);
  Serial.print(", LDR: ");
  Serial.print(ldrValue);
  Serial.print(", pH: ");
  Serial.println(Po);

  // Display values on LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Suhu: ");
  lcd.print(temperature);
  lcd.print(" C");

  lcd.setCursor(0, 1);
  lcd.print("Kelembaban: ");
  lcd.print(humidity);
  lcd.print(" %");

  lcd.setCursor(0, 2);
  lcd.print("Soil Moist: ");
  lcd.print(soilMoistureValue);

  lcd.setCursor(0, 3);
  lcd.print("Light: ");
  lcd.print(ldrValue); //Pastikan Semua Nilai Data Tertampil
  lcd.print(" Lux,");

  String phText = "pH: " + String(Po, 2) + " "; // Menambahkan spasi di akhir teks
  scrollText("Light: " + String(ldrValue) + " Lux, " + phText, 3, 0, 1000);
  
  // Wait a few seconds between measurements.
  delay(2000);
}

// Fungsi untuk scrolling teks pada LCD
void scrollText(String message, int row, int startCol, int delayTime) {
  int len = message.length();
  // Membuat teks berjalan dengan menampilkan substring pada posisi yang sesuai
  for (int i = 0; i < len - 19; i++) {
    lcd.setCursor(startCol, row);
    lcd.print(message.substring(i, i + 20)); // Menampilkan 20 karakter per baris
    delay(delayTime);
  }
}
