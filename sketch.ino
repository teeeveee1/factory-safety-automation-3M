#include "DHT.h"

#define DHT_PIN 15
#define DHT_TYPE DHT22

#define VIBRATION_PIN 34
#define LOAD_PIN 35
#define SMOKE_PIN 32

#define RED_LED 14
#define ORANGE_LED 27
#define YELLOW_LED 26
#define GREEN_LED 25

DHT dht(DHT_PIN, DHT_TYPE);

void setup() {
  Serial.begin(115200);
  dht.begin();

  pinMode(RED_LED, OUTPUT);
  pinMode(ORANGE_LED, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
}

void loop() {

  float temperature = dht.readTemperature();
  int vibration = analogRead(VIBRATION_PIN);
  int load = analogRead(LOAD_PIN);
  int smoke = analogRead(SMOKE_PIN);

  int riskScore = 0;

  // Turn all LEDs off
  digitalWrite(RED_LED, LOW);
  digitalWrite(ORANGE_LED, LOW);
  digitalWrite(YELLOW_LED, LOW);
  digitalWrite(GREEN_LED, LOW);

  // -------------------------
  // TEMPERATURE
  // -------------------------

  if (temperature >= 70) {
    riskScore += 5;
  }
  else if (temperature >= 55) {
    riskScore += 2;
  }
  else if (temperature >= 40) {
    riskScore += 1;
  }

  // -------------------------
  // VIBRATION
  // -------------------------

  if (vibration >= 3500) {
    riskScore += 4;
  }
  else if (vibration >= 2500) {
    riskScore += 2;
  }
  else if (vibration >= 1500) {
    riskScore += 1;
  }

  // -------------------------
  // LOAD
  // -------------------------

  if (load >= 3500) {
    riskScore += 4;
  }
  else if (load >= 2500) {
    riskScore += 2;
  }
  else if (load >= 1500) {
    riskScore += 1;
  }

  // -------------------------
  // SMOKE / GAS
  // -------------------------

  if (smoke >= 3000) {
    riskScore += 5;
  }
  else if (smoke >= 2000) {
    riskScore += 3;
  }
  else if (smoke >= 1000) {
    riskScore += 1;
  }

  // -------------------------
  // DISPLAY READINGS
  // -------------------------

  Serial.println("-------------------------");

  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" C");

  Serial.print("Vibration: ");
  Serial.println(vibration);

  Serial.print("Load: ");
  Serial.println(load);

  Serial.print("Smoke/Gas: ");
  Serial.println(smoke);

  Serial.print("Risk Score: ");
  Serial.println(riskScore);

  Serial.print("Status: ");

  // -------------------------
  // FINAL RISK LEVEL
  // -------------------------

  if (riskScore >= 9) {

    digitalWrite(RED_LED, HIGH);
    Serial.println("CRITICAL");

  }
  else if (riskScore >= 6) {

    digitalWrite(ORANGE_LED, HIGH);
    Serial.println("HIGH RISK");

  }
  else if (riskScore >= 3) {

    digitalWrite(YELLOW_LED, HIGH);
    Serial.println("WARNING");

  }
  else {

    digitalWrite(GREEN_LED, HIGH);
    Serial.println("NORMAL");
  }

  delay(2000);
}