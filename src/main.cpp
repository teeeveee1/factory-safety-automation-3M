#include <Arduino.h>
#include "DHT.h"
 
// ============================================================
//  IoT-Based Predictive Factory Safety & Machine Monitoring
//  Three machines, three cascaded 74HC595 shift registers
// ============================================================
 
// ---------- Sensor pins ----------
#define M1_DHT_PIN 4
#define M1_VIBRATION_PIN 34
#define M1_LOAD_PIN 35
#define M1_SMOKE_PIN 32
 
#define M2_DHT_PIN 16
#define M2_VIBRATION_PIN 36   // GPIO36 = "VP" pin on the Wokwi board
#define M2_LOAD_PIN 39        // GPIO39 = "VN" pin on the Wokwi board
#define M2_SMOKE_PIN 33
 
#define M3_DHT_PIN 17
#define M3_VIBRATION_PIN 25
#define M3_LOAD_PIN 26
#define M3_SMOKE_PIN 27
 
#define DHT_TYPE DHT22
 
// ---------- 74HC595 control pins ----------
#define DATA_PIN 5    // -> SR1 DS
#define CLOCK_PIN 18  // -> SR1/SR2/SR3 SHCP
#define LATCH_PIN 19  // -> SR1/SR2/SR3 STCP
 
// Set to 0 to disable the audible beep and just hold the buzzer pin HIGH.
// The Wokwi buzzer is a piezo: it only makes sound when its pin toggles
// at an audio frequency, so a steady HIGH is silent.
#define BUZZER_BEEP_ENABLED 1
 
DHT dht1(M1_DHT_PIN, DHT_TYPE);
DHT dht2(M2_DHT_PIN, DHT_TYPE);
DHT dht3(M3_DHT_PIN, DHT_TYPE);
 
// ------------------------------------------------------------
// Output map (matches diagram.json exactly)
//
// SR1: Q0 M1 green, Q1 M1 yellow, Q2 M1 orange, Q3 M1 red,
//      Q4 M1 purple, Q5 M1 relay, Q6 M2 green, Q7 M2 yellow
// SR2: Q0 M2 orange, Q1 M2 red, Q2 M2 purple, Q3 M2 relay,
//      Q4 M3 green, Q5 M3 yellow, Q6 M3 orange, Q7 M3 red
// SR3: Q0 M3 purple, Q1 M3 relay, Q2 shared buzzer
// ------------------------------------------------------------
enum { SR1 = 0, SR2 = 1, SR3 = 2 };
 
struct OutPin {
  uint8_t reg;  // which shift register (SR1/SR2/SR3)
  uint8_t bit;  // Q0..Q7
};
 
//                          Machine 1     Machine 2     Machine 3
const OutPin GREEN_LED[3]  = {{SR1, 0},  {SR1, 6},  {SR2, 4}};
const OutPin YELLOW_LED[3] = {{SR1, 1},  {SR1, 7},  {SR2, 5}};
const OutPin ORANGE_LED[3] = {{SR1, 2},  {SR2, 0},  {SR2, 6}};
const OutPin RED_LED[3]    = {{SR1, 3},  {SR2, 1},  {SR2, 7}};
const OutPin PURPLE_LED[3] = {{SR1, 4},  {SR2, 2},  {SR3, 0}};  // simulated shutdown indicator
const OutPin RELAY[3]      = {{SR1, 5},  {SR2, 3},  {SR3, 1}};
const OutPin BUZZER        = {SR3, 2};
 
inline void setOut(byte sr[3], const OutPin &p) {
  sr[p.reg] |= (byte)(1 << p.bit);
}
 
inline void clearOut(byte sr[3], const OutPin &p) {
  sr[p.reg] &= (byte)~(1 << p.bit);
}
 
// Chain: ESP32 DATA -> SR1 -> SR2 -> SR3.
// The first byte shifted in travels the farthest, so SR3's byte goes first.
// MSBFIRST places bit 7 on Q7 and bit 0 on Q0 of each register.
void updateOutputs(const byte sr[3]) {
  digitalWrite(LATCH_PIN, LOW);
  shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, sr[SR3]);
  shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, sr[SR2]);
  shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, sr[SR1]);
  digitalWrite(LATCH_PIN, HIGH);
}
 
// Toggle the buzzer pin at roughly 1.2 kHz for about a quarter second,
// leaving the buzzer bit in its alarm (HIGH) state afterwards.
void beepBuzzer(byte sr[3]) {
  byte tmp[3] = {sr[SR1], sr[SR2], sr[SR3]};
  for (int i = 0; i < 300; i++) {
    setOut(tmp, BUZZER);
    updateOutputs(tmp);
    delayMicroseconds(400);
    clearOut(tmp, BUZZER);
    updateOutputs(tmp);
    delayMicroseconds(400);
  }
  updateOutputs(sr);
}
 
int calculateRisk(float temperature, int vibration,
                  int load, int smoke) {
  int risk = 0;
 
  if (temperature >= 70) risk += 5;
  else if (temperature >= 55) risk += 2;
  else if (temperature >= 40) risk += 1;
 
  if (vibration >= 3500) risk += 4;
  else if (vibration >= 2500) risk += 2;
  else if (vibration >= 1500) risk += 1;
 
  if (load >= 3500) risk += 4;
  else if (load >= 2500) risk += 2;
  else if (load >= 1500) risk += 1;
 
  // Provisional raw ADC values, NOT calibrated ppm.
  if (smoke >= 3900) risk += 5;
  else if (smoke >= 3800) risk += 3;
  else if (smoke >= 3700) risk += 1;
 
  return risk;
}
 
// machineIndex: 0, 1 or 2
void applyMachineOutputs(byte sr[3], int machineIndex, int risk) {
  if (risk >= 9) {
    setOut(sr, RED_LED[machineIndex]);
    setOut(sr, PURPLE_LED[machineIndex]);
    setOut(sr, RELAY[machineIndex]);
  } else if (risk >= 6) {
    setOut(sr, ORANGE_LED[machineIndex]);
    setOut(sr, RELAY[machineIndex]);
  } else if (risk >= 3) {
    setOut(sr, YELLOW_LED[machineIndex]);
  } else {
    setOut(sr, GREEN_LED[machineIndex]);
  }
}
 
void displayMachine(int number, float temperature,
                    int vibration, int load, int smoke, int risk) {
  Serial.println();
  Serial.println("========================================");
  Serial.print("             MACHINE ");
  Serial.println(number);
  Serial.println("========================================");
 
  Serial.print("Temperature : ");
  Serial.print(temperature);
  Serial.println(" C");
 
  Serial.print("Vibration   : ");
  Serial.println(vibration);
 
  Serial.print("Load        : ");
  Serial.println(load);
 
  Serial.print("Smoke/Gas ADC: ");
  Serial.println(smoke);
 
  Serial.print("Risk Score  : ");
  Serial.println(risk);
 
  Serial.print("Status      : ");
 
  if (risk >= 9) {
    Serial.println("CRITICAL");
    Serial.println("Alarm       : ACTIVATED");
    Serial.println("Cooling     : ACTIVATED");
    Serial.println("Shutdown    : INDICATED (simulated)");
  } else if (risk >= 6) {
    Serial.println("HIGH RISK");
    Serial.println("Alarm       : ACTIVATED");
    Serial.println("Cooling     : ACTIVATED");
  } else if (risk >= 3) {
    Serial.println("WARNING");
    Serial.println("Action      : INCREASED MONITORING");
  } else {
    Serial.println("NORMAL");
    Serial.println("Action      : NO ACTION REQUIRED");
  }
}
 
void setup() {
  Serial.begin(115200);
 
  dht1.begin();
  dht2.begin();
  dht3.begin();
 
  pinMode(DATA_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(LATCH_PIN, OUTPUT);
 
  digitalWrite(CLOCK_PIN, LOW);
  digitalWrite(LATCH_PIN, HIGH);
 
  byte off[3] = {0, 0, 0};
  updateOutputs(off);
 
  Serial.println("Factory Safety Monitoring System");
  Serial.println("Three-machine monitoring initialized.");
}
 
void loop() {
  float t1 = dht1.readTemperature();
  int v1 = analogRead(M1_VIBRATION_PIN);
  int l1 = analogRead(M1_LOAD_PIN);
  int s1 = analogRead(M1_SMOKE_PIN);
 
  float t2 = dht2.readTemperature();
  int v2 = analogRead(M2_VIBRATION_PIN);
  int l2 = analogRead(M2_LOAD_PIN);
  int s2 = analogRead(M2_SMOKE_PIN);
 
  float t3 = dht3.readTemperature();
  int v3 = analogRead(M3_VIBRATION_PIN);
  int l3 = analogRead(M3_LOAD_PIN);
  int s3 = analogRead(M3_SMOKE_PIN);
 
  if (isnan(t1)) t1 = 0;
  if (isnan(t2)) t2 = 0;
  if (isnan(t3)) t3 = 0;
 
  int r1 = calculateRisk(t1, v1, l1, s1);
  int r2 = calculateRisk(t2, v2, l2, s2);
  int r3 = calculateRisk(t3, v3, l3, s3);
 
  byte sr[3] = {0, 0, 0};
 
  applyMachineOutputs(sr, 0, r1);
  applyMachineOutputs(sr, 1, r2);
  applyMachineOutputs(sr, 2, r3);
 
  // Shared buzzer: any machine at HIGH RISK (6+) or CRITICAL
  bool alarm = (r1 >= 6 || r2 >= 6 || r3 >= 6);
  if (alarm) {
    setOut(sr, BUZZER);
  }
 
  updateOutputs(sr);
 
#if BUZZER_BEEP_ENABLED
  if (alarm) {
    beepBuzzer(sr);
  }
#endif
 
  displayMachine(1, t1, v1, l1, s1, r1);
  displayMachine(2, t2, v2, l2, s2, r2);
  displayMachine(3, t3, v3, l3, s3, r3);
 
  Serial.println();
  Serial.println("========== FACTORY SUMMARY ==========");
  Serial.print("Machine 1 Risk: ");
  Serial.println(r1);
  Serial.print("Machine 2 Risk: ");
  Serial.println(r2);
  Serial.print("Machine 3 Risk: ");
  Serial.println(r3);
 
  delay(5000);
}#include <Arduino.h>
#include "DHT.h"
 
// ============================================================
//  IoT-Based Predictive Factory Safety & Machine Monitoring
//  Three machines, three cascaded 74HC595 shift registers
// ============================================================
 
// ---------- Sensor pins ----------
#define M1_DHT_PIN 4
#define M1_VIBRATION_PIN 34
#define M1_LOAD_PIN 35
#define M1_SMOKE_PIN 32
 
#define M2_DHT_PIN 16
#define M2_VIBRATION_PIN 36   // GPIO36 = "VP" pin on the Wokwi board
#define M2_LOAD_PIN 39        // GPIO39 = "VN" pin on the Wokwi board
#define M2_SMOKE_PIN 33
 
#define M3_DHT_PIN 17
#define M3_VIBRATION_PIN 25
#define M3_LOAD_PIN 26
#define M3_SMOKE_PIN 27
 
#define DHT_TYPE DHT22
 
// ---------- 74HC595 control pins ----------
#define DATA_PIN 5    // -> SR1 DS
#define CLOCK_PIN 18  // -> SR1/SR2/SR3 SHCP
#define LATCH_PIN 19  // -> SR1/SR2/SR3 STCP
 
// Set to 0 to disable the audible beep and just hold the buzzer pin HIGH.
// The Wokwi buzzer is a piezo: it only makes sound when its pin toggles
// at an audio frequency, so a steady HIGH is silent.
#define BUZZER_BEEP_ENABLED 1
 
DHT dht1(M1_DHT_PIN, DHT_TYPE);
DHT dht2(M2_DHT_PIN, DHT_TYPE);
DHT dht3(M3_DHT_PIN, DHT_TYPE);
 
// ------------------------------------------------------------
// Output map (matches diagram.json exactly)
//
// SR1: Q0 M1 green, Q1 M1 yellow, Q2 M1 orange, Q3 M1 red,
//      Q4 M1 purple, Q5 M1 relay, Q6 M2 green, Q7 M2 yellow
// SR2: Q0 M2 orange, Q1 M2 red, Q2 M2 purple, Q3 M2 relay,
//      Q4 M3 green, Q5 M3 yellow, Q6 M3 orange, Q7 M3 red
// SR3: Q0 M3 purple, Q1 M3 relay, Q2 shared buzzer
// ------------------------------------------------------------
enum { SR1 = 0, SR2 = 1, SR3 = 2 };
 
struct OutPin {
  uint8_t reg;  // which shift register (SR1/SR2/SR3)
  uint8_t bit;  // Q0..Q7
};
 
//                          Machine 1     Machine 2     Machine 3
const OutPin GREEN_LED[3]  = {{SR1, 0},  {SR1, 6},  {SR2, 4}};
const OutPin YELLOW_LED[3] = {{SR1, 1},  {SR1, 7},  {SR2, 5}};
const OutPin ORANGE_LED[3] = {{SR1, 2},  {SR2, 0},  {SR2, 6}};
const OutPin RED_LED[3]    = {{SR1, 3},  {SR2, 1},  {SR2, 7}};
const OutPin PURPLE_LED[3] = {{SR1, 4},  {SR2, 2},  {SR3, 0}};  // simulated shutdown indicator
const OutPin RELAY[3]      = {{SR1, 5},  {SR2, 3},  {SR3, 1}};
const OutPin BUZZER        = {SR3, 2};
 
inline void setOut(byte sr[3], const OutPin &p) {
  sr[p.reg] |= (byte)(1 << p.bit);
}
 
inline void clearOut(byte sr[3], const OutPin &p) {
  sr[p.reg] &= (byte)~(1 << p.bit);
}
 
// Chain: ESP32 DATA -> SR1 -> SR2 -> SR3.
// The first byte shifted in travels the farthest, so SR3's byte goes first.
// MSBFIRST places bit 7 on Q7 and bit 0 on Q0 of each register.
void updateOutputs(const byte sr[3]) {
  digitalWrite(LATCH_PIN, LOW);
  shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, sr[SR3]);
  shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, sr[SR2]);
  shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, sr[SR1]);
  digitalWrite(LATCH_PIN, HIGH);
}
 
// Toggle the buzzer pin at roughly 1.2 kHz for about a quarter second,
// leaving the buzzer bit in its alarm (HIGH) state afterwards.
void beepBuzzer(byte sr[3]) {
  byte tmp[3] = {sr[SR1], sr[SR2], sr[SR3]};
  for (int i = 0; i < 300; i++) {
    setOut(tmp, BUZZER);
    updateOutputs(tmp);
    delayMicroseconds(400);
    clearOut(tmp, BUZZER);
    updateOutputs(tmp);
    delayMicroseconds(400);
  }
  updateOutputs(sr);
}
 
int calculateRisk(float temperature, int vibration,
                  int load, int smoke) {
  int risk = 0;
 
  if (temperature >= 70) risk += 5;
  else if (temperature >= 55) risk += 2;
  else if (temperature >= 40) risk += 1;
 
  if (vibration >= 3500) risk += 4;
  else if (vibration >= 2500) risk += 2;
  else if (vibration >= 1500) risk += 1;
 
  if (load >= 3500) risk += 4;
  else if (load >= 2500) risk += 2;
  else if (load >= 1500) risk += 1;
 
  // Provisional raw ADC values, NOT calibrated ppm.
  if (smoke >= 3900) risk += 5;
  else if (smoke >= 3800) risk += 3;
  else if (smoke >= 3700) risk += 1;
 
  return risk;
}
 
// machineIndex: 0, 1 or 2
void applyMachineOutputs(byte sr[3], int machineIndex, int risk) {
  if (risk >= 9) {
    setOut(sr, RED_LED[machineIndex]);
    setOut(sr, PURPLE_LED[machineIndex]);
    setOut(sr, RELAY[machineIndex]);
  } else if (risk >= 6) {
    setOut(sr, ORANGE_LED[machineIndex]);
    setOut(sr, RELAY[machineIndex]);
  } else if (risk >= 3) {
    setOut(sr, YELLOW_LED[machineIndex]);
  } else {
    setOut(sr, GREEN_LED[machineIndex]);
  }
}
 
void displayMachine(int number, float temperature,
                    int vibration, int load, int smoke, int risk) {
  Serial.println();
  Serial.println("========================================");
  Serial.print("             MACHINE ");
  Serial.println(number);
  Serial.println("========================================");
 
  Serial.print("Temperature : ");
  Serial.print(temperature);
  Serial.println(" C");
 
  Serial.print("Vibration   : ");
  Serial.println(vibration);
 
  Serial.print("Load        : ");
  Serial.println(load);
 
  Serial.print("Smoke/Gas ADC: ");
  Serial.println(smoke);
 
  Serial.print("Risk Score  : ");
  Serial.println(risk);
 
  Serial.print("Status      : ");
 
  if (risk >= 9) {
    Serial.println("CRITICAL");
    Serial.println("Alarm       : ACTIVATED");
    Serial.println("Cooling     : ACTIVATED");
    Serial.println("Shutdown    : INDICATED (simulated)");
  } else if (risk >= 6) {
    Serial.println("HIGH RISK");
    Serial.println("Alarm       : ACTIVATED");
    Serial.println("Cooling     : ACTIVATED");
  } else if (risk >= 3) {
    Serial.println("WARNING");
    Serial.println("Action      : INCREASED MONITORING");
  } else {
    Serial.println("NORMAL");
    Serial.println("Action      : NO ACTION REQUIRED");
  }
}
 
void setup() {
  Serial.begin(115200);
 
  dht1.begin();
  dht2.begin();
  dht3.begin();
 
  pinMode(DATA_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(LATCH_PIN, OUTPUT);
 
  digitalWrite(CLOCK_PIN, LOW);
  digitalWrite(LATCH_PIN, HIGH);
 
  byte off[3] = {0, 0, 0};
  updateOutputs(off);
 
  Serial.println("Factory Safety Monitoring System");
  Serial.println("Three-machine monitoring initialized.");
}
 
void loop() {
  float t1 = dht1.readTemperature();
  int v1 = analogRead(M1_VIBRATION_PIN);
  int l1 = analogRead(M1_LOAD_PIN);
  int s1 = analogRead(M1_SMOKE_PIN);
 
  float t2 = dht2.readTemperature();
  int v2 = analogRead(M2_VIBRATION_PIN);
  int l2 = analogRead(M2_LOAD_PIN);
  int s2 = analogRead(M2_SMOKE_PIN);
 
  float t3 = dht3.readTemperature();
  int v3 = analogRead(M3_VIBRATION_PIN);
  int l3 = analogRead(M3_LOAD_PIN);
  int s3 = analogRead(M3_SMOKE_PIN);
 
  if (isnan(t1)) t1 = 0;
  if (isnan(t2)) t2 = 0;
  if (isnan(t3)) t3 = 0;
 
  int r1 = calculateRisk(t1, v1, l1, s1);
  int r2 = calculateRisk(t2, v2, l2, s2);
  int r3 = calculateRisk(t3, v3, l3, s3);
 
  byte sr[3] = {0, 0, 0};
 
  applyMachineOutputs(sr, 0, r1);
  applyMachineOutputs(sr, 1, r2);
  applyMachineOutputs(sr, 2, r3);
 
  // Shared buzzer: any machine at HIGH RISK (6+) or CRITICAL
  bool alarm = (r1 >= 6 || r2 >= 6 || r3 >= 6);
  if (alarm) {
    setOut(sr, BUZZER);
  }
 
  updateOutputs(sr);
 
#if BUZZER_BEEP_ENABLED
  if (alarm) {
    beepBuzzer(sr);
  }
#endif
 
  displayMachine(1, t1, v1, l1, s1, r1);
  displayMachine(2, t2, v2, l2, s2, r2);
  displayMachine(3, t3, v3, l3, s3, r3);
 
  Serial.println();
  Serial.println("========== FACTORY SUMMARY ==========");
  Serial.print("Machine 1 Risk: ");
  Serial.println(r1);
  Serial.print("Machine 2 Risk: ");
  Serial.println(r2);
  Serial.print("Machine 3 Risk: ");
  Serial.println(r3);
 
  delay(5000);
}
