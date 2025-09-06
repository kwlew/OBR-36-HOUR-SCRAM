#include <Arduino.h>
#include <PIDEasy.h>
#include <QTRSensors.h>
#include <Adafruit_TCS34725.h>
#include <Wire.h>

// Definições dos pinos
#define ENA 5   // Enable Motor A
#define ENB 6   // Enable Motor B
#define IN1 A11
#define IN2 A10
#define IN3 A9
#define IN4 A8
#define TCA9548A_ADDR 0x70
#define TCS 0x29
#define Trja 30
#define Echo 32

QTRSensors qtr;
PID lineFollower(0.3, 0.0, 2.5);

float hue = 0.0;
float hsv[3];
uint16_t r, g, b, c, colorTemp, lux;
uint16_t r1, g1, b1, c1, colorTemp1, lux1;
const uint16_t pinos[8] = {A0, A1, A2, A3, A4, A5, A6, A7};
const uint8_t sensorCount = 8;
uint16_t sensorValues[sensorCount];
uint16_t limite = 860;
Adafruit_TCS34725 tcs0 = Adafruit_TCS34725();
Adafruit_TCS34725 tcs1 = Adafruit_TCS34725();

void tcaSelect(uint8_t channel) {
  if (channel > 7) return;

  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(1 << channel); // Ativa apenas o canal desejado
  Wire.endTransmission();
  delay(10); // Pequena pausa para garantir comutação
}

void initTCS(){
  tcaSelect(0);
  if (tcs0.begin()){
    Serial.println("TCS34725 da Esquerda encontrado!");
  }
  else{
    Serial.println("Erro ao encontrar TCS da esquerda!");
  }
  delay(200);
  tcaSelect(1);
  if(tcs1.begin()){
    Serial.println("TCS34725 da Direita encontrado!");
  }
  else{
    Serial.println("Erro ao encontrar TCS da Direita!");
  }
}

float mix(float a, float b, float t) { return a + (b - a) * t; }
float step(float e, float x) { return x < e ? 0.0 : 1.0; }

void rgbToHsv(float r, float g, float b, float &h, float &s, float &v) {
  float maxVal = max(r, max(g, b));
  float minVal = min(r, min(g, b));
  float delta = maxVal - minVal;

  v = maxVal;

  if (maxVal != 0)
    s = delta / maxVal;
  else {
    s = 0;
    h = -1;
    return;
  }

  if (delta == 0) {
    h = 0; // Sem matiz
  } else if (maxVal == r) {
    h = 60 * fmod(((g - b) / delta), 6.0);
  } else if (maxVal == g) {
    h = 60 * (((b - r) / delta) + 2);
  } else if (maxVal == b) {
    h = 60 * (((r - g) / delta) + 4);
  }

  if (h < 0)
    h += 360;
}

// Leitura e conversão do sensor da esquerda
void readTCSEsquerda() {
  tcaSelect(0);
  tcs0.getRawData(&r, &g, &b, &c);

  // Normalizar os valores (0-1) dividindo por clear
  float rf = (float)r / c;
  float gf = (float)g / c;
  float bf = (float)b / c;

  float h, s, v;
  rgbToHsv(rf, gf, bf, h, s, v);

  Serial.println("---- ESQUERDA ----");
  Serial.print("R: "); Serial.print(r);
  Serial.print(" G: "); Serial.print(g);
  Serial.print(" B: "); Serial.print(b);
  Serial.print(" C: "); Serial.println(c);
  Serial.print("Hue: "); Serial.print(h);
  Serial.print(" Sat: "); Serial.print(s * 100);
  Serial.print("% Val: "); Serial.print(v * 100);
  Serial.println("%");
}

// Leitura e conversão do sensor da direita
void readTCSDireita() {
  tcaSelect(1);
  tcs1.getRawData(&r1, &g1, &b1, &c1);

  float rf = (float)r1 / c1;
  float gf = (float)g1 / c1;
  float bf = (float)b1 / c1;

  float h, s, v;
  rgbToHsv(rf, gf, bf, h, s, v);

  Serial.println("---- DIREITA ----");
  Serial.print("R: "); Serial.print(r1);
  Serial.print(" G: "); Serial.print(g1);
  Serial.print(" B: "); Serial.print(b1);
  Serial.print(" C: "); Serial.println(c1);
  Serial.print("Hue: "); Serial.print(h);
  Serial.print(" Sat: "); Serial.print(s * 100);
  Serial.print("% Val: "); Serial.print(v * 100);
  Serial.println("%");
}

void initQTR(){
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, sensorCount);
  delay(500);
}

void I2C_Scanner(){
  Serial.println(F("🔍 Scanner I2C com TCA9548A"));

  for (uint8_t channel = 0; channel < 8; channel++) {
    tcaSelect(channel);
    Serial.print(F("\n🔀 Canal ")); Serial.print(channel); Serial.println(F(":"));

    uint8_t devicesFound = 0;

    for (uint8_t addr = 1; addr < 127; addr++) {
      Wire.beginTransmission(addr);
      uint8_t error = Wire.endTransmission();

      if (error == 0) {
        Serial.print(F("  ✅ Dispositivo I2C encontrado no endereço 0x"));
        if (addr < 16) Serial.print("0");
        Serial.print(addr, HEX);
        Serial.println();
        devicesFound++;
      } else if (error == 4) {
        Serial.print(F("  ⚠️ Erro desconhecido no endereço 0x"));
        if (addr < 16) Serial.print("0");
        Serial.println(addr, HEX);
      }
    }

    if (devicesFound == 0) {
      Serial.println(F("  ❌ Nenhum dispositivo encontrado nesse canal."));
    }
  }

  Serial.println(F("\n✅ Varredura completa."));
}

void calibrateQTR(){
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  for (uint16_t i = 0; i<200; i++){
    qtr.calibrate();
    Serial.println("Calibrating...\n");
  }
  digitalWrite(LED_BUILTIN, LOW);
}

void motor(int velEsq, int velDir) {
  velEsq = constrain(velEsq, -255, 255);
  velDir = constrain(velDir, -255, 255);

  int esq = velEsq >= 0 ? 1 : 0;
  int dir = velDir >= 0 ? 1 : 0;

  digitalWrite(IN1, !esq);
  digitalWrite(IN2, esq);
  digitalWrite(IN3, !dir);
  digitalWrite(IN4, dir);

  if (velEsq == 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }
  if (velDir == 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }

  analogWrite(ENA, abs(velEsq));
  analogWrite(ENB, abs(velDir));
}

long readCase() {
  long output = 0;
  long multiplier = 1;
  qtr.readLineBlack(sensorValues);

  for (int i = 0; i < sensorCount; i++) {
    if (sensorValues[i] >= limite) {
      output += 1 * multiplier;
    }

    multiplier *= 10;
  }

  return output;
}

float dist(){
  long duration;
  float distance;

  digitalWrite(Trja, 0);
  delayMicroseconds(2);
  digitalWrite(Trja, 1);
  delayMicroseconds(10);
  digitalWrite(Trja, 0);

  duration = pulseIn(Echo, 1);
  distance = duration * 0.034 / 2;

  Serial.println("Distance" + String(distance) + String(" cm"));
  delay(500);
  return distance;
}

void forward(){
  dist();
  uint16_t pos = qtr.readLineBlack(sensorValues);
  int error = pos - 3500;
  Serial.println("error: " + String(error));
  int output = lineFollower.compute(error, 1);
  Serial.println("output: " + String(output));
  motor(40 + output, 40 - output);
  Serial.println(qtr.readLineBlack(sensorValues));
}

void gap(){
  while (readCase() == 0){
    motor (60, 60);
  }
}

void setup() {
  Wire.begin();
  Serial.begin(9600);
  I2C_Scanner();  
  // Configuração dos pinos como saída
  pinMode(Trja, OUTPUT);
  pinMode(Echo, INPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  //initQTR();
  //calibrateQTR();
  //lineFollower.setConstrain(-40, 40);
}

void loop() {
  // if (readCase() == 0){
  //    Serial.println("Gap found!");
  //    gap();
  //  }
   //else{
   //  Serial.println("Forwarding!");
   //  forward();
   //}
  readTCSDireita();
  readTCSEsquerda();
  }