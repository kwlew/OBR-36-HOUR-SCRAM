#include <Arduino.h>
#include <PIDEasy.h>
#include <QTRSensors.h>
#include <Adafruit_TCS34725.h>

// Definições dos pinos
#define ENA 5   // Enable Motor A
#define ENB 6   // Enable Motor B
#define IN1 A11
#define IN2 A10
#define IN3 A9
#define IN4 A8

QTRSensors qtr;
PID lineFollower(0.28, 0.0, 0.7);

const uint16_t pinos[8] = {A0, A1, A2, A3, A4, A5, A6, A7};
const uint8_t sensorCount = 8;
uint16_t sensorValues[sensorCount];
uint16_t limite = 860;

void initQTR(){
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, sensorCount);
  delay(500);
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

void forward(){
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
  Serial.begin(9600);  
  // Configuração dos pinos como saída
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  initQTR();
  calibrateQTR();
  lineFollower.setConstrain(-80, 80);
}

void loop() {
  if (readCase() == 0){
    gap();
  }
  else{
    forward();
  }
}