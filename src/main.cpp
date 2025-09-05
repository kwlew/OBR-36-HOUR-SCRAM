#include <Arduino.h>
#include <PIDEasy.h>
#include <QTRSensors.h>

// Definições dos pinos
#define ENA 5   // Enable Motor A
#define ENB 6   // Enable Motor B
#define IN1 A11
#define IN2 A10
#define IN3 A9
#define IN4 A8

PID lineFollower(1.0, 0.0, 0.0);

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


void setup() {
  // Configuração dos pinos como saída
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
}

void loop() {
  motor(70, 200);
  delay(1000);
  motor(0, 0);
  delay(500);
  motor(-70, -200);
  delay(1000);
}