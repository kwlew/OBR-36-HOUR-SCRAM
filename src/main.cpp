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
int velEsq = 0, velDir = 0;
PID lineFollower(1.0, 0.0, 0.0);

void motor(int velEsq, int velDir);

void motor(int velEsq, int velDir) {
  velEsq = constrain(velEsq, -255, 255);
  velDir = constrain(velDir, -255, 255);

  // Motor Esquerdo
  if (velEsq > 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else if (velEsq < 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    velEsq = -velEsq;
  } else {
    // Freio ativo
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, HIGH);
  }

  // Motor Direito
  if (velDir > 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  } else if (velDir < 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    velDir = -velDir;
  } else {
    // Freio ativo
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, HIGH);
  }

  analogWrite(ENA, velEsq);
  analogWrite(ENB, velDir);
}


void setup() {
  // Configuração dos pinos como saída
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void loop() {
  motor(100, 100);
  delay(1000);
  motor(0, 0);
  delay(1000);
  motor(-100, -100);
  delay(10000);
}