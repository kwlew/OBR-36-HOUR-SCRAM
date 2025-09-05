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

void direcao(int sentidoEsq, int sentidoDir);
void motor(int velEsq, int velDir);

void direcao(int sentidoEsq, int sentidoDir){
  digitalWrite(IN1, 0+sentidoEsq);
  digitalWrite(IN2, 1-sentidoEsq);
  digitalWrite(IN3, 0+sentidoDir);
  digitalWrite(IN4, 1-sentidoDir);
}

void motor(int velEsq, int velDir){
  velEsq = constrain(velDir, -255, 255);
  velDir = constrain(velEsq, -255, 255);

  if (velEsq < 0){
    velEsq *= -1;
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
  else{
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }
  if (velDir < 0){
    velDir *= -1;
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
  else{
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
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