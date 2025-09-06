#include <Wire.h>
#include <Arduino.h>

#define TCA9548A_ADDR 0x70  // Endereço padrão do multiplexador

// Função para selecionar um canal no TCA9548A
void tcaSelect(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(1 << channel); // Ativa apenas o canal desejado
  Wire.endTransmission();
  delay(10); // Pequena pausa para garantir comutação
}

void initTCS(){
    
}

void setup() {
  Wire.begin();
  Serial.begin(9600);
  delay(1000);
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

void loop() {
  // Nada no loop — scanner roda uma vez no setup
}
