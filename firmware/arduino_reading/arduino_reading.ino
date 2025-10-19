/*
 * Reader for packets: 0xAA 0x55 <int8 L> <int8 R>
 * Board: Arduino Nano / Uno (ATmega328P)
 * Wire: ESP32-CAM TX (GPIO12) -> Nano RX0 (D0), GND common
 * Baud: 115200
 */

const uint8_t H1 = 0xAA;
const uint8_t H2 = 0x55;

enum { ST_H1, ST_H2, ST_L, ST_R } state = ST_H1;

int8_t L = 0, R = 0;
int8_t lastL = 127, lastR = 127;  // чтобы первое значение точно вывелось

// Таймаут между байтами, чтобы не «залипнуть» в середине пакета
const uint16_t BYTE_TIMEOUT_MS = 30;
uint32_t tLastByte = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; } // для некоторых плат/IDE, можно убрать

  Serial.println(F("---- Robot View RX ----"));
  Serial.println(F("Waiting: 0xAA 0x55 <L:int8> <R:int8> @115200"));
  Serial.println(F("Wiring: ESP32 TX(GPIO12) -> Nano RX0(D0), GND common"));
  Serial.println(F("---------------------------------------------"));
}

void loop() {
  // сбрасываем парсер, если слишком долго нет байтов
  if (millis() - tLastByte > BYTE_TIMEOUT_MS && state != ST_H1) {
    state = ST_H1;
  }

  while (Serial.available() > 0) {
    tLastByte = millis();
    uint8_t b = (uint8_t)Serial.read();

    switch (state) {
      case ST_H1:
        state = (b == H1) ? ST_H2 : ST_H1;
        break;

      case ST_H2:
        if (b == H2) {
          state = ST_L;
        } else {
          // если пришёл снова 0xAA — остаёмся ждать 0x55
          state = (b == H1) ? ST_H2 : ST_H1;
        }
        break;

      case ST_L:
        L = (int8_t)b;  // знаковое преобразование
        state = ST_R;
        break;

      case ST_R:
        R = (int8_t)b;
        state = ST_H1;

        // Печатаем только при изменении, чтобы не забивать порт
        if (L != lastL || R != lastR) {
          lastL = L; lastR = R;
          Serial.print(F("L=")); Serial.print(L);
          Serial.print(F(" R=")); Serial.println(R);
        }
        break;
    }
  }
}
