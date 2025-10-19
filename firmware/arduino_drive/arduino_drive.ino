// === Робот: приём от ESP32-CAM (0xAA 0x55 L R), *2, НО L/R ПЕРЕСТАВЛЕНЫ МЕСТАМИ ===
// Пины моторов:
#define DIR_L  8
#define PWM_L  9
#define DIR_R  5
#define PWM_R  6

// Кнопка экстренной остановки
#define BUTTON_PIN 7   // INPUT_PULLUP (нажато = LOW)

// Светодиод "сердцебиение" при приёме пакета
#define LED_HB 13

// Последовательный порт от камеры:
#define RX_BAUD 115200

// Протокол от ESP32-CAM:
const uint8_t H1 = 0xAA;
const uint8_t H2 = 0x55;

// Тайминги/защита
const uint16_t BYTE_TIMEOUT_MS  = 30;    // межбайтовый таймаут парсера
const uint16_t LINK_TIMEOUT_MS  = 1000;  // если нет пакетов дольше — STOP
const uint16_t PRINT_EVERY_MS   = 100;   // периодический лог состояния

// Текущее состояние
int speedL = 0;   // -255..255 (то, что реально подали на ЛЕВОЕ колесо)
int speedR = 0;   // -255..255 (то, что реально подали на ПРАВОЕ колесо)
volatile uint32_t packetCount = 0;

// --- Управление каналом DIR/PWM (ваша логика) ---
void wheelChannel(uint8_t DIR, uint8_t PWM, int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    digitalWrite(DIR, LOW);
    analogWrite(PWM, speed);              // 0..255
  } else if (speed < 0) {
    digitalWrite(DIR, HIGH);
    analogWrite(PWM, 255 - abs(speed));   // инвертированный ШИМ
  } else {
    digitalWrite(DIR, LOW);
    analogWrite(PWM, 0);
  }
}
inline void leftWheel(int s)  { wheelChannel(DIR_L, PWM_L, s); }
inline void rightWheel(int s) { wheelChannel(DIR_R, PWM_R, s); }

void STOP() {
  wheelChannel(DIR_L, PWM_L, 0);
  wheelChannel(DIR_R, PWM_R, 0);
}

// --- Парсер пакетов 0xAA 0x55 L R на RX0 ---
enum { ST_H1, ST_H2, ST_L, ST_R } state = ST_H1;
uint32_t tLastByte = 0;
uint32_t tLastPacket = 0;

int8_t inL = 0, inR = 0;

void feedByte(uint8_t b) {
  switch (state) {
    case ST_H1:
      state = (b == H1) ? ST_H2 : ST_H1;
      break;
    case ST_H2:
      if (b == H2) state = ST_L;
      else state = (b == H1) ? ST_H2 : ST_H1;
      break;
    case ST_L:
      inL = (int8_t)b;
      state = ST_R;
      break;
    case ST_R:
      inR = (int8_t)b;
      state = ST_H1;
      tLastPacket = millis();
      packetCount++;

      digitalWrite(LED_HB, HIGH);  // короткий всплеск при приёме

      // масштабирование: *2 с насыщением
      long Lraw = (long)inL * 2;
      long Rraw = (long)inR * 2;
      int Lscaled = (int)constrain(Lraw, -255, 255);
      int Rscaled = (int)constrain(Rraw, -255, 255);

      // ВАЖНО: МЕНЯЕМ МЕСТАМИ — левому колесу даём R, правому — L
      speedL = Rscaled;
      speedR = Lscaled;

      leftWheel(speedL);
      rightWheel(speedR);
      break;
  }
}

void readSerialPackets() {
  if (millis() - tLastByte > BYTE_TIMEOUT_MS && state != ST_H1) {
    state = ST_H1;
  }
  while (Serial.available() > 0) {
    tLastByte = millis();
    uint8_t b = (uint8_t)Serial.read();
    feedByte(b);
  }
}

// --- Кнопка экстренной остановки (антидребезг) ---
byte lastRaw = HIGH, stable = HIGH;
uint32_t lastBounceAt = 0;
const uint16_t debounceMs = 30;

void handleButton() {
  byte raw = digitalRead(BUTTON_PIN);
  if (raw != lastRaw) { lastBounceAt = millis(); lastRaw = raw; }
  if (millis() - lastBounceAt > debounceMs) {
    if (stable != raw) {
      stable = raw;
      if (stable == LOW) {
        STOP(); speedL = speedR = 0;
        Serial.println(F("STOP (button)"));
      }
    }
  }
}

void setup() {
  pinMode(DIR_L, OUTPUT); pinMode(PWM_L, OUTPUT);
  pinMode(DIR_R, OUTPUT); pinMode(PWM_R, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_HB, OUTPUT);
  digitalWrite(LED_HB, LOW);

  STOP();

  Serial.begin(RX_BAUD); // RX0: сюда приходит поток от ESP32 (и отсюда печать в монитор)
  Serial.println(F("Robot RX ready @115200 (AA 55 L R, *2, L<->R swapped)"));
  Serial.println(F("Wiring: ESP32 TX->D0(RX), GND common"));

  tLastPacket = millis();
}

void loop() {
  readSerialPackets();
  handleButton();

  // погасить всплеск на LED через ~1 мс
  static uint32_t tLedOff = 0;
  if (digitalRead(LED_HB) == HIGH) {
    if (tLedOff == 0) tLedOff = millis();
    if (millis() - tLedOff >= 1) { digitalWrite(LED_HB, LOW); tLedOff = 0; }
  }

  // периодический лог (даже если значения не менялись)
  static uint32_t tLastPrint = 0;
  if (millis() - tLastPrint >= PRINT_EVERY_MS) {
    tLastPrint = millis();
    Serial.print(F("cnt=")); Serial.print(packetCount);
    Serial.print(F("  L=")); Serial.print(speedL);
    Serial.print(F("  R=")); Serial.println(speedR);
  }

  // Fail-safe: если давно нет пакетов — стоп
  if (millis() - tLastPacket > LINK_TIMEOUT_MS) {
    if (speedL != 0 || speedR != 0) {
      STOP(); speedL = speedR = 0;
      Serial.println(F("STOP (link timeout)"));
    }
  }
}
