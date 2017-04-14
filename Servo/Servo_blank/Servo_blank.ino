#define sleep_enable 1   // спящий режим (0 - выключить, 1 - включить)
#define sleep_time 10000 // время, через которое замок уйдёт в сон после просыпания

#include <EEPROMex.h>   // библиотека для работы со внутренней памятью ардуино
#include <LowPower.h>   // библиотека сна

byte set_pass_btn = 4; // кнопка смены пароля на 4 пин
boolean set_pass_btn_flag;
byte close_btn = 5;     // концевик двери (кнопка закрытия)
boolean door_opened, door_closed;
volatile unsigned long awake_timer;
volatile boolean inside_open_flag;
unsigned long open_timer;

void setup() {
  Serial.begin(9600); // открыть порт для отладки
  // подтянуть все кнопки
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  pinMode(set_pass_btn, INPUT_PULLUP);
  pinMode(close_btn, INPUT_PULLUP);

  // прерывания: 2 пин на внутреннюю кнопку
  attachInterrupt(0, inside_open, FALLING);
  // 3 пин на внешнюю (проснуться)
  attachInterrupt(1, wake_up, FALLING);

}

void loop() {
  // если дверь была открыта и нажат концевик
  if (!digitalRead(close_btn) && door_opened) {
    close_door();     // закрыть дверь
  }

  // если дверь закрыта и нажата кнопка открытия
  if (inside_open_flag && door_closed) {
    inside_open_flag = 0;
    open_door(); // команда для открытия двери
    if (sleep_enable) LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF); // спать. mode POWER_OFF, АЦП выкл
  }

  // если разрешён сон и прошло больше времени чем по таймеру
  if (sleep_enable && millis() - awake_timer > sleep_time) {
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF); // спать. mode POWER_OFF, АЦП выкл
  }

  if (!digitalRead(set_pass_btn) && !set_pass_btn_flag) {
    set_pass_btn_flag = 1;    
  }
  if (digitalRead(set_pass_btn) && set_pass_btn_flag) {
    set_pass_btn_flag = 0;
  }
}

// набор команд для открытия двери
void open_door() {
  // ОТКРЫТЬ

  // если не открыли до таймера, считать закрытой
  // если открыли до таймера, считать открытой
  open_timer = millis();
  door_closed = 1;    // считать дверь закрытой
  door_opened = 0;    // считать дверь закрытой
  while (millis() - open_timer < 4000) {
    if (digitalRead(close_btn)) {  // если кнопка отпущена
    door_opened = 1;  // считать дверь открытой
    door_closed = 0;  // считать дверь открытой
    break;
    }
  }
}

// набор команд для закрытия двери
void close_door() {
  door_closed = 1;
  door_opened = 0;
}

void inside_open() {
  inside_open_flag = 1;
}
void wake_up() {
  awake_timer = millis();
}
