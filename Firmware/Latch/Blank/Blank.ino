/*
  ВНИМАНИЕ! ПУТЬ К ПАПКЕ СО СКЕТЧЕМ НЕ ДОЛЖЕН СОДЕРЖАТЬ РУССКИХ СИМВОЛОВ
  ВО ИЗБЕЖАНИЕ ПРОБЛЕМ ПОЛОЖИТЕ ПАПКУ В КОРЕНЬ ДИСКА С

  Внимание! При первом запуске initial_calibration должен быть равен 1 (строка №17)
  При подключении и открытии монитора порта будет запущен процесс калибровки.
  Вам нужно при помощи вольтметра измерить напряжение на пинах 5V и GND,
  затем отправить его в монитор В МИЛЛИВОЛЬТАХ, т.е. если на вольтметре 4.56
  то отправить примерно 4560. После этого изменить initial_calibration на 0
  и заново прошить Arduino.
  Если хотите пропустить процесс калибровки, то введите то же самое напряжение,
  что было показано вам при калибровке (real VCC). И снова прошейте код.
*/

//-------------------------------НАСТРОЙКИ-----------------------------------
#define sleep_time 10000   // время на ввод пароля (если не начать вводить - уснёт)

#define sleep_enable 1     // спящий режим с кнопкой проснуться (0 - выключить, 1 - включить)
#define vol_calibration 0  // калибровка вольтметра (если работа от АКБ) 1 - включить, 0 - выключить
boolean open_bat_low = 1;  // 1 - открыть дверь, если акум разряжен (просыпаемся каждые 5 мин и проверяем), 0 - нет
#define bat_low 2800       // напряжение акума в МИЛЛИВОЛЬТАХ, при котором дверь откроется, и система уйдёт в сон

boolean latch_inverse = 0; // 0 - если у вас реле высокого уровня или МОСФЕТ, 1 - если у вас реле низкого уровня
int latch_time = 1000;     // время (в миллисекундах), которое ток будет подаваться на защёлку для открытия
//-------------------------------НАСТРОЙКИ-----------------------------------

#define close_button 0     // 0 если используем захлопывающуюся щеколду, 1 если серво или привод замка (кнопка закрытия или концевик)

//----------------------БИБЛИОТЕКИ------------------------
#include <EEPROMex.h>   // библиотека для работы со внутренней памятью ардуино
#include <LowPower.h>   // библиотека сна
//----------------------БИБЛИОТЕКИ------------------------


#define set_pass_btn 4           // кнопка смены пароля на 4 пин
boolean set_pass_btn_flag;       // флажок кнопки смены пароля
boolean batteryOK = true;        // дверь можно закрыть, если акум заряжен
boolean set_access_flag;         // флажок режима смены пароля/ключа
volatile boolean door_state;     // состояние двери (1 - открыто, 0 - закрыто)
float my_vcc_const = 1.1;        // начальное значение константы вольтметра
volatile unsigned long awake_timer, auto_awake_timer;
volatile boolean inside_open_flag, close_flag;
byte sleep_count;

void setup() {
  Serial.begin(9600); // открыть порт для отладки
  if (vol_calibration) calibration();  // калибровка, если разрешена

  // подтянуть все кнопки
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  pinMode(set_pass_btn, INPUT_PULLUP);
  pinMode(A3, OUTPUT);
  digitalWrite(A4, latch_inverse);

  //----читаем из памяти-----
  my_vcc_const = EEPROM.readFloat(1000);
  //----читаем из памяти-----

  // прерывания: 2 пин на внутреннюю кнопку ОТКРЫТИЯ
  attachInterrupt(0, inside_open, FALLING);
  // 3 пин на внешнюю ЗАКРЫТИЯ (она же ПРОСНУТЬСЯ) и концевик ЗАКРЫТИЯ двери
  attachInterrupt(1, wake_or_close, FALLING);
}

// набор команд для открытия двери
void open_door() {
  //--------ОТКРЫТЬ---------
  digitalWrite(A4, !latch_inverse);  // подать ток на защёлку
  delay(latch_time);                 // подождать 
  digitalWrite(A4, latch_inverse);   // прекратить подачу тока
  //--------ОТКРЫТЬ---------
  // если не открыли до таймера, считать закрытой
  // если открыли до таймера, считать открытой
  Serial.println("Open door");
  if (!close_button) door_state = 0;    // считать дверь закрытой (сама захлопнется)
  else door_state = 1;                  // считать дверь открытой
  Serial.println("Sleep");
  sleep_mode();
}

// набор команд для закрытия двери
void close_door() {
  if (batteryOK) {
    //--------ЗАКРЫТЬ---------

    //--------ЗАКРЫТЬ---------
    Serial.println("Close door");
    door_state = 0;
  }
  Serial.println("Sleep");
  sleep_mode();
}

void loop() {
  if (set_access_flag) {
    //------------СМЕНА ПАРОЛЯ / КЛЮЧА ДОСТУПА--------------

    //------------СМЕНА ПАРОЛЯ / КЛЮЧА ДОСТУПА--------------
    Serial.println("Pass change");
    set_access_flag = 0;
  }

  // если дверь была открыта и нажат концевик (кнопка закрыть)
  if (close_flag) {
    close_flag = 0;
    close_door();     // закрыть дверь
  }

  // если дверь закрыта и нажата кнопка открытия изнутри
  if (inside_open_flag && !door_state) {
    inside_open_flag = 0;
    open_door(); // команда для открытия двери
  }

  // отработка нажатия кнопки смены пароля
  if (!digitalRead(set_pass_btn) && !set_pass_btn_flag) {
    awake_timer = millis();
    set_pass_btn_flag = 1;
  }
  if (digitalRead(set_pass_btn) && set_pass_btn_flag) {
    set_pass_btn_flag = 0;
    set_access_flag = 1;
  }

  // если разрешён сон и прошло больше времени, чем по таймеру
  if (sleep_enable && millis() - awake_timer > sleep_time && !set_access_flag) {
    Serial.println("Sleep (idle)");
    sleep_mode();
  }
}

// отработка прерывания открытия изнутри
void inside_open() {
  auto_awake_timer = millis();  // сбросить таймер
  if (!door_state)
    inside_open_flag = 1;
  sleep_count = 0;
}

// отработка прерывания кнопки "проснуться" (она же кнопка "закрыть")
// если нажали когда дверь открыта - дверь закроется
// если нажали когда дверь закрыта - система "проснётся"
void wake_or_close() {
  auto_awake_timer = millis();
  if (door_state) {
    close_flag = 1;
  } else {
    Serial.println("Wake up");
    awake_timer = millis();
  }
  sleep_count = 0;
}

// режим сна (зависит от того, измеряем мы напряжение акума или нет)
void sleep_mode() {
  delay(100);
  if (sleep_enable && !open_bat_low) LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF); // спать. mode POWER_OFF, АЦП выкл
  if (sleep_enable && open_bat_low) {
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);  // спать 8 сек. mode POWER_OFF, АЦП выкл
    Serial.println("Battery awake");
    sleep_count++;
    if (sleep_count > 75) {       // если прошло 10 минут
      sleep_count = 0;            // сбросить счётчик
      // измеряем напряжение
      if (readVcc() < bat_low) {  // если акум разряжен
        Serial.println("Low battery");
        open_bat_low = false;     // поставить режим "вечного сна"
        batteryOK = false;        // запретить закрытие двери (чтоыб понять, что акум сдох)
        open_door();              // открыть дверь (и уснуть)
      }
    }
  }
}

void calibration() {
  //--------калибровка----------
  my_vcc_const = 1.1;                                           // начальаня константа калибровки
  Serial.print("Real VCC is: "); Serial.println(readVcc());     // общаемся с пользователем
  Serial.println("Write your VCC (in millivolts)");
  while (Serial.available() == 0); int Vcc = Serial.parseInt(); // напряжение от пользователя
  float real_const = (float)1.1 * Vcc / readVcc();              // расчёт константы
  Serial.print("New voltage constant: "); Serial.println(real_const, 3);
  EEPROM.writeFloat(1000, real_const);                          // запись в EEPROM
  while (1);                                                    // уйти в бесконечный цикл
  //------конец калибровки-------
}

long readVcc() { //функция чтения внутреннего опорного напряжения, универсальная (для всех ардуин)
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
  ADMUX = _BV(MUX5) | _BV(MUX0);
#elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
  ADMUX = _BV(MUX3) | _BV(MUX2);
#else
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA, ADSC)); // measuring
  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH; // unlocks both
  long result = (high << 8) | low;

  result = my_vcc_const * 1023 * 1000 / result; // расчёт реального VCC
  return result; // возвращает VCC
}
