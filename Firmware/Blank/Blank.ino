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
#define lock_type 0          // 0 - сервопривод, 1 - щеколда, 2 - привод автомобильный (в разработке)

#define tail_button 0        // 1 - используется концевик на закрытие, 0 - не используется
#define sleep_time 10000     // время на ввод пароля после просыпания (если не начать вводить - уснёт)

#define sleep_enable 1       // спящий режим с кнопкой проснуться (0 - выключить, 1 - включить)
#define wake_button 0        // 1 - просыпаться только по наружной кнопке, 0 - просыпаться периодически и проверять СОБЫТИЕ
boolean battery_monitor = 1; // измерение заряда акума (защита от переразряда)
#define bat_low 3000         // напряжение акума в МИЛЛИВОЛЬТАХ, при котором сработает защита
boolean open_bat_low = 1;    // 1 - открыть дверь, если акум разряжен

#define vol_calibration 0    // калибровка вольтметра (если работа от АКБ) 1 - включить, 0 - выключить

//--------------СЕРВО--------------
#define servoMin 40         // угол сервы для положения "замок открыт", подбирается экспериментально
#define servoMax 170        // угол сервы для положения "замок закрыт", подбирается экспериментально
//--------------СЕРВО--------------

//--------------ЩЕКОЛДА--------------
boolean latch_inverse = 0; // 0 - если у вас реле высокого уровня или МОСФЕТ, 1 - если у вас реле низкого уровня
int latch_time = 1000;     // время (в миллисекундах), которое ток будет подаваться на защёлку для открытия
//--------------ЩЕКОЛДА--------------

//--------------ПРИВОД--------------
#define gear_inv 1         // инвертировать привод
//--------------ПРИВОД--------------

//-------------------------------НАСТРОЙКИ-----------------------------------

// автоматический выбор close_button: 0 - если используем захлопывающуюся щеколду, 1 если серво или привод замка
#if lock_type == 1
#define close_button 0
#else
#define close_button 1
#endif

//----------------------БИБЛИОТЕКИ------------------------
#include <EEPROMex.h>   // библиотека для работы со внутренней памятью ардуино
#include <LowPower.h>   // библиотека сна
//----------------------БИБЛИОТЕКИ------------------------

//---АВТОМАТИЧСЕКИЕ НАСТРОЙКИ В ЗАВИСИМОСТИ ОТ ТИПА ЗАМКА-----
#if lock_type == 0
//-----------------СЕРВО----------------
#include <Servo.h>      //используем библиотеку для работы с сервоприводом
#define servo_pin A3
Servo servo;            //объявляем переменную servo типа Servo
//-----------------СЕРВО----------------
#elif lock_type == 1
//-----------------ЩЕКОЛДА----------------
#define latch_pin A4
//-----------------ЩЕКОЛДА----------------
#elif lock_type == 2
//-----------------ПРИВОД----------------
#define relay1 A2        // пин реле 1 подключен к А2
#define relay2 A1        // пин реле 2 подключен к А1
#define gear_delay 400   // время (в мс), которое ток подаётся на привод (время открытия/закрытия)
//-----------------ПРИВОД----------------
#endif
//---АВТОМАТИЧСЕКИЕ НАСТРОЙКИ В ЗАВИСИМОСТИ ОТ ТИПА ЗАМКА-----

#define LEDgrn A0                // красный светодиод на А0
#define LEDred A1                // зелёный светодиод на А1
#define LEDgnd A2                // земля светодиодов на А2
#define set_pass_btn 4           // кнопка смены пароля на 4 пин
#define tail_pin 5               // кнопка концевика
float my_vcc_const = 1.1;        // начальное значение константы вольтметра

boolean set_pass_btn_flag;       // флажок кнопки смены пароля
boolean batteryOK = true;        // дверь можно закрыть, если акум заряжен
boolean set_access_flag;         // флажок режима смены пароля/ключа
volatile boolean door_state;     // состояние двери (1 - открыто, 0 - закрыто)
volatile unsigned long awake_timer, auto_awake_timer;
volatile boolean inside_btn_flag, close_flag;
volatile byte sleep_count;
boolean wake_event = false, wait_for_event = true, wakeUP_procedure_flag = false;

void setup() {
  Serial.begin(9600); // открыть порт для отладки
  if (vol_calibration) calibration();  // калибровка, если разрешена
  my_vcc_const = EEPROM.readFloat(1000);

  // подтянуть все кнопки
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  pinMode(set_pass_btn, INPUT_PULLUP);
  pinMode(tail_pin, INPUT_PULLUP);

  pinMode(LEDred, OUTPUT);
  pinMode(LEDgrn, OUTPUT);
  pinMode(LEDgnd, OUTPUT);
  digitalWrite(LEDred, 0);
  digitalWrite(LEDgrn, 0);
  digitalWrite(LEDgnd, 0);

  // прерывания: 2 пин на внутреннюю кнопку
  attachInterrupt(0, inside_btn, FALLING);
  // 3 пин на внешнюю кнопку
  attachInterrupt(1, outside_btn, FALLING);

  //---АВТОМАТИЧСЕКИЕ НАСТРОЙКИ В ЗАВИСИМОСТИ ОТ ТИПА ЗАМКА-----
#if lock_type == 1                        // если щеколда
  pinMode(latch_pin, OUTPUT);
  digitalWrite(latch_pin, latch_inverse);
#elif lock_type == 2                      // если привод (2 реле)
  pinMode(relay1, OUTPUT);
  pinMode(relay2, OUTPUT);
  digitalWrite(relay1, 1);
  digitalWrite(relay2, 1);
#endif
  //---АВТОМАТИЧСЕКИЕ НАСТРОЙКИ В ЗАВИСИМОСТИ ОТ ТИПА ЗАМКА-----
}

// набор команд для открытия двери
void open_door() {
  if (batteryOK) {                        // если акум заряжен
    digitalWrite(LEDgrn, 1);              // зелёный свет
    digitalWrite(LEDred, 0);
    //--------ОТКРЫТЬ---------
#if lock_type == 0
    servo.attach(servo_pin);              // привязываем привод к порту
    servo.write(servoMin);                // поворачиваем серву
    delay(500);                           // ждём поворот
    servo.detach();                       // отвязываем привод
#elif lock_type == 1                      // если щеколда
    digitalWrite(A4, !latch_inverse);     // подать ток на защёлку
    delay(latch_time);                    // подождать
    digitalWrite(A4, latch_inverse);      // прекратить подачу тока
#elif lock_type == 2                      // если привод замка
    if (gear_inv) {                       // перещёлкнуть реле, подождать, все дела
      digitalWrite(relay1, 0);
      delay(gear_delay);
      digitalWrite(relay1, 1);
    } else {
      digitalWrite(relay2, 0);
      delay(gear_delay);
      digitalWrite(relay2, 1);
    }
#endif
    //--------ОТКРЫТЬ---------
    Serial.println("Open door");
    if (!close_button) {
      door_state = 0; // считать дверь закрытой (сама захлопнется)
      digitalWrite(LEDred, 0);            // выключить свет
      digitalWrite(LEDgrn, 0);
    }
    else door_state = 1;                  // считать дверь открытой
  } else {                                // если акум разряжен
    for (int i = 0; i < 3; i++) {         // мигать красным светодиодом
      digitalWrite(LEDred, 1);
      delay(500);
      digitalWrite(LEDred, 0);
      delay(500);
    }
  }
  if (sleep_enable) sleep_mode();
}

// набор команд для закрытия двери
void close_door() {
  if (batteryOK) {                       // если акум заряжен
    digitalWrite(LEDred, 1);             // зелёный свет
    //--------ЗАКРЫТЬ---------
#if lock_type == 0                       // если серво
    servo.attach(servo_pin);             // привязываем привод к порту
    servo.write(servoMax);               // поворачиваем серву
    delay(500);                          // ждём поворот
    servo.detach();                      // отвязываем привод
#elif lock_type == 1                     // если щеколда
    // ничего не делать, щеколда же =)
#elif lock_type == 2                     // если привод замка
    if (gear_inv) {                      // перещёлкнуть реле, подождать, все дела
      digitalWrite(relay2, 0);
      delay(gear_delay);
      digitalWrite(relay2, 1);
    } else {
      digitalWrite(relay1, 0);
      delay(gear_delay);
      digitalWrite(relay1, 1);
    }
#endif
    //--------ЗАКРЫТЬ---------
    Serial.println("Close door");
    door_state = 0;
    delay(500);
    digitalWrite(LEDred, 0);              // выключить свет
    digitalWrite(LEDgrn, 0);
  } else {                                // если акум разряжен
    for (int i = 0; i < 3; i++) {         // мигать красным светодиодом
      digitalWrite(LEDred, 1);
      delay(500);
      digitalWrite(LEDred, 0);
      delay(500);
    }
  }
  if (sleep_enable) sleep_mode();
}

void wakeUP_procedure() {

}
void sleep_procedure() {

}

void loop() {
  // ------ ЖДЁМ СОБЫТИЯ, ЧТОБЫ ПРОСНУТЬСЯ ------
  if (wait_for_event && !wake_button) {
    // тут появляется wake_event
  }
  // ------ ЖДЁМ СОБЫТИЯ, ЧТОБЫ ПРОСНУТЬСЯ ------

  // ---- ВЫПОЛНИТЬ КАК ПРОСНЁМСЯ -----
  if (wakeUP_procedure_flag) {
    wakeUP_procedure();
    wakeUP_procedure_flag = false;
  }
  // ---- ВЫПОЛНИТЬ КАК ПРОСНЁМСЯ -----

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
  if (inside_btn_flag && !door_state) {
    inside_btn_flag = 0;
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
  // отработка нажатия кнопки смены пароля

  // если разрешён сон и прошло больше времени, чем по таймеру
  if (millis() - awake_timer > sleep_time && !set_access_flag) {
    if (!door_state) {          // если дверь закрыта, выключить светодиоды
      digitalWrite(LEDred, 0);
      digitalWrite(LEDgrn, 0);
    }
    if (sleep_enable) {
      Serial.println("Sleep (idle)");
      sleep_mode();
    }
  }
}

// отработка прерывания нажатия изнутри
void inside_btn() {
  auto_awake_timer = millis();  // сбросить таймер
  if (!door_state)              // если дверь ЗАКРЫТА
    inside_btn_flag = 1;        // флажок на открытие
  else                          // если дверь ОТКРЫТА
    close_flag = 1;             // флажок на закрытие
  sleep_count = 0;              // обнулить таймер вольтметра
}

// отработка прерывания нажатия снаружи
void outside_btn() {
  auto_awake_timer = millis();
  if (door_state) {              // если дверь ОТКРЫТА
    close_flag = 1;              // флажок на закрытие
  } else {                       // если нет
    Serial.println("Wake up");   // проснуться и обнулить таймер
    digitalWrite(LEDred, 1);
    digitalWrite(LEDgrn, 1);
    wakeUP_procedure_flag = true;     // выполнить 1 раз когда проснулся
    awake_timer = millis();
  }
  sleep_count = 0;               // обнулить таймер вольтметра
}

// режим сна (зависит от того, измеряем мы напряжение акума или нет)
void sleep_mode() {
  sleep_procedure();
  if (!batteryOK) digitalWrite(LEDgrn, 0);
  Serial.println("Sleep");
  delay(50);
  if (tail_button && door_state && close_button) {
    LowPower.powerDown(SLEEP_500MS, ADC_OFF, BOD_OFF);  // спать 500 мс. mode POWER_OFF, АЦП выкл
    awake_timer = millis() + sleep_time;  // КОСТЫЛЬ БЛЯ
    if (!digitalRead(tail_pin)) {
      Serial.println("Tail!");
      close_door();
    }
  } else {
    if (sleep_enable && wake_button && !battery_monitor)   // если ничего не надо
      LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF); // спать. mode POWER_OFF, АЦП выкл
    if (sleep_enable && wake_button && battery_monitor) {  // если просто исзеряем акб
      LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);      // спать 8 сек. mode POWER_OFF, АЦП выкл
      awake_timer = millis() + sleep_time;  // КОСТЫЛЬ БЛЯ
      battery_m();
    }
    if (sleep_enable && !wake_button) {                    // если ловим событие + акб измеряем
      LowPower.powerDown(SLEEP_4S, ADC_OFF, BOD_OFF);      // спать 4 сек. mode POWER_OFF, АЦП выкл
      awake_timer = millis() + sleep_time;  // КОСТЫЛЬ БЛЯ
      event_m();
      if (battery_monitor) battery_m();                    // если хотим ещё акб измерять, измеряем
    }
  }
}
void battery_m() {
  sleep_count++;
  if (sleep_count > 75) {             // если прошло ~5 минут
    sleep_count = 0;                  // сбросить счётчик
    if (readVcc() < bat_low) {        // если акум разряжен
      Serial.println("Low battery");
      wait_for_event = false;         // перестать ждать СОБЫТИЕ
      battery_monitor = false;        // перестать мониторить АКБ
      if (open_bat_low) open_door();  // открыть дверь (и уснуть)
      batteryOK = false;              // запретить закрытие и открытие двери
    }
  }
}
void event_m() {
  Serial.println("Event awake");
  if (wake_event) {
    Serial.println("Wake up");          // проснуться и обнулить таймер
    digitalWrite(LEDred, 1);
    digitalWrite(LEDgrn, 1);
    wake_event = false;
    wakeUP_procedure_flag = true;       // выполнить 1 раз когда проснулся
    awake_timer = millis();
    sleep_count = 0;                    // обнулить таймер вольтметра
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
/*
  Дверь закрыта, нажато СНАРУЖИ - проснуться, ждать ввод пароля
  Дверь закрыта, нажато ВНУТРИ - открыть
  Дверь открыта, нажато СНАРУЖИ - закрыть
  Дверь открыта, нажато ВНУТРИ - закрыть
  Дверь открыта, нажат КОНЦЕВИК - закрыть

  Просыпаться каждые несколько секунд, следить за СОБЫТИЕМ
  Каждые несколько минут следить за напряжением акума
  Если акум разряжен
  - открыть дверь (опционально)
  - запретить дальнейшее открытие и закрытие
  - при нажатии на кнопки мигать красным светодиодом
  - перестать следить за СОБЫТИЕМ
*/
