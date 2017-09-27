#include "Keypad.h"  //библиотека клавиатуры
#include <EEPROMex.h>

unsigned long pass_timer;
char key;
String str_pass = "";
byte pass_lenght, j;
unsigned long int_pass; // 10 знаков максимум!!
char keys[4][3] = {
  {'1', '2', '3'},
  {'4', '5', '6'},
  {'7', '8', '9'},
  {'*', '0', '#'}
};
byte rowPins[] = {12, 11, 10, 9};     // Подключены строки (4 пина)
byte colPins[] = {8, 7, 6};          // Подключены столбцы (3 пина)
Keypad keypad = Keypad( makeKeymap(keys), rowPins, colPins, 4, 3 ); //инициализировать клавиатуру

void setup() {
  Serial.begin(9600);
  int_pass = EEPROM.readLong(0);     // вспоминаем пароль из памяти
  str_pass = String(int_pass, DEC);  // переводим в строчный тип
  pass_lenght = str_pass.length();   // получаем длину пароля

  str_pass = "";                       // сброс пароля (пустая строка)
  pass_timer = millis();               // сброс таймера ввода пароля
  while (1) {                          // бесконечный цикл
    key = keypad.getKey();             // обработка нажатия
    if (key != NO_KEY) {               // если была нажата
      pass_timer = millis();           // сбросить таймер
      if (key == '*') {                // если нажата *
        int_pass = str_pass.toInt();   // перевести в число
        EEPROM.writeLong(0, int_pass); // записать в память
        pass_lenght = str_pass.length();   // получаем длину пароля
        break;                         // выйти из цикла
      }
      else if (key == '#') {           // если нажата #
        str_pass = "";                 // начать ввод сначала
      }
      else {                           // если * не нажата
        str_pass += key;               // прибавить нажатую цифру к паролю
      }
    }
    if (millis() - pass_timer > 10000) {     // если сработал таймер
      str_pass = String(int_pass, DEC);      // сбросить ввод и выйти из цикла
      break;
    }
  }
}

void pass_check() {
  key = keypad.getKey();             // обработка нажатия
  if (key == '*') {                  // если была нажата *
    pass_timer = millis();           // сбросить таймер
    j = 0;
    while (1) {                      // бесконечный цикл ввода пароля
      key = keypad.getKey();             // обработка нажатия
      if (key != NO_KEY) {               // если была нажата
        pass_timer = millis();           // сбросить таймер
        if (key == str_pass[j]) {        // если новая введённая цифра совпала с цифрой пароля
          j++;                           // прибавить счётчик
        } else {                         // если нет
          j = 0;                         // начать с начала
        }
        if (j == pass_lenght) {          // если были введены все правильные цифры пароля
          Serial.println("Success");
          break;                         // выйти из цикла
        }
        if (key == '#') {                // если нажата #
          break;                         // выйти из цикла
        }
      }
      if (millis() - pass_timer > 10000) {    // если сработал таймер
        break;                                // выйти из цикла
      }
    }
  }
}

void loop() {
  pass_check();

}
