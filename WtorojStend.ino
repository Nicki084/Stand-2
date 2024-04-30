// кнопки наше всё
// сделать UART те же 0 и 1 
// Использовать энеогосберегающую память запомненные позиции

#include "GyverStepper.h"
#include <EEPROM.h>
//#include "GyverPlanner.h"
//#include <SoftwareSerial.h>

#define sensorMax1 30
#define sensorMax2 31
#define sensorMax3 32
#define sensorMax4 33
#define catcher 2
#define buttonRotationPlusA1 11
#define buttonRotationMinusA1 12
#define buttonRotationPlusA2 13
#define buttonRotationMinusA2 14
#define buttonRotationPlusA3 15
#define buttonRotationMinusA3 16
#define buttonRotationPlusA4 17
#define buttonRotationMinusA4 18
#define UART_RX 19
#define UART_TX 20
#define EEPROM_POSITION1 0
#define EEPROM_POSITION2 1
#define EEPROM_POSITION3 2
#define EEPROM_POSITION4 3
//#define PowerPin пин, отслеживающий состояиние питания 

int pins[4][3] = {
  { 0, 1, 6 },
  { 2, 3, 7 },
  { 3, 4, 9 },
  { 11, 5, 10 },
};  // {{dir,en,step}}
long positionsDrivers[4] = { 0, 0, 0, 0 };

int counterCoordinates = 0;




int readyDrivers = 0;



GStepper<STEPPER2WIRE> stepper1(8192, pins[0][0], pins[0][0], pins[0][1]);
GStepper<STEPPER2WIRE> stepper2(8192, pins[1][3], pins[1][0], pins[1][1]);
GStepper<STEPPER2WIRE> stepper3(8192, pins[2][3], pins[2][0], pins[2][1]);
GStepper<STEPPER2WIRE> stepper4(8192, pins[3][3], pins[3][0], pins[3][1]);

//GPlanner< STEPPER2WIRE, 4 > planner;

void setup() {
  Serial.begin(115200);
  stepper1.setMaxSpeed(400);
  stepper1.setAcceleration(300);
  stepper1.enable();
  stepper1.setRunMode(FOLLOW_POS);
  stepper2.setMaxSpeed(400);
  stepper2.setAcceleration(300);
  stepper2.enable();
  stepper2.setRunMode(FOLLOW_POS);
  stepper3.setMaxSpeed(400);
  stepper3.setAcceleration(300);
  stepper3.enable();
  stepper3.setRunMode(FOLLOW_POS);
  stepper4.setMaxSpeed(400);
  stepper4.setAcceleration(300);
  stepper4.enable();
  stepper4.setRunMode(FOLLOW_POS);
  
  /*planner.addStepper(0,stepper1);
  planner.addStepper(1,stepper2);
  planner.addStepper(2,stepper3);
  planner.addStepper(3,stepper4);
  planner.setAcceleration(100);
  planner.setMaxSpeed(300);
  planner.reset();
  */
}
// Проверка ручного или автоматического режима
void loop() {
  if (startMove == true){
    rotationAuto();
  }
  else {
    rotation();
  }
  // Проверка на выключение системы
  if (digitalRead(PowerPin) == LOW){
    moweToHome();                                // Возвращение домой при выключении
  }
}




// Ручное
void rotation() {
  int buttonsActive = (digitalRead(buttonRotationPlusA1) == HIGH) ? 1 : 0
    + (digitalRead(buttonRotationMinusA1) == HIGH) ? 1 : 0
    + (digitalRead(buttonRotationPlusA2) == HIGH) ? 1 : 0
    + (digitalRead(buttonRotationMinusA2) == HIGH) ? 1 : 0 
    + (digitalRead(buttonRotationPlusA3) == HIGH) ? 1 : 0
    + (digitalRead(buttonRotationMinusA3) == HIGH) ? 1 : 0
    + (digitalRead(buttonRotationPlusA4) == HIGH) ? 1 : 0
    + (digitalRead(buttonRotationMinusA4) == HIGH) ? 1 : 0;
  if (buttonsActive > 1) {
    return;
  }
  if (digitalRead(buttonRotationPlusA1) == HIGH) {
    stepper1.reverse(false);
    stepper1.setTarget(1, RELATIVE);
  }
  if (digitalRead(buttonRotationMinusA1) == HIGH) {
    stepper1.reverse(true);
    stepper1.setTarget(1, RELATIVE);
  }
  if (digitalRead(buttonRotationPlusA2) == HIGH) {
    stepper2.reverse(false);
    stepper2.setTarget(1, RELATIVE);
  }
  if (digitalRead(buttonRotationMinusA2) == HIGH) {
    stepper2.reverse(true);
    stepper2.setTarget(1, RELATIVE);
  }
 if (digitalRead(buttonRotationPlusA3) == HIGH) {
    stepper3.reverse(false);
    stepper3.setTarget(1, RELATIVE);
  }
  if (digitalRead(buttonRotationMinusA3) == HIGH) {
    stepper3.reverse(true);
    stepper3.setTarget(1, RELATIVE);
  }
 if (digitalRead(buttonRotationPlusA4) == HIGH) {
    stepper4.reverse(false);
    stepper4.setTarget(1, RELATIVE);
  }
  if (digitalRead(buttonRotationMinusA4) == HIGH) {
    stepper4.reverse(true);
    stepper4.setTarget(1, RELATIVE);
  }
}
// Автоматическое
// Работа через запарашенные коорды
int rotationIndex = 0;
void rotationAUTO() {
  if (rotationIndex == 4){
    rotationIndex = 0;
  }
  if (rotationIndex == 0) {
      stepper1.setTarget(coordinates[0], ABSOLUTE);
      rotationIndex++;
  } else if (rotationIndex == 1 && !stepper1.tick() ) {
      stepper2.setTarget(coordinates[1], ABSOLUTE);
      rotationIndex++;
  } else if (rotationIndex == 2 && !stepper2.tick()) {
      stepper3.setTarget(coordinates[2], ABSOLUTE);
      rotationIndex++;
  } else if (rotationIndex == 3 && !stepper3.tick()) {
      stepper4.setTarget(coordinates[3], ABSOLUTE);
      rotationIndex++;
  }
}


// Заданная программа похвастаться
void Program(){

}

// Функция дома
void moveToHome() {
  while (true) {
    if (digitalRead(sensorMax1) == 0) {
        stepper1.reverse(true);
        stepper1.setTarget(1, RELATIVE);
    }
    else if (digitalRead(sensorMax2) == 0) {
        stepper2.reverse(true);
        stepper2.setTarget(1, RELATIVE);
    }
    else if (digitalRead(sensorMax3) == 0) {
        stepper3.reverse(true);
        stepper3.setTarget(1, RELATIVE);
    }
    else if (digitalRead(sensorMax1) == 1 && digitalRead(sensorMax2) == 1 && digitalRead(sensorMax3) == 1 ) {
      stepper1.setCurrent(0);
      stepper2.setCurrent(0);
      stepper3.setCurrent(0);
      break;
    }
  }
  // Сохраняем позиции дома в EEPROM
  for (int i = 0; i < 4; i++) {
    EEPROM.write(EEPROM_POSITION1 + 1, positionsDrivers[i]);
  }
 EEPROM.commit(); // Функция, которая гарантирует, что все изменения будут сохранены перед выключением питания
}



    // Число значений в массиве, который хотим получить
int parsingData[257] = new int [257];         // Массив численных значений после парсинга
boolean recievedFlag = false;
boolean getStarted = false;
byte index = 0;
String string_convert = "";

void parsing() {
  if (Serial.available() > 0) {
    char incomingByte = Serial.read();        // Обязательно ЧИТАЕМ входящий символ
    if (getStarted) {                         // Если приняли начальный символ (парсинг разрешён)
      if (incomingByte != ' ' && incomingByte != ';') {   // Если это не пробел И не конец
        string_convert += incomingByte;       // Складываем в строку
      } else {                                // Если это пробел или ; конец пакета
        parsingData[index] = string_convert.toInt();  // Преобразуем строку в int и кладём в массив
        string_convert = "";                  // Очищаем строку
        index++;                              // Переходим к парсингу следующего элемента массива
      }
    }
    if (incomingByte == '$') {                // Если это $
      getStarted = true;                      // Поднимаем флаг, что можно парсить
      index = 0;                              // Сбрасываем индекс
      string_convert = "";                    // Очищаем строку
    }
    if (incomingByte == ';') {                // Если таки приняли ; - конец парсинга
      getStarted = false;                     // Сброс
      recievedFlag = true;                    // Флаг на принятие
    }
  }
}

SoftwareSerial uart(UART_RX, UART_TX);        // Создание объекта UART
void setup() {
  uart.begin(9600);                           // Инициализация данных ... UART со скоростью 9600 бит/с (можно и другую скорость) между модулем и Arduino
  Serial.begin(9600);                         // Инициализация данных ... между Arduino и компьтером (нужно будет выбрать скорость)
}

void UART() {
 if (uart.available() ) {                     // Если есть данные в серийном порту
  char incommingByte = uart.read();           // Чтение первого байта из UART и сохранение в incommingByte
  parsing(incomingByte);                      // Передача символа в функцию парсинга и по идее дальше то же самое, что и выше ╮(￣～￣)╭
  if (getStarted) {                           // Если приняли начальный символ (парсинг разрешён)
      if (incomingByte != ' ' && incomingByte != ';') {   // Если это не пробел И не конец
        string_convert += incomingByte;       // Складываем в строку
      } else {                                // Если это пробел или ; конец пакета
        parsingData[index] = string_convert.toInt();  // Преобразуем строку в int и кладём в массив
        string_convert = "";                  // Очищаем строку
        index++;                              // Переходим к парсингу следующего элемента массива
      }
    }
    if (incomingByte == '$') {                // Если это $
      getStarted = true;                      // Поднимаем флаг, что можно парсить
      index = 0;                              // Сбрасываем индекс
      string_convert = "";                    // Очищаем строку
    }
    if (incomingByte == ';') {                // Если таки приняли ; - конец парсинга
      getStarted = false;                     // Сброс
      recievedFlag = true;                    // Флаг на принятие
    }
  }
  if (uart.available()) {
    Serial.write(uart.read());                // Передача данных от модуля через Arduino к компьютеру
  }
  if (Serial.available()){
    uart.write(Serial.read());                // Передача данных от компьютера через Arduino к модулю
  }
}
