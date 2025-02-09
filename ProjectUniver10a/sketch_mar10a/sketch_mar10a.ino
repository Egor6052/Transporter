#include <TM1637Display.h>       // Бібліотека для виводу інформації на дисплей

const int GreenLed1 = 11;        // Змінні для світлових індикаторів
const int YellowLed1 = 5;
const int RedLed1 = 12;

const int LedLeft = 6;
const int LedRight = 7;

const int Button = 4;            // Кнопка включения
const int ButtonLeft = 3;        // Кнопка влево
const int ButtonRight = 2;       // Кнопка вправо

const int EmergencySignal = 8;   // Экстренное выключение

const int Buzzer = 13;           // Сигнализация
const int Engine_IN1 = 9;        // Пин IN1 подключен к пину 9
const int Engine_IN2 = 10;       // Пин IN2 подключен к пину 10

bool buttonState = HIGH;         // Стан кнопки. Змінні для антидребезгого захисту
bool lastButtonState = HIGH;
bool motorState = false;         // Состояние мотора (включен или выключен)
bool LeftState = false;          // Состояние лампочки LedLeft
bool RightState = false;         // Состояние лампочки LedRight

unsigned long lastDebounceTime = 0;   // Змінні для урахування останнього часу нажаття.
unsigned long debounceDelay = 50;
bool systemActive = false;            // Флаг, указывающий, активна ли система
bool systemButtonPressed = false;     // Флаг, указывающий, была ли нажата кнопка включения

const int ButtonPlus = A3;        // Змінні для збульшення чи зменьшення об'єктів на конвеєрі.
const int ButtonMinus = A2;
int counter = 0;

const int CLK = A1;  // Подключите CLK вашего дисплея к аналоговому пину A1
const int DIO = A0;  // Подключите DIO вашего дисплея к аналоговому пину A0

TM1637Display display(CLK, DIO);  // Создайте объект дисплея с аналоговыми пинами

void setup() {
  // Налаштування піна як вихід
  pinMode(Engine_IN1, OUTPUT);
  pinMode(Engine_IN2, OUTPUT);
  pinMode(Buzzer, OUTPUT);
  pinMode(GreenLed1, OUTPUT);
  pinMode(YellowLed1, OUTPUT);
  pinMode(RedLed1, OUTPUT);
  pinMode(LedLeft, OUTPUT);
  pinMode(LedRight, OUTPUT);
  
  // Налаштування піна як вхід з внутрішнім підтягувачем до живлення
  pinMode(Button, INPUT_PULLUP);
  pinMode(ButtonLeft, INPUT_PULLUP);
  pinMode(ButtonRight, INPUT_PULLUP);
  
  // Налаштування піна як вхід
  pinMode(EmergencySignal, INPUT);

  // Налаштування піна як вхід з внутрішнім підтягувачем до живлення
  pinMode(ButtonPlus, INPUT_PULLUP); // Устанавливаем кнопку плюс как вход с подтяжкой к питанию
  pinMode(ButtonMinus, INPUT_PULLUP); // Устанавливаем кнопку минус как вход с подтяжкой к питанию
  
  // Налаштування яскравості дисплея
  display.setBrightness(0x0a);
}

// Стан кнопки екстренного вимикання
void loop() {
  int emergencySignal = digitalRead(EmergencySignal);

  if (emergencySignal == HIGH) {
    NormalMode();
  } else {
    EmergencyMode();
  }
}

void NormalMode() {
  digitalWrite(RedLed1, LOW);  // Выключаем красный индикатор

  int reading = digitalRead(Button);
  
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }
// перевірка для антидребезгу
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;

      if (buttonState == LOW) {
        tone(Buzzer, 1000);
        delay(1000);
        noTone(Buzzer);

        lastDebounceTime = millis();

        if (systemActive) {
          display.showNumberDec(0000,false); 
          systemActive = false; // Если система активна, повторное нажатие отключает её
        } else {
          display.showNumberDec(0000,false); 
          systemActive = true;  // Если система неактивна, первое нажатие включает её
        }
      }
    }
  }

  lastButtonState = reading;

  if (systemActive) {            // Проверяем, активна ли система перед обработкой кнопок
    ButtonsCounter();

    digitalWrite(YellowLed1, HIGH); // Жовтий індикатор робимо активним (робота транспортеру)
    digitalWrite(GreenLed1, LOW);   // Зелений неактивним (Безопасний режим)
    ButtonLeftMode();            // Класс для задання напрямку на ліво
    ButtonRightMode();           // Класс для задання напрямку направо
  } 
  // Якщо система не активна
    else {
      ButtonsCounter();
      
      digitalWrite(LedLeft, LOW);
      digitalWrite(LedRight, LOW);
      digitalWrite(Engine_IN1, LOW);
      digitalWrite(Engine_IN2, LOW);
      digitalWrite(YellowLed1, LOW);
      digitalWrite(GreenLed1, HIGH);
    }
}


void ButtonLeftMode() {
  // Перевірка для антидребезгу
  if (digitalRead(ButtonLeft) == LOW && (millis() - lastDebounceTime > debounceDelay)) {
    tone(Buzzer, 1000);
    delay(1000);                         // Звуковий сигнал на 1 секунду
    noTone(Buzzer);

    LeftState = !LeftState;
    motorState = !motorState;

    if (LeftState) {                     // Якщо напрямок - вліво
      if (RightState = HIGH) {
        RightState = !RightState;        // то блокуємо напрямок вправо
      }
      
      if (motorState) {                  // Задаємо послідовність сигналів на мотор
        digitalWrite(Engine_IN1, LOW);
        digitalWrite(Engine_IN2, HIGH);
      } else {
        digitalWrite(Engine_IN1, LOW);
        digitalWrite(Engine_IN2, LOW);
      }
      digitalWrite(LedLeft, HIGH);
      digitalWrite(LedRight, LOW);
      digitalWrite(YellowLed1, HIGH);
      digitalWrite(GreenLed1, LOW);
    } else {
     
      digitalWrite(LedLeft, LOW);
      digitalWrite(YellowLed1, LOW);
      digitalWrite(GreenLed1, HIGH);
    }

    lastDebounceTime = millis();
  }
}

void ButtonRightMode() {
  if (digitalRead(ButtonRight) == LOW && (millis() - lastDebounceTime > debounceDelay)) {
    tone(Buzzer, 1000);
    delay(1000);              // Звуковий сигнал на 1 секунду
    noTone(Buzzer);

    RightState = !RightState;
    motorState = !motorState;

    if (RightState) {                     // Якщо напрямок - на право
      if (LeftState = HIGH) {
        LeftState = !LeftState;           // то блокуємо напрямок на ліво
      }

      if (motorState) {                   // Задаємо послідовність сигналів на мотор
        digitalWrite(Engine_IN1, HIGH);
        digitalWrite(Engine_IN2, LOW);
      } else {
        digitalWrite(Engine_IN1, LOW);
        digitalWrite(Engine_IN2, LOW);
      }
      
      digitalWrite(LedRight, HIGH);
      digitalWrite(LedLeft, LOW);
      digitalWrite(YellowLed1, HIGH);
      digitalWrite(GreenLed1, LOW);
    } else {
      
      digitalWrite(LedRight, LOW);
      digitalWrite(YellowLed1, LOW);
      digitalWrite(GreenLed1, HIGH);
    }
    digitalWrite(LedRight, RightState ? HIGH : LOW);

    lastDebounceTime = millis();
  }
}

void ButtonsCounter() {

  if (digitalRead(ButtonPlus) == LOW) {
    delay(50);          // Делаем небольшую задержку для устранения дребезга контактов
    if (digitalRead(ButtonPlus) == LOW) { // Проверяем состояние кнопки после задержки
      counter++;
      
      delay(50);
      if (counter >= 9999){
        counter = 0;
      }
    }
  }

  if (digitalRead(ButtonMinus) == LOW) {
    delay(50); // Делаем небольшую задержку для устранения дребезга контактов
    if (digitalRead(ButtonMinus) == LOW) { // Проверяем состояние кнопки после задержки
      counter--; // Уменьшаем счетчик на 1
      
      delay(50);
      if (counter <= 0){
        counter = 0;
      }
    }
  }
  display.showNumberDec(counter, false); // Виводимо на дисплей значення
}

void EmergencyMode() {
 display.showNumberDec(0000, false);  // false указывает на то, что лидирующие нули не будут отображаться;
 digitalWrite(RedLed1, HIGH);         // Червоний світлодіод вмикаємо;
 digitalWrite(YellowLed1, LOW);
 digitalWrite(GreenLed1, LOW);

for (int i = 0; i < 3; i++) { // Вмикаємо сигналізацію з частотою затримки в 1 секунду;
  tone(Buzzer, 600);
  delay(100);
  noTone(Buzzer);
  delay(100);
}
delay(1000);

  motorState = false;
}
