#include <Wire.h>
#include <LiquidCrystal_I2C.h>

const int clk = 2;   // Пин CLK энкодера
const int dt = 3;    // Пин DT энкодера
const int sw = 4;    // Пин кнопки энкодера
const int buzzer = 8; // Пин пищалки

int last_clk;
int curr_clk;
int frequency = 500; // Начальная частота, Гц

// Укажи адрес дисплея (обычно 0x27 или 0x3F)
LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup() {
  pinMode(clk, INPUT);
  pinMode(dt, INPUT);
  pinMode(sw, INPUT_PULLUP);
  pinMode(buzzer, OUTPUT);

  last_clk = digitalRead(clk);

  Serial.begin(9600);

  lcd.init();         // Инициализация дисплея
  lcd.backlight();    // Включаем подсветку
  lcd.setCursor(0, 0);
  lcd.print("Freq: "); // Выводим начальное значение
  lcd.setCursor(6, 0);
  lcd.print(frequency);
}

void loop() {
  // Проверка вращения энкодера
  curr_clk = digitalRead(clk);
  if (curr_clk != last_clk) { // Если произошло изменение
    if (digitalRead(dt) != curr_clk) {
      frequency += 1; // Увеличиваем частоту
    } else {
      frequency -= 1; // Уменьшаем частоту
    }

    // Ограничим диапазон частот
    if (frequency < 100) frequency = 100;
    if (frequency > 1000) frequency = 1000;

    Serial.print("Частота: ");
    Serial.println(frequency);

    tone(buzzer, frequency); // ОБНОВЛЯЕМ ЗВУК!!!

    // Обновляем значение на дисплее
    lcd.setCursor(6, 0);
    lcd.print("     ");  // Очищаем старое значение
    lcd.setCursor(6, 0);
    lcd.print(frequency);

    last_clk = curr_clk;
  }

  // Проверка кнопки энкодера
  if (digitalRead(sw) == LOW) {
    delay(50); // Антидребезг
    if (digitalRead(sw) == LOW) {
      frequency = 500; // Сброс частоты
      Serial.println("Частота сброшена");
      tone(buzzer, frequency); // Сброс звука

      // Обновляем дисплей
      lcd.setCursor(6, 0);
      lcd.print("     ");  // Очищаем старое значение
      lcd.setCursor(6, 0);
      lcd.print(frequency);

      while (digitalRead(sw) == LOW); // Ждем отпускания кнопки
      delay(50);
    }
  }
}  