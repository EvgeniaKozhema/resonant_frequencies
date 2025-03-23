#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "arduinoFFT.h"

const int micPin = A0;
const int samples = 128;
const double samplingFrequency = 1630;

double vReal[samples];
double vImag[samples];

ArduinoFFT<double> FFT = ArduinoFFT<double>();

// Энкодер
const int encoderCLK = 2;
const int encoderDT = 3;
const int encoderSW = 4;
volatile int lastEncoderState;
int amplitudeThreshold = 30;  // Начальное значение порога

// LCD 1602A (I2C-адрес 0x27)
LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup() {
  Serial.begin(115200);
  pinMode(micPin, INPUT);

  // Настройка энкодера
  pinMode(encoderCLK, INPUT);
  pinMode(encoderDT, INPUT);
  pinMode(encoderSW, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderCLK), updateEncoder, CHANGE);

  // Настройка дисплея
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Freq: --- Hz");
  lcd.setCursor(0, 1);
  lcd.print("Threshold: 30");
}

void loop() {
  // Считываем сигнал
  for (int i = 0; i < samples; i++) {
    vReal[i] = analogRead(micPin) - 128;  // Центрируем сигнал
    vImag[i] = 0;
    delayMicroseconds(1000000 / samplingFrequency);
  }

  // FFT
  FFT.windowing(vReal, samples, FFT_WIN_TYP_HANN, FFT_FORWARD);
  FFT.compute(vReal, vImag, samples, FFT_FORWARD);
  FFT.complexToMagnitude(vReal, vImag, samples);

  // Поиск резонансной частоты
  double peakFrequency = 0;
  double peakMagnitude = 0;

  for (int i = 2; i < samples / 2; i++) {
    double freq = (i * samplingFrequency) / samples;
    if (freq >= 100 && freq <= 1000 && vReal[i] > amplitudeThreshold) {
      if (vReal[i] > peakMagnitude) {
        peakMagnitude = vReal[i];
        peakFrequency = freq;
      }
    }
  }

  // Вывод в Serial
  Serial.print("Peak Frequency: ");
  Serial.print(peakFrequency);
  Serial.print(" Hz | Threshold: ");
  Serial.println(amplitudeThreshold);

  // Вывод на дисплей
  lcd.setCursor(6, 0);
  lcd.print(peakFrequency, 0);
  lcd.print(" Hz ");

  lcd.setCursor(10, 1);
  lcd.print(amplitudeThreshold);
  lcd.print("   ");

  delay(500);
}

// Функция обработки энкодера
void updateEncoder() {
  int newState = digitalRead(encoderCLK);
  if (newState != lastEncoderState) {
    if (digitalRead(encoderDT) != newState) {
      amplitudeThreshold += 1;  // Увеличиваем порог
    } else {
      amplitudeThreshold -= 1;  // Уменьшаем порог
    }
    amplitudeThreshold = constrain(amplitudeThreshold, 10, 200);
  }
  lastEncoderState = newState;
}

// Сброс порога кнопкой
void resetThreshold() {
  if (digitalRead(encoderSW) == LOW) {
    amplitudeThreshold = 30;
    lcd.setCursor(10, 1);
    lcd.print("30  ");
  }
}
