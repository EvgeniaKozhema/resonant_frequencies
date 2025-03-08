#include "arduinoFFT.h"

const int micPin = A0;  // Пин, к которому подключен микрофон
const int samples = 128;  // Количество семплов для FFT
const double samplingFrequency = 2000;  // Частота дискретизации (примерно)

double vReal[samples];  // Массив для хранения реальной части данных
double vImag[samples];  // Массив для хранения мнимой части данных

ArduinoFFT<double> FFT = ArduinoFFT<double>();  // Создаем объект FFT с типом double

void setup() {
  Serial.begin(115200);
  pinMode(micPin, INPUT);
}

void loop() {
  // Сбор данных
  for (int i = 0; i < samples; i++) {
    vReal[i] = analogRead(micPin);  // Считываем данные с микрофона
    vImag[i] = 0;  // Мнимая часть всегда 0 для реальных сигналов
    delayMicroseconds(1000000 / samplingFrequency);  // Задержка для соблюдения частоты дискретизации
  }

  // Выполняем FFT
  FFT.windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.compute(vReal, vImag, samples, FFT_FORWARD);
  FFT.complexToMagnitude(vReal, vImag, samples);

  // Поиск частоты с максимальной амплитудой
  double peakFrequency = 0;
  double peakMagnitude = 0;
  for (int i = 2; i < (samples / 2); i++) {  // Игнорируем постоянную составляющую и зеркальные частоты
    if (vReal[i] > peakMagnitude) {
      peakMagnitude = vReal[i];
      peakFrequency = (i * 1.0 * samplingFrequency) / samples;
    }
  }

  // Выводим результат
  Serial.print("Peak Frequency: ");
  Serial.print(peakFrequency);
  Serial.println(" Hz");
  delay(1000);  // Задержка перед следующим измерением
}