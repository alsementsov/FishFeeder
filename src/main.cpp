#include <HX711.h>
#include "main.h"

HX711 scale;
const int z = 10;                                                        // указываем количество измерений, по которым будет найдено среднее значение
float     calibration_value[z];                                                           // создаём массив для хранения считанных значений
float     calibration_factor = 0;  
float units;                                                  // задаём переменную для измерений в граммах
float ounces;                                                       // создаём переменную для значения калибровочного коэффициента

void setup() {
  Serial.begin(115200);
  Serial.println("HX711 Demo");

  Serial.println("Initializing the scale");

  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);                                                         // инициируем работу с платой HX711, указав номера выводов Arduino, к которым подключена плата
  scale.set_scale();                                                                      // не калибруем полученные значения
  scale.tare();                                                                           // обнуляем вес на весах (тарируем)
  Serial.println("You have 10 seconds to set your known load");                           // выводим в монитор порта текст о том, что у вас есть 10 секунд для установки эталонного веса на весы
  delay(10000);                                                                           // ждём 10 секунд
  Serial.print("calibration factor: ");                                                   // выводим текст в монитор поседовательного порта
  for (int i = 0; i < z; i++) {                                                           // запускаем цикл, в котором
    calibration_value[i] = scale.get_units(1) / (ETALON / CONVERTION_RATE);   // считываем значение с тензодатчика и переводим его в граммы
    calibration_factor += calibration_value[i];                                           // суммируем все значения
  }
  calibration_factor = calibration_factor / z;                                            // делим сумму на количество измерений
  Serial.println(calibration_factor);
  Serial.println("Remove your load for tare (10 seconds...)");  
  delay(10000);    
  scale.set_scale();                                          // выполняем измерение значения без калибровочного коэффициента
  scale.tare();                                               // сбрасываем значения веса на датчике в 0
  scale.set_scale(calibration_factor);        
}

void loop() {
  Serial.print("Reading:\t");
  units = scale.get_units(10);
  ounces = units * 0.035274;                                  // переводим вес из унций в граммы
  Serial.print(ounces);                                       // выводим в монитор последовательного порта вес в граммах
  Serial.println(" grams"); 
	        // put the ADC in sleep mode
  delay(5000);

}