#include <Arduino.h>
#include <RTClib.h>
#include <WiFi.h>
#include <HX711.h>
#include <String> 
using namespace std;

#define MOTORPIN 15
#define LEDEXT 2
#define LED1 12
#define LED2 14
#define LED3 4
#define ETALON 194 //My Lenovo smartphone                                                   // указываем эталонный вес
#define CONVERTION_RATE 0.03527
#define LOADCELL_DOUT_PIN 5
#define LOADCELL_SCK_PIN 18
#define SLength 65
#define Nparams 15
#define TWAIT_AP 50
#define SSID_DEFAULT "ABS"
#define PWD_DEFAULT "13121985"
#define DELAY_FOR_SCALE 3000 
#define KF_Q 0.1 
#define KF_R 0.01
#define T_CLEAN 20000  
#define TH_CLEAN 5000

// Описание битов регистра статуса
#define STATUS_ADJUSTMENT 0 // 0 - without adj feeding timings
#define STATUS_CLEAN      1
#define STATUS_STOP       2
#define STATUS_ERROR_NOCHANGING   3 //Проверяется в main в процессе кормления
#define STATUS_ERROR_SCALE      4  //Проверяется в setup
#define STATUS_ERROR_RTC        5  //Проверяется в setup
#define STATUS_ERROR_Imotor     6  // Нет пока !!! - нужно тестить плату измерителя тока
#define STATUS_RESERV   7      // В статусе AP = 1 (по умолчанию), если коннектится как Станция = 0

// AP config itself
#define OWN_SSID "FishFeeder"
#define OWN_PWD "14091982" 

struct Parameters
{   int ID_f;
    int Hour_start;
    int Minute_start;
    int Hour_end;
    int Minute_end;
    int NperDay;
    int WperDay;
    long Consumption; //MilliGramm per second 
    long Weight;
    float CalFactor;
    long Offset;
    uint8_t Status=0;
    String ssid;
    String password;
    uint8_t Mode; // 0-AP, 1-Station 
};
struct timings
{
  unsigned long Tstart;
  unsigned long Tend;
  unsigned long T;// Период между выбросами в миллисекундах
  long WperSample; // Вес за выброс в миллиграммах
  long Duration;  // длительность выброса в миллисекундах
  long WperSample_cur;
  float Qr;
};

////////////////////////////////////////////////////////////////////////
String Client_connect(RTC_DS3231 *rtc, Parameters *jdata, WiFiServer *server,HX711 *scale);
void EEPROM_float_write(int addr, float val);
float EEPROM_float_read(int addr);
void EEPROM_long_write(int addr, long val);
long EEPROM_long_read(int addr);
float Calibrate(HX711 *scale,Parameters *jdata);
long MeausureWeight(HX711 *scale);
void Calculate_timings(Parameters *jdata, timings *Feed_timings);
void ParseJSON(String *s,RTC_DS3231 *rtc,Parameters *jdata,timings *Feed_timings,HX711 *scale);
struct Parameters ReadParameters();
void EEPROM_String_write(int addr,String data);
String EEPROM_String_read(int addr);
bool WiFi_connect(Parameters *jdata, WiFiServer *server);
void RTC_init(Parameters *jdata,RTC_DS3231 *rtc);
bool SmartDelay (const unsigned long Tdelay);
long Kalman_filter(long val,float Q);