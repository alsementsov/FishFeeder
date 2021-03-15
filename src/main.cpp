#define Version 24 //Version AP
#include <Arduino.h>
#include <HX711.h>
#include <WiFi.h>
#include <RTClib.h>
#include <EEPROM.h>
#include <ArduinoJson.h>
#include "feeder.h"

RTC_DS3231 rtc;
WiFiServer server(80);
struct Parameters jdata;
HX711 scale;
struct timings Feed_timings;
unsigned long t_start_sample=0; 
unsigned long tnow =0;
int N = 0;
unsigned long delta =0;
long Weight_before;
uint16_t NoChanging_cnt=0;
bool firstloop;
bool Connect_ExtAP=0;
bool feed = 0;
bool flag_calc = 0;
long Consumption_temp;
unsigned long Tstart_clean=0;
long Wstart_clean=0;
bool IP_flag;
bool IPR_flag;
IPAddress local_IP;
IPAddress gateway_IP; 
IPAddress subnet(255, 255, 255, 0); // По умолчанию

///////////////////////////// SETUP ////////////////////////////////////////////////////
void setup() {
  //Обнуление статуса ошибок при старта
  Serial.begin(115200);
  pinMode(MOTORPIN, OUTPUT); 
  pinMode(LED1, OUTPUT);  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);  pinMode(LEDEXT, OUTPUT);
  digitalWrite(LED1,LOW);  digitalWrite(LED2,LOW);
  digitalWrite(LED3,HIGH);  digitalWrite(LEDEXT,LOW);
   
  EEPROM.begin(64);     // set the LED pin mode
  delay(10);
  //Version
  Serial.print("Version: ");Serial.println(Version);
  //////// READ PARAMETERS from EEPROM ////////
  jdata = ReadParameters();
  // RTC starting
  RTC_init(&jdata,&rtc); // было выше чтения параметров
  Serial.print("IP: ");Serial.println(jdata.IP);
  Serial.print("IP_gateway: ");Serial.println(jdata.IPR);
  // Station starting
  if (jdata.Mode ==1) {
    unsigned long t = millis();
    // Задаем статический IP-адрес:
    IP_flag = local_IP.fromString(jdata.IP);
    // Задаем IP-адрес сетевого шлюза:
    IPR_flag = gateway_IP.fromString(jdata.IPR);
    // Настраиваем статический IP-адрес:
    if ((!WiFi.config(local_IP, gateway_IP, subnet))||(IP_flag==0)||(IPR_flag==0)) {
      Serial.println("-----> ERROR  - STA Failed to configure !"); // Если есть ошибка в конфигурации сети
      jdata.Mode=0; // переключаемся на дефолтную AP
    }
    else{
      while ((Connect_ExtAP==0)&&((millis()-t)<10000))    
      {
        Connect_ExtAP = WiFi_connect(&jdata, &server);  
      }
      if (WiFi.getAutoConnect() != true) WiFi.setAutoConnect(true);  //on power-on automatically connects to last used hwAP
      WiFi.setAutoReconnect(true);
      firstloop = 1;
    }
  }
  if (jdata.Mode ==0){
    //AP start
      IPAddress apIP(192, 168, 0, 1);
      WiFi.mode(WIFI_AP);
      WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
      WiFi.softAP(OWN_SSID, OWN_PWD);
      Serial.print("-----> WIFI_AP:   ");Serial.println(WiFi.softAPIP());
      server.begin();
  }
  // Расчет таймингов кормления
  Calculate_timings(&jdata,&Feed_timings);
  // Настройка ТЕНЗОДАТЧИКА
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  delay(1000); 
  if (scale.is_ready()) {     // сбрасываем значения веса на датчике в 0   
    scale.set_offset(jdata.Offset);
    scale.set_scale(jdata.CalFactor);  
    Serial.print("Scale parameters init:");Serial.print(jdata.Offset);Serial.print("/");Serial.println(jdata.CalFactor);  
  }
  else  {  
    bitSet(jdata.Status,STATUS_ERROR_SCALE);// Запись ошибки ВЕСОВ
    Serial.println("-- Scale connection ERROR ! --");
  }
  Consumption_temp = jdata.Consumption;
}
//////////////////////////////// LOOP ////////////////////////////////////////////////////////
void loop() {

  // ******** FEEDING **********
  // if there are no Clean or global STOP
  if ((bitRead(jdata.Status,STATUS_CLEAN)==0)&&(bitRead(jdata.Status,STATUS_STOP)==0)) 
  {
      // if counter is null -> first sample (start day) 
      if (N==0){
        DateTime rtctime = rtc.now();
        unsigned long urtctime = rtctime.unixtime();
        urtctime %= 86400;
      // Check active period of day
      if ((urtctime >= Feed_timings.Tstart) && (urtctime < (Feed_timings.Tend-(Feed_timings.T/1000))))
      {
        //START feeding
        jdata.Weight = MeausureWeight(&scale);
        Serial.print("Weight at starting = ");Serial.println(jdata.Weight);
        Weight_before = jdata.Weight;
        N = ceil((urtctime-Feed_timings.Tstart)/Feed_timings.T)+1;
        Serial.print("........................Nfeed = ");Serial.println(N);
        Feed_timings.WperSample_cur = Feed_timings.WperSample;// Weight of samples to default value
        Feed_timings.Duration = (Feed_timings.WperSample_cur*1000)/jdata.Consumption;// Default duration
        Serial.print("WperSample_cur = ");Serial.print(Feed_timings.WperSample_cur);
        Serial.print(" / Duration = ");Serial.println(Feed_timings.Duration);      
        t_start_sample= millis(); // Save start time
        digitalWrite(MOTORPIN,HIGH);
        digitalWrite(LED1,HIGH);
        feed=1;
      }
    }
    // Обычный выброс
    else if (N <= jdata.NperDay){
      unsigned long tnow =  millis();
      delta = long(tnow - t_start_sample); //Calc delta between start and now
      // Старт выброса
      if (delta >= Feed_timings.T){ // Duration between samples more than nominal
        //Check overload by abs-time
        DateTime rtctime = rtc.now();
        long urtctime = rtctime.unixtime();
        urtctime %= 86400;
        //Serial.println("Delta > T");
        if (urtctime <= Feed_timings.Tend){
          t_start_sample = tnow; // Start new sample and save timer value
          N++; //
          Weight_before = MeausureWeight(&scale);//Save weight before sample
          Serial.print("Weight_before = ");Serial.println(Weight_before);
          jdata.Weight = Weight_before;
          digitalWrite(MOTORPIN,HIGH);
          digitalWrite(LED1,HIGH);
          feed=1;
          Serial.print("........................Nfeed = ");Serial.println(N,DEC); 
        } 
        else{ //if more than day - switch off 
          N=0;
          digitalWrite(MOTORPIN,LOW);
          digitalWrite(LED1,LOW);
          if (feed==1)
          {
            feed=0;
            Serial.println("-> STOP !");
          }
        }
      }
      //End of sample
      else if (delta >= Feed_timings.Duration)
      {
        if (feed==1)
        {
          digitalWrite(MOTORPIN,LOW);
          digitalWrite(LED1,LOW);
          feed = 0;
          flag_calc = 1;
        }
        if (delta > (Feed_timings.Duration + DELAY_FOR_SCALE)&&(flag_calc==1))
        {
          flag_calc=0;
          jdata.Weight = MeausureWeight(&scale);
          Serial.print("Weight after = ");Serial.println(jdata.Weight);
          long wdelta = Weight_before-jdata.Weight; //Weight per current feeds
          Serial.print("Wdelta = ");Serial.println(wdelta);
          if (Feed_timings.Duration!=0)
          {
            Consumption_temp = (wdelta*1000)/Feed_timings.Duration;
            Serial.print("Consumption_temp = ");Serial.println(Consumption_temp);
          }
          if (Consumption_temp>100)
          {
            if(bitRead(jdata.Status,STATUS_ADJUSTMENT)==1)
            {
             jdata.Consumption = Kalman_filter(Consumption_temp,Feed_timings.Qr);
             Serial.print("Сonsumption:");Serial.println(jdata.Consumption);
            }
            NoChanging_cnt=0;
            bitClear(jdata.Status,STATUS_ERROR_NOCHANGING);//Restore error of feedings
          }
          else if  (NoChanging_cnt >= 5)
          {
            bitSet(jdata.Status,STATUS_ERROR_NOCHANGING);//Write error of feedings
            Serial.println("-> ERROR_NOCHANGING !");
          }
          else
          {
            NoChanging_cnt++;
            Serial.print("NO-CHANGING COUNTER = ");Serial.println(NoChanging_cnt);
          }
          //if (jdata.Consumption>100)
          //{
          Feed_timings.WperSample_cur = Feed_timings.WperSample + Feed_timings.WperSample_cur - wdelta;//Recalc weight of next samples
          Serial.print("WperSample_cur:");Serial.print(Feed_timings.WperSample_cur);
          if (Feed_timings.WperSample_cur > 0)
          {
            Feed_timings.Duration = (Feed_timings.WperSample_cur*1000)/jdata.Consumption;// Recalc next duration 
            if (Feed_timings.Duration >= Feed_timings.T)
              Feed_timings.Duration = Feed_timings.T-3500;
          }
          else 
            Feed_timings.Duration = 0;
          Serial.print(" / Duration:");Serial.println(Feed_timings.Duration);
        }
      }
    } 
    // N>Nperday = if counter over - null. Stop feeding
    else {
      digitalWrite(MOTORPIN,LOW); 
      digitalWrite(LED1,LOW);
      Serial.println("-> STOP DAY !");
      N=0;
      feed=0;
      // Save Consumption in the end of day
      //if (bitRead(jdata.Status,STATUS_ADJUSTMENT)==1)
      //{
        //EEPROM.begin(11);
       // EEPROM.write(9,highByte(jdata.Consumption));
        //EEPROM.write(10,lowByte(jdata.Consumption));
        //EEPROM.commit();
        //Serial.print("Update consumption = ");Serial.println(String(jdata.Consumption));
      //}
    }
  }
  ////////// SMART CLEANING ///////////
  else if (bitRead(jdata.Status,STATUS_CLEAN)==1)
  {
    if (Tstart_clean==0)
    {
      Tstart_clean = millis();
      Wstart_clean = MeausureWeight(&scale);
      Serial.println(Wstart_clean);
      digitalWrite(MOTORPIN,HIGH);
      digitalWrite(LED1,HIGH);
    }
    else if (((millis()-Tstart_clean)>=T_CLEAN))
    {
      digitalWrite(MOTORPIN,LOW);
      digitalWrite(LED1,LOW);
      delay(1000);
      long Wcur = MeausureWeight(&scale);
      Serial.println(Wcur);
      if ((Wstart_clean-Wcur) <= TH_CLEAN)
      {
        bitClear(jdata.Status,STATUS_CLEAN);
        Serial.print("-> END CLEANING !");
      }
      Tstart_clean=0;
    }
  }

  //////////////////////////////////////////////////////
  ///////////////////// WIFI ///////////////////////////
  //////////////////////////////////////////////////////
  if (jdata.Mode==1)
  {
    if ((Connect_ExtAP==0)&&(firstloop==0)){
      Connect_ExtAP =  WiFi_connect(&jdata, &server);
    }
    else if (WiFi.status() != WL_CONNECTED)
    {
      WiFi.reconnect(); 
      Serial.println("AP is lost. Trying to AP reconnect...");
    }
    else
    {
      String s;
      s = Client_connect(&rtc,&jdata,&server,&scale);
      ParseJSON(&s,&rtc,&jdata,&Feed_timings,&scale);
      if (jdata.Mode==0)
      {
        IPAddress apIP(192, 168, 0, 1);
        WiFi.mode(WIFI_AP);
        WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
        WiFi.softAP(OWN_SSID, OWN_PWD);
        Serial.print("Start as AP: ");Serial.println(WiFi.softAPIP());
        server.begin();
      }
    }
  }  
  else
  {
    String s;
    s = Client_connect(&rtc,&jdata,&server,&scale);
    ParseJSON(&s,&rtc,&jdata,&Feed_timings,&scale);
    if (jdata.Mode==1)
    {
      String temp_ssid = jdata.ssid;
      String temp_pwd = jdata.password;
      IP_flag = local_IP.fromString(jdata.IP);
      IPR_flag = gateway_IP.fromString(jdata.IPR);
      // Настраиваем статический IP-адрес:
      if ((!WiFi.config(local_IP, gateway_IP, subnet))||(IP_flag==0)||(IPR_flag==0)) {
        Serial.println("-----> ERROR  - STA Failed to configure !"); // Если есть ошибка в конфигурации сети
        jdata.Mode=0; // остаемся на дефолтной AP
      }
      else
      {
        WiFi.begin(&temp_ssid[0],&temp_pwd[0]); //новые параметры из jdata
        Serial.print("Start as STA = ");Serial.println(temp_ssid+" / "+temp_pwd+" / IP= "+jdata.IP+" / IPg="+jdata.IPR);
      }
    } 
  }
  if (firstloop ==1)  {firstloop = 0;}
}
