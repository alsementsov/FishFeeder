#include <Arduino.h>
#include "feeder.h"
#include <RTClib.h>
#include <WiFi.h>
#include <EEPROM.h>
#include <ArduinoJson.h>

bool SmartDelay (const unsigned long Tdelay)
{
  static bool flag = 1;
  static unsigned long Tstart;

  //Счетчик не запущен - запускаем при вызове функции
  if (flag==1) 
  {
    flag=0; 
    Tstart=millis();
  }
  //Счетчик запущен
  else if ((millis()-Tstart)>Tdelay)
  {
    flag=1;
  }
  return flag;
}

String Client_connect(RTC_DS3231 *rtc, Parameters *jdata, WiFiServer *server,HX711 *scale){
  String StrokaParametrov = "";
  WiFiClient client = server->available(); 
 // listen for incoming clients
  if (client) {                             // if you get a client,
    Serial.print("...... New connection ......");           // print a message out the serial port
    String currentLine = "";
    boolean flag = false; 
    // make a String to hold incoming data from the client
    //delay(50); // Заменено на чтение RTC 
    DateTime prtc = rtc->now(); //Измерение времени если есть новое соединение
    long Weight_cur = MeausureWeight(scale);//Измерение текущего веса
    while (client.connected()) { 
      // loop while the client's connected
       if (client.available() or (flag==false)) { 
        // if there's bytes to read from the client,
        char c = client.read();   
        //Serial.print(c); // read a byte, then
        if (c == '{'){
          flag = true;
        }
        if (flag==true) {
          StrokaParametrov += c;
          }
      }
      else {
            Serial.println("Answer...");
            client.println("HTTP/1.1 200 OK");
            client.println("Content-Type: application/json");
            client.println();
            // Передача параметров обратно на сервер опроса
            //client.println(StrokaParametrov);// для примера
            client.print("{\"ID\":");
            client.print(jdata->ID_f); 
            client.print(",\"Status\":");
            client.print(jdata->Status); 
            client.print(",\"Data\":{"); //DATA structure
            client.print("\"Date\":");
            if (prtc.day()<10) client.print("0"); client.print(prtc.day()); client.print("-");
            if (prtc.month()<10) client.print("0");client.print(prtc.month()); client.print("-");
            client.print(prtc.year());
            client.print(",\"Time\":");
            if (prtc.hour()<10) client.print("0"); client.print(prtc.hour()); client.print(":");
            if (prtc.minute()<10) client.print("0");client.print(prtc.minute());
            client.print(",\"EjectStart\":");
            if (jdata->Hour_start<10) client.print("0"); client.print(jdata->Hour_start); client.print(":");
            if (jdata->Minute_start<10) client.print("0");client.print(jdata->Minute_start); 
            client.print(",\"EjectEnd\":");
            if (jdata->Hour_end<10) client.print("0"); client.print(jdata->Hour_end); client.print(":");
            if (jdata->Minute_end<10) client.print("0");client.print(jdata->Minute_end);
            client.print(",\"EjectFreq \":");
            client.print(jdata->NperDay); 
            client.print(",\"EjectWeight\":");
            client.print(jdata->WperDay); 
            client.print(",\"Weight\":");        
            client.print(Weight_cur);//jdata->Weight); 
            client.print(",\"DefConsump\":");
            client.print(jdata->Consumption);
            client.print("}}"); 
            client.println(); 
            break;
       }
      }
      client.stop();
    }
    return StrokaParametrov;
}
void EEPROM_float_write(int addr, float val) // запись в ЕЕПРОМ
{ 
  //EEPROM.begin(addr+4);
  byte *x = (byte *)&val;
  for(byte i = 0; i < 4; i++) EEPROM.write(i+addr, x[i]);
  EEPROM.commit();
}
float EEPROM_float_read(int addr) // чтение из ЕЕПРОМ
{   
  byte x[4];
  for(byte i = 0; i < 4; i++) x[i] = EEPROM.read(i+addr);
  float *y = (float *)&x;
  return y[0];
}
long EEPROM_long_read(int addr)// Для чтения 4х байтовго значения пишим номер ячейки первого байта
{ // далее функция прочитает 4 ячейки по одному байту и соберет их в одну переменную типа unsigned long.
  long data = 0;
  byte val = EEPROM.read(addr+3);
  data = (data << 8) | val;
  val = EEPROM.read(addr+2);
  data = (data << 8) | val;
  val = EEPROM.read(addr+1);
  data = (data << 8) | val;
  val = EEPROM.read(addr);
  data = (data << 8) | val;
  return data;
}
void EEPROM_long_write(int addr, long val) {
  //EEPROM.begin(addr+4);
  byte *x = (byte *)&val;
  for(byte i = 0; i < 4; i++) EEPROM.write(i+addr, x[i]);
  EEPROM.commit();
}
float Calibrate(HX711 *scale,Parameters *jdata){
  float calibration_factor = 0;  
  const int z = 10;                                                        // указываем количество измерений, по которым будет найдено среднее значение
  float calibration_value[z];  
  scale->set_scale();                                                                      // не калибруем полученные значения
  //scale.tare(); //  тарировка уже известн                                                                    // ждём 8 секунд
  Serial.print("Calculating calibration factor...");                                                   // выводим текст в монитор поседовательного порта
  for (int i = 0; i < z; i++) {                                                           // запускаем цикл, в котором
    calibration_value[i] = scale->get_units(1) / (ETALON / CONVERTION_RATE);   // считываем значение с тензодатчика и переводим его в граммы
    calibration_factor += calibration_value[i];                                           // суммируем все значения
  }
  calibration_factor = calibration_factor/z;                                            // делим сумму на количество измерений
  Serial.println(calibration_factor);
  Serial.println("Calibration complete!");  
  scale->set_scale(calibration_factor);
  scale->set_offset(jdata->Offset);
  return calibration_factor;
}
long MeausureWeight(HX711 *scale)
// Измеряет вес в миллиграммах
{
  return (long)(scale->get_units(10)*35.274); //0.035274;
}
void Calculate_timings(Parameters *jdata, timings *Feed_timings){
  Feed_timings->Tstart = (jdata->Hour_start*3600) + (jdata->Minute_start*60);//
  Feed_timings->Tend = (jdata->Hour_end*3600) + (jdata->Minute_end*60);
  Feed_timings->T = ((Feed_timings->Tend-Feed_timings->Tstart)*1000)/jdata->NperDay;// Период между выбросами в миллисекундах
  Feed_timings->WperSample = (jdata->WperDay*1000)/jdata->NperDay; // Вес за выброс в миллиграммах
  Feed_timings->Duration = (Feed_timings->WperSample*1000)/jdata->Consumption; 
  Feed_timings->WperSample_cur = Feed_timings->WperSample;
  Feed_timings->Qr = KF_Q/jdata->NperDay;
  Serial.print("Tstart(sec) = ");Serial.println(Feed_timings->Tstart);
  Serial.print("Tend(sec) = ");Serial.println(Feed_timings->Tend);
  Serial.print("T(msec) = ");Serial.println(Feed_timings->T);
  Serial.print("Duration(msec) = ");Serial.println(Feed_timings->Duration);
  Serial.print("WperSample(mg) = ");Serial.println(Feed_timings->WperSample);
  Serial.print("WperSample_cur(mg) = ");Serial.println(Feed_timings->WperSample_cur);
  Serial.print("Consumption(mg/sec) = ");Serial.println(jdata->Consumption);
}
void ParseJSON(String *s,RTC_DS3231 *rtc,Parameters *jdata,timings *Feed_timings,HX711 *scale)
{
  int cmd;
  int ID;
  if (*s!=""){
    Serial.print("JSON request = ");
    Serial.println(*s); 
    DynamicJsonDocument doc(256);
    DeserializationError error = deserializeJson(doc, *s);
    if (error)  {Serial.println("Json error");}
    cmd = doc["Command"];
    ID = doc["ID"];
    Serial.print("ID=");Serial.print(ID); Serial.print(" / CMD=");Serial.println(cmd); 
      // Обработка запроса к этому устройству
    if (jdata->ID_f == ID){
        // 1 - Обновление времени (TIME UPDATE)
        if (cmd == 1){
          rtc->adjust(DateTime(doc["Data"]["Year"], doc["Data"]["Month"], doc["Data"]["Day"], doc["Data"]["Hour"], doc["Data"]["Minute"], 0));
          Serial.println(" -> Time is updated");
        }
        // 2 - Обновление режима кормления (FEEDING SETTINGS)
        else if (cmd == 2){
        jdata->Hour_start = doc["Data"]["StartHour"];
        Serial.println(jdata->Hour_start);
        jdata->Minute_start = doc["Data"]["StartMinute"];
        jdata->Hour_end = doc["Data"]["EndHour"];
        jdata->Minute_end = doc["Data"]["EndMinute"];
        jdata->NperDay = doc["Data"]["EjectFreq "];
        jdata->WperDay = doc["Data"]["EjectWeight"];
        //EEPROM.begin(9);
        EEPROM.write(1,jdata->Hour_start);
        EEPROM.write(2,jdata->Minute_start);
        EEPROM.write(3,jdata->Hour_end);
        EEPROM.write(4,jdata->Minute_end);
        EEPROM.write(5,highByte(jdata->NperDay));
        EEPROM.write(6,lowByte(jdata->NperDay));
        EEPROM.write(7,highByte(jdata->WperDay));
        EEPROM.write(8,lowByte(jdata->WperDay));
        EEPROM.commit();
        Serial.println(" -> Mode is updated:");
        Calculate_timings(jdata,Feed_timings);
        }
        // 3 - Смена ID
        else if (cmd==3){
        //EEPROM.begin(1);
        EEPROM.write(0,doc["Data"]["IDnew"]);
        jdata->ID_f = doc["Data"]["IDnew"];
        EEPROM.commit();
        Serial.print(" -> ID is updated = ");
        Serial.println(jdata->ID_f,DEC);
        }
        // 4 - Тарировка (нужно чтобы измерялся вес и записывался в EEPROM а также вычитался при старте из текущего веса)
        else if (cmd==4){
          scale->set_scale();   // выполняем измерение значения без калибровочного коэффициента
          jdata->Offset = scale->read_average(10); 
          scale->set_offset(jdata->Offset);
          EEPROM_long_write(16,jdata->Offset);
          scale->set_scale(jdata->CalFactor);
          Serial.print(" -> OFSSET is updated = ");Serial.println(jdata->Offset);
        }
        // 5 - Калибровка
        else if (cmd==5){
          jdata->CalFactor =  Calibrate(scale,jdata);
          EEPROM_float_write(12,jdata->CalFactor);
          Serial.print(" -> CALFACTOR is updated =");Serial.println(jdata->CalFactor);
        }
        // 6 - Очистка - старт 
        else if (cmd==6){
          bitSet(jdata->Status,STATUS_CLEAN);
          Serial.println("-> START CLEANING...");
        }
        // 7 - Очистка - отключение
        else if (cmd==7){
          bitClear(jdata->Status,STATUS_CLEAN);
          Serial.println(" -> END CLEANING !");
          digitalWrite(MOTORPIN,LOW);
          digitalWrite(LED1,LOW);
        }
        // 8 - ГЛОБАЛЬНЫЙ СТОП
        else if (cmd==8){
          bitSet(jdata->Status,STATUS_STOP);
          Serial.print(" -> STOP !");
          digitalWrite(MOTORPIN,LOW);
          digitalWrite(LED1,LOW);
        }
        // 9 - Отключение СТОПа - ГЛОБАЛЬНЫЙ СТАРТ
        else if (cmd==9){
          bitClear(jdata->Status,STATUS_STOP);
          Serial.print(" -> START...");
        }
        // 10 - Обновление расхода по умолчанию (CONSUMPTION)
        else if (cmd== 10){
          jdata->Consumption = doc["Data"]["DefConsump"];
          uint8_t status_from_server = doc["Status"];
          if ((bitRead(status_from_server,STATUS_ADJUSTMENT))==1)
            bitSet(jdata->Status,STATUS_ADJUSTMENT);
          else
            bitClear(jdata->Status,STATUS_ADJUSTMENT);
          //EEPROM.begin(12);
          EEPROM.write(9,highByte(jdata->Consumption));
          EEPROM.write(10,lowByte(jdata->Consumption));
          EEPROM.write(11,jdata->Status);
          EEPROM.commit();
          Serial.println(" -> NEW Consumption="+String(jdata->Consumption)+" / Adjustment_mode="+String(bitRead(jdata->Status,STATUS_ADJUSTMENT)));
        }
        //11 - SSID+PWD
        else if (cmd== 11){
        const char *ch_ssid = doc["Data"]["SSID"];
        const char *ch_pwd = doc["Data"]["PASSWORD"];
        jdata->ssid = String(ch_ssid);
        jdata->password = String(ch_pwd);
        String wr_data = jdata->ssid+':'+jdata->password+'&';
        EEPROM_String_write(20,wr_data);
        EEPROM.commit();
        Serial.println("-> Write auth.data = "+wr_data);
        }
    }
  }
}
struct Parameters ReadParameters()
{ 
  //EEPROM.begin(50);
  struct Parameters jdata;
  jdata.ID_f = EEPROM.read(0);
  jdata.Hour_start = EEPROM.read(1);
  if (jdata.Hour_start > 24){jdata.Hour_start = 9;}
  jdata.Minute_start = EEPROM.read(2);
  if (jdata.Minute_start > 60){jdata.Minute_start =0;}
  jdata.Hour_end = EEPROM.read(3);
  if (jdata.Hour_end > 24){jdata.Hour_end = 21;}
  jdata.Minute_end = EEPROM.read(4);
  if (jdata.Minute_end > 60){jdata.Minute_end = 0;}
  jdata.NperDay = word(EEPROM.read(5),EEPROM.read(6));
  if ((jdata.NperDay == 0 )||(jdata.NperDay > 65530)){jdata.NperDay=100;}    
  jdata.WperDay = word(EEPROM.read(7),EEPROM.read(8));
  if ((jdata.WperDay == 0 )||(jdata.WperDay > 65530)){jdata.WperDay=100;} 
  jdata.Consumption = word(EEPROM.read(9),EEPROM.read(10));
  if ((jdata.Consumption == 0)||(jdata.Consumption > 65530)){jdata.Consumption=1000;}
  jdata.Status = EEPROM.read(11); // Регистр статуса 
  jdata.Status &= 0x01;
  Serial.print("Adj_mode: ");Serial.println(jdata.Status);
  jdata.CalFactor = EEPROM_float_read(12); //12,13,14,15 for float
  if (jdata.CalFactor == 0){jdata.CalFactor=1;} 
  jdata.Offset = EEPROM_long_read(16);//16,17,18,19 for long
  // SSID + PWD
  String recivedData;
  recivedData = EEPROM_String_read(20);
  int num = recivedData.indexOf(':');
  if (num==-1){
    jdata.password = PWD_DEFAULT;
    jdata.ssid = SSID_DEFAULT;
    Serial.println("No saving SSID! Loading default connection:"+jdata.ssid+"/"+jdata.password);
  }
  else{
    String S_login = recivedData.substring(0,num);
    String pwd = recivedData.substring(num+1,recivedData.length()-1);
    jdata.ssid = S_login;
    Serial.print("SSID="); Serial.print(jdata.ssid);
    jdata.password = pwd;
    Serial.print("/password="); Serial.println(jdata.password);
  }
  return jdata; 
}
void EEPROM_String_write (int addr,String data)
{
  int String_size = data.length();
  int i;
  //EEPROM.begin(addr+String_size+1);
  for(i=0;i<String_size;i++)
  {
    EEPROM.write(addr+i,data[i]);
  }
  EEPROM.write(addr+String_size,'&');   //Add termination null character for String Data
  EEPROM.commit();
}
String EEPROM_String_read(int addr)
{
  char data[100]; //Max 100 Bytes
  int len=0;
  unsigned char k;
  k=EEPROM.read(addr);
  while(k != '&' && len<99)   //Read until null character
  {    
    k=EEPROM.read(addr+len);
    data[len]=k;
    len++;
  }
  data[len]='\0';
  return String(data);
}
bool WiFi_connect(Parameters *jdata, WiFiServer *server)
{
  if (SmartDelay(500)==1)
  { 
    String temp_ssid = jdata->ssid;
    String temp_pwd = jdata->password;
    Serial.print("Connecting to:");Serial.println(&temp_ssid[0]);
    WiFi.begin(&temp_ssid[0],&temp_pwd[0]);

  }
  // Запуск сервера если есть подключение по Wifi
  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.print("WiFi connected/"); Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    server->begin();
    return 1;
  }
  else 
    return 0;
}
void RTC_init(Parameters *jdata,RTC_DS3231 *rtc)
{
    if (! rtc->begin()) {
    Serial.println("ERROR: Couldn't find RTC...");
    bitSet(jdata->Status,STATUS_ERROR_RTC); //Запись ошибки ЧАСОВ;
  }
  DateTime prtc = rtc->now();
  Serial.print("RTC: ");Serial.print(prtc.year(), DEC);
  Serial.print('/');
  Serial.print(prtc.month(), DEC);
  Serial.print('/');
  Serial.print(prtc.day(), DEC);
  Serial.print(" (");
  Serial.print(prtc.hour(), DEC);
  Serial.print(':');
  Serial.print(prtc.minute(), DEC);
  Serial.print(':');
  Serial.print(prtc.second(), DEC);
  Serial.print(')');
  Serial.println();
}
long Kalman_filter(long val,float Q)
{
  static float Xe = val;
  static float Xp;
  static float P = 1;
  static float Pc;
  static float K;
  Xp = Xe;
  Pc = P + Q;
  K = Pc/(Pc + KF_R);
  P = (1-K)*Pc;
  Xe = K*(val-Xp)+Xp; 
  return (long)Xe;
}



