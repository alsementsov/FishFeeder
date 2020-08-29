# -*- coding: utf-8 -*-
import requests
import json
url = 'http://192.168.0.111'

ID=1
#Choose command
SW=0

if SW==0:
    payload = {'ID': ID,'Command':0}
elif SW==1:
    payload = {'ID': ID,'Command':1,'Data':{'Year':2020, 'Month':8, 'Day':7, 'Hour':14, 'Minute':10}}
elif SW==2:
    payload = {'ID': ID,'Command':2,'Data':{'StartHour':9,'StartMinute':0,'EndHour':10,'EndMinute':0,'Freq_day':120,'Weight_day':720}}
elif SW==3:
    payload = {'ID': ID,'Command':3,'Data':{'IDnew':1}}
elif SW==4:
    payload = {'ID': ID,'Command':4} # Тарировка  
elif SW==5:
    payload = {'ID': ID,'Command':5} # Калибровка
elif SW==6:
    payload = {'ID': ID,'Command':6} # Старт очистки
elif SW==7:
    payload = {'ID': ID,'Command':7} # Стоп очистки
elif SW==8:
    payload = {'ID': ID,'Command':8} # Глобальный СТОП 
elif SW==9:
    payload = {'ID': ID,'Command':9} # Отключение СТОПа - ГЛОБАЛЬНЫЙ СТАРТ
elif SW==10:
    payload = {'ID': ID,'Command':10,'Data':{'DefConsump':1000,'Status':1}} # Consumption  
elif SW==11:
    payload = {'ID': ID,'Command':11,'Data':{'SSID':'ABS','PASSWORD':'13121985'}} # AP


headers = {'content-type': 'application/json'}

resp = requests.post(url, data=json.dumps(payload), headers=headers)

print('HTTP Status: ',resp.status_code)
print(resp.text)
answer = resp.json() 
w=int(answer['Data']['Weight'])/1000
print('Weight: '+str(round(w)))
#print('Start = ',answer['StartHour'],':',answer['StartMinute'])
#print('End   = ',answer['EndHour'],':',answer['EndMinute'])
#print('Частота = ',answer['Freq_day'],' / Вес за день = ',answer['Weight_day'])