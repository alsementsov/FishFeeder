# -*- coding: utf-8 -*-
import requests
import json
url = 'http://192.168.0.110'

#Choose command
SW=6

if SW==0:
    payload = {'ID': 1,'Command':0}
elif SW==1:
    payload = {'ID': 1,'Command':1,'Year':2020, 'Month':6, 'Day':24, 'Hour':12, 'Minute':40}
elif SW==2:
    payload = {'ID': 1,'Command':2,'StartHour':23,'StartMinute':15,'EndHour':23,'EndMinute':35,'Freq_day':20,'Weight_day':100}
elif SW==3:
    payload = {'ID': 255,'Command':3,'IDnew':1}
elif SW==4:
    payload = {'ID': 1,'Command':4} # Тарировка  
elif SW==5:
    payload = {'ID': 1,'Command':5} # Калибровка
elif SW==6:
    payload = {'ID': 1,'Command':6} # Старт очистки
elif SW==7:
    payload = {'ID': 1,'Command':7} # Стоп очистки
elif SW==8:
    payload = {'ID': 1,'Command':8} # Глобальный СТОП 
elif SW==9:
    payload = {'ID': 1,'Command':9} # Отключение СТОПа - ГЛОБАЛЬНЫЙ СТАРТ
elif SW==10:
    payload = {'ID': 1,'Command':10,'DefConsump':1000,'Status':1} # Consumption  
elif SW==11:
    payload = {'ID': 1,'Command':11,'SSID':'ABS','PASSWORD':'13121985'} # AP


headers = {'content-type': 'application/json'}

resp = requests.post(url, data=json.dumps(payload), headers=headers)

print('HTTP Status: ',resp.status_code)
print(resp.text)
answer = resp.json() 
w=int(answer['Weight'])/1000
print('Weight: '+str(round(w)))
print('Start = ',answer['StartHour'],':',answer['StartMinute'])
print('End   = ',answer['EndHour'],':',answer['EndMinute'])
print('Частота = ',answer['Freq_day'],' / Вес за день = ',answer['Weight_day'])