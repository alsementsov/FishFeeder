# -*- coding: utf-8 -*-
import requests
import json

url = 'http://192.168.0.111'

#Choose command
SW = 0

if SW==0:
    payload = {'Command':0}
elif SW==1:
    payload = {'Command':1,'Time':'2020-08-29T13:40:00'}
elif SW==2:
    payload = {'Command':2,'EjectStart':'09:34','EjectEnd':'19:00','EjectFreq':100,'EjectWeight':700}
elif SW==3:
    payload = {'Command':3} # Тарировка  
elif SW==4:
    payload = {'Command':4} # Калибровка
elif SW==5:
    payload = {'Command':5} # Старт очистки
elif SW==6:
    payload = {'Command':6} # Стоп очистки
elif SW==7:
    payload = {'Command':7} # Глобальный СТОП 
elif SW==8:
    payload = {'Command':8} # Отключение СТОПа - ГЛОБАЛЬНЫЙ СТАРТ
elif SW==9:
    payload = {'Command':9,'DefConsump':1000,'Status':1} # Расход по умолчанию и режим автокоррекции
elif SW==10:
    payload = {'Command':10,'SSID':'ABS','Password':'13121985','Mode':'1','IP':'192.168.0.111','IPR':'192.168.0.1'} # AP

headers = {'content-type': 'application/json'}

resp = requests.post(url, data=json.dumps(payload), headers=headers)

print('HTTP Status: ',resp.status_code)
print(resp.text)
answer = resp.json() 
w=int(answer['Weight'])/1000
print('Weight: '+str(round(w)))
