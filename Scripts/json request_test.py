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
    payload = {'ID': ID,'Command':1,'Data':{'Time':'2020-08-29T13:40:00'}}
elif SW==2:
    payload = {'ID': ID,'Command':2,'Data':{'EjectStart':'09:34','EjectEnd':'19:00','EjectFreq':100,'EjectWeight':700}}
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
    payload = {'ID': ID,'Command':10,'Data':{'DefConsump':1000,'Status':1}} # Расход по умолчанию и режим автокоррекции
elif SW==11:
    payload = {'ID': ID,'Command':11,'Data':{'SSID':'ABS','Password':'13121985'}} # AP

headers = {'content-type': 'application/json'}

resp = requests.post(url, data=json.dumps(payload), headers=headers)

print('HTTP Status: ',resp.status_code)
print(resp.text)
answer = resp.json() 
w=int(answer['Data']['Weight'])/1000
print('Weight: '+str(round(w)))
