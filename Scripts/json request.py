# -*- coding: utf-8 -*-
import requests
import json

url = 'http://192.168.0.105'

payload = {'ID': 1,'Command':0}
headers = {'content-type': 'application/json'}

resp = requests.post(url, data=json.dumps(payload), headers=headers)

print('HTTP Status: ',resp.status_code)
print(resp.text)
answer = resp.json() 
print('ID = ',answer['ID'])
print('Year = ',answer['Year'])
print('Month = ',answer['Month'])
print('Day = ',answer['Day'])