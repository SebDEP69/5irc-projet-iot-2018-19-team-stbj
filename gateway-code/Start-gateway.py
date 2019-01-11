import time
import requests
import serial
import json
import re

TOKEN = "PUT_YOUR_TOKEN_HERE"  # Put your TOKEN here
DEVICE_LABEL = "IOT_STBJ"  # Put your device label here 

# fonction return a JSON with variable give in input
def build_payload(co2_value,lat_value,lng_value):

    payload = {"location" : 
              {"value" : co2_value, "context" :{ "lat" : lat_value, "lng": lng_value }}}
    return payload

## send the information to the cloud
def post_request(payload):
    # Creates the headers for the HTTP requests
    url = "http://things.ubidots.com"
    url = "{}/api/v1.6/devices/{}".format(url, DEVICE_LABEL)
    headers = {"X-Auth-Token": TOKEN, "Content-Type": "application/json"}

    # Makes the HTTP requests
    status = 400
    attempts = 0
    while status >= 400 and attempts <= 5:
        req = requesorseLz2NhgmdLq7od0NgkGSO2eoOtLts.post(url=url, headers=headers, json=payload)
        status = req.status_code
        attempts += 1
        time.sleep(1)

    # Processes results
    if status >= 400:
        print("[ERROR] Could not send data after 5 attempts, please check \
            your token credentials and internet connection")
        return False

    print("[INFO] Request made properly, your device is updated")
    return True



def main():
    prog  = re.compile("^[0-9]{1,3}\/[0-9]{1,3}\.[0-9]{4}\/[0-9]{1,3}\.[0-9]{4}\/[0-1]\\r\\n$")
    condition = None
    fix_value = '0'
    payload = ""
    while fix_value == '0' or condition==None : ##wait until a valid tram is receive
        print("[INFO] Waiting for data")
        ser = serial.Serial('/dev/ttyO2', 115200, timeout=None) ##open serial port
        data = ser.readline()   # read on the serial port
        ser.close()
        condition = prog.match(data)
        #print 'Trame receive from serial'
        #print(data)
        
        if prog.match(data): ##if the tram receive is valide
           print '[INFO] Tram valid '
           co2_value,lat_value,lng_value,fix_value = data.split("/")
           fix_value=fix_value.rstrip()
           print '[INFO] Data'
           print "Fix :"+fix_value
           print "CO2 :"+co2_value
           print "LATITUDE :"+lat_value
           print "LONGITUDE :"+lng_value
           if fix_value == '1': ## if GPS coordonnee are valide
               payload =  build_payload(co2_value,lat_value,lng_value)
        else:
            print '[INFO] Not matched'
    ##print payload
    print("[INFO] Attemping to send data")
    post_request(payload) ##send data to the cloud
    print("[INFO] Finished")


if __name__ == '__main__':
    while (True):
        main()
        time.sleep(1)
