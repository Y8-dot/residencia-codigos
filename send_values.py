import urequests
import network
import time
from machine import Pin, ADC

ssid = "your SSID"
password = "the ssid password"
firebase_url = "your firebase url database"
firebase_key = "your key from firebase"

sensor1_pin = Pin(4, Pin.IN)
sensor2_pin = Pin(5, Pin.IN)
sensor3_pin = Pin(6, Pin.IN)
adc = ADC(0)


def connect_wifi():
    sta_if = network.WLAN(network.STA_IF)
    if not sta_if.isconnected():
        print("Connecting to WiFi...")
        sta_if.active(True)
        sta_if.connect(ssid, password)
        while not sta_if.isconnected():
            pass
    print("Connected to WiFi")


def send_to_firebase(sensor1, sensor2, sensor3):
    url = firebase_url + "data.json?auth=" + firebase_key
    data = {
        "sensor1": sensor1,
        "sensor2": sensor2,
        "sensor3": sensor3
    }
    headers = {"Content-Type": "application/json"}
    response = urequests.post(url, json=data, headers=headers)
    print("Firebase Response:", response.text)


connect_wifi()

while True:
    sensor1_value = sensor1_pin.value()
    sensor2_value = sensor2_pin.value()
    sensor3_value = sensor3_pin.value()

    send_to_firebase(sensor1_value, sensor2_value, sensor3_value)
    time.sleep(0.5)

