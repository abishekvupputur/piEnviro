#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import time
import json
import threading
import subprocess
from datetime import datetime
from collections import deque
from queue import Queue, Empty

import numpy
import pytz
from PIL import ImageFont
from smbus2 import SMBus
import paho.mqtt.client as mqtt

from bme280 import BME280
from ltr559 import LTR559
from enviroplus.noise import Noise
from fonts.ttf import RobotoMedium as UserFont
from astral import LocationInfo

from pms5003 import PMS5003, ReadTimeoutError as pmsReadTimeoutError
from enviroplus import gas

# ---------------------------- Configuration ----------------------------
MQTT_BROKER = "192.168.8.236"
MQTT_PORT = 1883
MQTT_TOPIC = "sensor/environment"
CITY = LocationInfo("Munich", "Germany", "Europe/Berlin", 48.1351, 11.5820)
TIME_ZONE = CITY.timezone

TPH_RATE = 5
DISPLAY_RATE = 1
MQTT_RATE = 2
NOISE_RATE = 1
AIR_RATE = 1/(2*60)

TPH_INTERVAL = 1.0 / TPH_RATE
DISPLAY_INTERVAL = 1.0 / DISPLAY_RATE
MQTT_INTERVAL = 1.0 / MQTT_RATE
NOISE_INTERVAL = 1.0 / NOISE_RATE
AIR_INTERVAL = 1.0 / AIR_RATE

# ---------------------------- Global Queues ----------------------------
data_queue = Queue(maxsize=1)
noise_queue = Queue(maxsize=1)
air_queue = Queue(maxsize=1)

# ---------------------------- Initialisation ----------------------------
bus = SMBus(1)
bme280 = BME280(i2c_dev=bus)
ltr559 = LTR559()
noise = Noise()
pms5003 = PMS5003()

client = mqtt.Client()
connected = False

font_sm = ImageFont.truetype(UserFont, 12)
font_lg = ImageFont.truetype(UserFont, 14)
margin = 3

cpu_temps = deque([0] * 5, maxlen=5)
pressure_vals = []
time_vals = []
trend = "-"
num_vals = 1000
factor = 1.31

path = os.path.dirname(os.path.realpath(__file__))

# ---------------------------- Utility Functions ----------------------------
def get_cpu_temperature():
    with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
        return int(f.read()) / 1000.0

def correct_humidity(h, t, corr_t):
    dewpoint = t - ((100 - h) / 5)
    return min(100, 100 - (5 * (corr_t - dewpoint)))

def analyse_pressure(p, t):
    global trend
    if len(pressure_vals) >= num_vals:
        pressure_vals.pop(0)
        time_vals.pop(0)
    pressure_vals.append(p)
    time_vals.append(t)
    return numpy.mean(pressure_vals)

def is_ip_reachable(ip, timeout=1):
    try:
        output = subprocess.run(
            ["ping", "-c", "1", "-W", str(timeout)],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )
        return output.returncode == 0
    except Exception:
        return False

def ensure_mqtt_connection():
    global connected
    try:
        if not client.is_connected():
            client.connect(MQTT_BROKER, MQTT_PORT, 60)
            client.loop_start()
            connected = True
            print("MQTT connection RESTARTED")
    except Exception:
        connected = False
        print("MQTT connection failed, retrying...")

# ---------------------------- Worker Threads ----------------------------
def pm_gas_worker():
    while True:
        start = time.time()
        payload = {"timestamp": datetime.now(pytz.timezone(TIME_ZONE)).isoformat()}

        try:
            pms_data = pms5003.read()
            payload.update({
                "pm1": pms_data.pm_ug_per_m3(1.0),
                "pm25": pms_data.pm_ug_per_m3(2.5),
                "pm10": pms_data.pm_ug_per_m3(10)
            })
        except pmsReadTimeoutError:
            payload.update({"pm1": None, "pm25": None, "pm10": None})

        try:
            gas_data = gas.read_all()
            payload.update({
                "oxidised": round(gas_data.oxidising / 1000, 3),
                "reduced": round(gas_data.reducing / 1000, 3),
                "nh3": round(gas_data.nh3 / 1000, 3)
            })
        except Exception:
            payload.update({"oxidised": None, "reduced": None, "nh3": None})

        if not air_queue.full():
            air_queue.put(payload, block=False)

        time.sleep(max(0, AIR_INTERVAL - (time.time() - start)))

def tph_worker():
    while True:
        start = time.time()
        temp = bme280.get_temperature()
        cpu = get_cpu_temperature()
        cpu_temps.append(cpu)
        avg_cpu = sum(cpu_temps) / len(cpu_temps)
        corr_temp = temp - 2.0

        hum = correct_humidity(bme280.get_humidity(), temp, corr_temp)
        pres = bme280.get_pressure()
        mean_pres = analyse_pressure(pres, time.time())
        light = ltr559.get_lux()

        payload = {
            "temperature": corr_temp,
            "humidity": hum,
            "pressure": mean_pres,
            "cpu_temperature": avg_cpu,
            "raw_temperature": temp,
            "light": light,
            "timestamp": datetime.now(pytz.timezone(TIME_ZONE))
        }

        if not data_queue.full():
            data_queue.put(payload, block=False)

        time.sleep(max(0, TPH_INTERVAL - (time.time() - start)))

def noise_worker():
    while True:
        start = time.time()
        amps = noise.get_amplitudes_at_frequency_ranges([(100, 500), (500, 1000), (1000, 1500)])
        scaled = [round(n * 32, 2) for n in amps]
        if not noise_queue.full():
            noise_queue.put(scaled, block=False)
        time.sleep(max(0, NOISE_INTERVAL - (time.time() - start)))

def mqtt_worker():
    latest_data = {}
    latest_noise = [0, 0, 0]
    latest_air = {}

    while True:
        start = time.time()
        try:
            latest_data.update(data_queue.get(timeout=MQTT_INTERVAL))
        except Empty:
            pass

        try:
            latest_noise = noise_queue.get_nowait()
        except Empty:
            pass

        try:
            latest_air = air_queue.get_nowait()
        except Empty:
            pass

        if latest_data:
            payload = {
                "temperature": round(latest_data.get("temperature", 0), 2),
                "humidity": round(latest_data.get("humidity", 0), 2),
                "pressure": round(latest_data.get("pressure", 0), 2),
                "light": round(latest_data.get("light", 0), 2),
                "cpu_temperature": round(latest_data.get("cpu_temperature", 0), 2),
                "raw_temperature": round(latest_data.get("raw_temperature", 0), 2),
                "sound_levels": {
                    "low": latest_noise[0],
                    "mid": latest_noise[1],
                    "high": latest_noise[2]
                },
                "timestamp": latest_data.get("timestamp", datetime.now()).isoformat()
            }

            if latest_air:
                payload["air_quality"] = {
                    "pm1": latest_air.get("pm1"),
                    "pm25": latest_air.get("pm25"),
                    "pm10": latest_air.get("pm10"),
                    "oxidised": latest_air.get("oxidised"),
                    "reduced": latest_air.get("reduced"),
                    "nh3": latest_air.get("nh3"),
                    "air_timestamp": latest_air.get("timestamp")
                }

            if client.is_connected():
                client.publish(MQTT_TOPIC, json.dumps(payload))
            else:
                ensure_mqtt_connection()

        time.sleep(max(0, MQTT_INTERVAL - (time.time() - start)))

def mqtt_watchdog():
    while True:
        if is_ip_reachable(MQTT_BROKER):
            ensure_mqtt_connection()
        else:
            global connected
            connected = False
        time.sleep(5)

# ---------------------------- Main Execution ----------------------------
threading.Thread(target=tph_worker, daemon=True).start()
threading.Thread(target=noise_worker, daemon=True).start()
threading.Thread(target=mqtt_worker, daemon=True).start()
threading.Thread(target=pm_gas_worker, daemon=True).start()
threading.Thread(target=mqtt_watchdog, daemon=True).start()

while True:
    time.sleep(1)
