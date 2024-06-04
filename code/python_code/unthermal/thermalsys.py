# Required libraries
# import matplotlib
# matplotlib.use("TkAgg", force=True)
import paho.mqtt.client as mqtt
import control as ct
import struct
from queue import Queue
from math import ceil
import json
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
from scipy.signal import cont2discrete

#


# parameters of communication


BROKER = "192.168.0.3" # "18.204.70.207" # "192.168.0.3"
PORT = 1883
USER = "hpdesktop"
PASSWORD = "hpdesktop"

#topics for subscribing

PLANT_NUMBER = "1234"
codes ={"SYS_USER_SIGNALS_CLOSED"  : "/thermal/thermal_" + PLANT_NUMBER + "/user/sig_closed",
        "SYS_USER_SIGNALS_OPEN"  : "/thermal/thermal_" + PLANT_NUMBER + "/user/sig_open",
        "USER_SYS_SET_REF"  : "/thermal/user/thermal_" + PLANT_NUMBER + "/set_ref",
        "USER_SYS_SET_PID"  : "/thermal/user/thermal_" + PLANT_NUMBER  + "/set_pid",
        "USER_SYS_STEP_CLOSED": "/thermal/user/thermal_" + PLANT_NUMBER +"/step_closed",
        "USER_SYS_STAIRS_CLOSED": "/thermal/user/thermal_" + PLANT_NUMBER + "/stairs_closed",
        "USER_SYS_PRBS_OPEN": "/thermal/user/thermal_" + PLANT_NUMBER + "/prbs_open",
        "USER_SYS_STEP_OPEN": "/thermal/user/thermal_" + PLANT_NUMBER + "/step_open",
        "USER_SYS_SET_GENCON": "/thermal/user/thermal_" + PLANT_NUMBER + "/set_gencon",
        "USER_SYS_PROFILE_CLOSED": "/thermal/user/thermal_" + PLANT_NUMBER + "/prof_closed",
        "THERMAL_SAMPLING_TIME" : 0.8
        }


PATH_DEFAULT = r"./experiment_files/"
PATH_DATA = str(Path(__file__).parent) + r"/datafiles/"
FONT_SIZE = 10

Path(PATH_DEFAULT).mkdir(exist_ok=True)

class ThermalSystemIoT:

    def __init__(self, broker_address = BROKER, port= PORT, user=USER, password=PASSWORD,  client_id="", clean_session=True):
        self.client = mqtt.Client()
        self.broker_address = broker_address
        self.port = port
        self.client.on_connect = self.on_connect
        self.client.on_disconnect = self.on_disconnect
        self.client.on_message = self.on_message
        self.client.on_subscribe = self.on_subscribe
        self.client.on_publish = self.on_publish
        self.codes = codes


    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            print("Successfully connected to MQTT Broker " + self.broker_address)
        else:
            print("Failed to connect, return code %d\n", rc)

    def on_disconnect(self, client, userdata, rc):
        if rc != 0:
            print("Unexpected disconnection.")

    def on_message(self, client, userdata, message):
        print(f"Received  '{message.payload.decode()}'")

    def on_subscribe(self, client, userdata, mid, granted_qos):
        print("Subscribed: ", mid, " ", granted_qos)

    def on_publish(self, client, userdata, mid):
        print("Message Published: ", mid)

    def connect(self):
        self.client.username_pw_set(USER, PASSWORD)
        self.client.connect(self.broker_address, self.port)
        self.client.loop_start()

    def disconnect(self):
        self.client.loop_stop()
        self.client.disconnect()

    def subscribe(self, topic, qos=2):
        self.client.subscribe(topic, qos)

    def publish(self, topic, message, qos=2):
        self.client.publish(topic, message, qos)



    def transfer_function(self, temperature=50):
        Kp = -0.0025901782151786 * temperature + 0.987094648761147
        Tao = -0.0973494029141449 * temperature + 66.5927276606595
        delay = -0.00446863636363636 * temperature + 3.57201818181818
        uN = 0.9719 * temperature - 24.7355
        G = ct.TransferFunction(Kp, [Tao, 1])
        return G, delay, uN

