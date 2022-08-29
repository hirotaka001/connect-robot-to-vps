import sys
import msgpack
import paho.mqtt.client as mqtt 

class MqttPublisher:
    def __init__(self, broker_url, user_name, pw):
        self.__client = mqtt.Client()
        self.__broker_url = broker_url
        self.__client.username_pw_set(user_name, pw)  
        return

    def setCallback(self, on_connect, on_disconnect, on_publish):
        self.__client.on_connect = on_connect
        self.__client.on_disconnect = on_disconnect
        self.__client.on_publish = on_publish

    def connect(self):
        self.__client.connect(self.__broker_url, 1883, 60)
        self.__client.loop_start()

    def send_msg(self, topic, msg):
        send_msg = bytearray(msgpack.packb({"data": msg}, use_bin_type=False))
        self.__client.publish(topic, send_msg)