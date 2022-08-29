#!usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import msgpack
import paho.mqtt.client as mqtt     

import psycopg2
import numpy as np
import math
from pyquaternion import Quaternion
from scipy.spatial.transform import Rotation as R

def on_connect(client, userdata, flag, rc):
  print("Connected with result code " + str(rc)) 
  client.subscribe(userdata[2]) 


def on_disconnect(client, userdata, rc):
  if  rc != 0:
    print("Unexpected disconnection.")


def on_message(client, userdata, msg):
  # msg.topicにトピック名が，msg.payloadに届いたデータ本体が入っている
  msgs = msgpack.unpackb(msg.payload)
  insert_db_from_vps_mqtt_msg(userdata, msgs)


def quaternion_to_euler_zxy(q):
    r = R.from_quat([q[0], q[1], q[2], q[3]])
    return r.as_euler('zxy', degrees=True)


def insert_db_from_vps_mqtt_msg(userdata, msgs):
 
  status = msgs["data"]["status"]
  message = msgs["data"]["message"]
  if status == 0:
    print("vps result is failed. so after process will skip")
    return

  quat = msgs["data"]["rotation"]
  pos = msgs["data"]["position"]
  print(f'[POS]:{pos}')

  table_name, connection, *others = userdata
  cur = connection.cursor()
  posx, posy, posz = pos

  q = [-quat[1], -quat[2], -quat[3], quat[0]] # 右手系左手系の変換
  euler = quaternion_to_euler_zxy(q)
  print('[Quaternion(xyzw)]:', q)
  print('[Euler]:', euler)

  cur.execute("INSERT INTO " + table_name + " (posx,posy,posz,rotx,roty,rotz,rotw) values (%s,%s,%s,%s,%s,%s,%s)", (posx,posy,posz,euler[1],euler[2],euler[0],0.0))
  # commit
  connection.commit()

  cur.execute("select * from " + table_name)

  # if you want to check result, use below code
  #for row in cur:
  #  print(row)

  print('data lenght:', len(cur.fetchall()))

def simple_db_test(db_cur, db_connection, table_name):
  temp_dict = {
    "data": {
      "status": 1,
      "message": "test_message",
      "rotation": [-0.033414,
        -0.006612,
        0.99797,
        -0.05381],
      "position": [2.440409,
        1.217573,
        9.248654],
    }
  }
  insert_db_from_vps_mqtt_msg([table_name, db_connection], temp_dict)
  print("[After]")
  db_cur.execute("select * from " + table_name)
  for row in db_cur:
    print(row)

def main():
  args = sys.argv

  if len(args) != 2:
    print("Usage: python sub.py {$1 MQTT Topic}")
    exit(0)

  print('Subscribe Topic:', args[1]) # MQTT topic

  # DB Setting
  table_name = 'robot_pose'
  db_connection = psycopg2.connect("host=172.19.73.134 port=5432 dbname=rbag user=rbag password=rbag123456")
  db_cur = db_connection.cursor()
  db_cur.execute("truncate table " + table_name + " restart identity")
  db_connection.commit()

  # DB test code
  if 0:
    simple_db_test()

  # MQTT Setting
  client = mqtt.Client()
  client.on_connect = on_connect
  client.on_disconnect = on_disconnect
  client.on_message = on_message
  client.user_data_set([table_name, db_connection, args[1]])
  client.username_pw_set("rbag","rbag")

  # Connect MQTT server
  client.connect("172.19.73.190", 1883, 60)
  client.loop_forever()

  db_cur.close()
  db_connection.close()

if __name__ == '__main__':
  main()
