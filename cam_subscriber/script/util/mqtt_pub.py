#!usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import paho.mqtt.client as mqtt     # MQTTのライブラリをインポート
from time import sleep              # 3秒間のウェイトのために使う
import msgpack
 
# ブローカーに接続できたときの処理
def on_connect(client, userdata, flag, rc):
    print("Connected with result code " + str(rc))

# ブローカーが切断したときの処理
#def on_disconnect(client, userdata, flag, rc):
def on_disconnect(client, userdata, rc):
    if rc != 0:
        print("Unexpected disconnection.")

# publishが完了したときの処理
def on_publish(client, userdata, mid):
    print("publish: {0}".format(mid))

# メイン関数   この関数は末尾のif文から呼び出される
def main(args):
    client = mqtt.Client()                 # クラスのインスタンス(実体)の作成
    client.on_connect = on_connect         # 接続時のコールバック関数を登録
    client.on_disconnect = on_disconnect   # 切断時のコールバックを登録
    client.on_publish = on_publish         # メッセージ送信時のコールバック
    client.username_pw_set("rbag","rbag")
    client.connect("172.19.73.190", 1883, 60)  # 接続先は自分自身

    # 通信処理スタート
    client.loop_start()    # subはloop_forever()だが，pubはloop_start()で起動[>

    # 永久に繰り返す
    #  while True:
    #    client.publish("test/001","Hello, Drone!")    # トピック名とメッセージ[>
    #    sleep(3)   # 3秒待つ
    #  msg=bytearray(msgpack.packb('{"linear": {"x": 0, "y": 0, "z": 0}, "angula>
    msg=bytearray(msgpack.packb({"data": str(args[2])}, use_bin_type=False))
    client.publish(str(args[1]), msg)    # トピック名とメッセージを決めて送信
    sleep(3)
if __name__ == '__main__':          # importされないときだけmain()を呼ぶ
    args = sys.argv
    if len(args) != 3:
        print("Usage: python pub.py {$1 MQTT Topic} {$2 Message}")
        print("  - $2 Message Format : RobotID|Command|Args|")
        print("[ Command ]")
        print("  rostopic list: Get a list of topics.")
        print("  bridge: Specify the topic to bridge.")
        print("    [ Args ]")
        print("      Topic,Topic,...")
        print("  move: Move the robot.")
        print("    [ Args ]")
        print("      forward/backward/left/right/stop")
        exit(0)

    print('Topic: ' + str(args[1])) # MQTT topic
    print('  Msg: ' + str(args[2])) # Msg
    main(args)    # メイン関数を呼び出す
