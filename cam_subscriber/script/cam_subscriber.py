#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import os
import glob
import sys
import configparser
import tf
import json
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import PoseStamped
import geometry_msgs
from std_msgs.msg import String

from queue import Queue
from ROBO_CMD import ROBO_CMD
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge
import cv2

from vps_client import VpsUtil, VpsParam, VpsClient
from mqtt_publisher import MqttPublisher

from robo_commander import Turtlebot3CommandCreateor
from auto_driving_controller import AutoDrivingController

counter = 0
g_start = False
# use satic image
g_use_test_img = True
g_debug_img_count = 0

# execute when camera image arrived
def process_image(msg, user_data):
    global counter
    global g_start
    global g_use_test_img
    global g_debug_img_count
   
    vps_queue, driving_controller, debug_imgs = user_data

    try:
        bridge = CvBridge()
        org = bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imshow('cap robot', org)
        file_name = 'cap%04d.jpg' % counter
        # TODO ループ回数でなく経過時間で制御
        if counter % 1 == 0 and g_use_test_img == False:
            time_stamp = VpsUtil.c_timestamp()
            vps_queue.put([time_stamp, org, str(counter)])
        counter = counter + 1
        key = cv2.waitKey(1)

        # for debug
        if key == ord('d') and g_use_test_img == True:
            size = len(debug_imgs)
            img = debug_imgs[g_debug_img_count]

            # display file name on image windows
            cv2.imshow('debug', img[1])

            time_stamp = VpsUtil.c_timestamp()
            vps_queue.put([time_stamp, img[1], img[0]])
            g_debug_img_count = g_debug_img_count + 1
            if size <= g_debug_img_count:
                g_debug_img_count = 0

        if key == ord('s') and g_start == False:
            print('Start auto driving ...')
            driving_controller.start()
            g_start = True
    except KeyboardInterrupt:
        print('KEYBOARD INTERRUPT !')
        sys.exit
    except Exception as err:
        print (err)   

def start_node(topic_name, user_data):
    print('start ROS Topic' + topic_name + ' subscribe')
    rospy.loginfo('cam_subscriber node started')
    sub = rospy.Subscriber(topic_name, Image, process_image, user_data)
    
    rospy.spin()

# do when VPS result arrived.
def vps_callback(response, user_data):

    mqtt_publisher, vps_result_queue, pub_vps = user_data
    print(response)
    # only success
    if response['status'] == 1:
        # publish result to Grafana DB
        mqtt_publisher.send_msg('vps_result', response) 

        # publis vps result on ROS
        if -0.01 < response['confidence']:
            json_string = _convert_vps_to_ros_coord(response)
            pub_vps.publish(json_string)
        else:
            print('confidence is too low')

    # push vps_result_queue   
    vps_result_queue.put(response)

def quaternion_to_euler_zxy(q):
    r = R.from_quat([q[0], q[1], q[2], q[3]])
    return r.as_euler('zxy', degrees=True)

# convert VPS coord to ROS coord
def _convert_vps_to_ros_coord(response):
    # pos
    px, py, pz = response['position']
    py, pz, px = -px, py, -pz

    # rot
    rw, rx, ry, rz = response['rotation']
    q = [rx, ry, rz, rw] # 右手系左手系の変換
    e_vps_z, e_vps_x, e_vps_y = quaternion_to_euler_zxy(q)    

    # deg to radian
    e_vps_y = (e_vps_y + 180) * (3.14159265359 / 180)
    
    q_vps = tf.transformations.quaternion_from_euler(0, 0, e_vps_y)
    qx, qy, qz, qw = q_vps

    stamp = rospy.Time.now()
    json_obj = {
        "position":[px, py,pz], "rotation":[qx, qy, qz, qw], "map_id":response['map_id'], "status":response['status'], 
        "time_stamp_secs":str(stamp.secs), "time_stamp_nsecs":str(stamp.nsecs)
    }       
    return json.dumps(json_obj)

def driving_order_callback(move_direction, user_data):
    if move_direction != ROBO_CMD.NONE:
        # create move cmd
        cmd = user_data[1].create_move_cmd(move_direction)
        print('created cmd:', cmd)
        # publish cmd to robo
        user_data[0].send_msg('cmd_to_robot', cmd)

# MQTT callbacks
def mqtt_on_connect(client, userdata, flag, rc):
  print("Mqtt connected with result code " + str(rc))

def mqtt_on_disconnect(client, userdata, rc):
  if rc != 0:
     print("Mqtt Unexpected disconnection.")

def mqtt_on_publish(client, userdata, mid):
    mid = mid + 1

if __name__ == '__main__':

    vps_queue = Queue()
    vps_result_queue = Queue()

    rospy.init_node('cam_subscriber')

    g_use_test_img = False
    if rospy.has_param('use_test_img'):
        g_use_test_img = rospy.get_param('use_test_img')
    print('use_test_img:', g_use_test_img)

    pub_vps = rospy.Publisher('vps_pose', String, queue_size=0)

    # 画像データでテスト (ロボットのCameraノード起動が不要)
    debug_imgs = []
    if g_use_test_img :
        #wildcard = './test_img/check_tf_01/*.jpg'
        wildcard = './test_img/iphone2/*.jpg'
        files = sorted(glob.glob(wildcard))
        for file in files:
            debug_imgs.append((os.path.basename(file), cv2.imread(file)))

    # config parser
    config_ini = configparser.ConfigParser()
    config_ini.read('config.ini', encoding='utf-8')

    # mqtt setting
    mqtt_broker_url = config_ini['MQTT']['broker_url']
    mqtt_broker_user = config_ini['MQTT']['broker_user']
    mqtt_broker_password = config_ini['MQTT']['broker_pass']
    mqtt_publisher = MqttPublisher(mqtt_broker_url, mqtt_broker_user, mqtt_broker_password)
    mqtt_publisher.setCallback(mqtt_on_connect, mqtt_on_disconnect, mqtt_on_publish)
    mqtt_publisher.connect()

    cmd_creator = Turtlebot3CommandCreateor()

    vps_api = config_ini['VPS']['api']
    vps_map_id = config_ini['VPS']['map_id']
    vps_facal_lenght = float(config_ini['VPS']['focal_length'])
    vps_intrinsic_file_path = '' # use current
    vps_work_dir = config_ini['VPS']['work_dir']
    print(f'Use focal lenght:{vps_facal_lenght}')
   
    vps_param = VpsParam(vps_api, vps_map_id, vps_facal_lenght, vps_intrinsic_file_path, vps_work_dir)
    vps_client = VpsClient(vps_param)
    vps_client.run(vps_queue, vps_callback, [mqtt_publisher, vps_result_queue, pub_vps])

    drivingController = AutoDrivingController()
    drivingController.prepare(vps_result_queue, driving_order_callback, [mqtt_publisher, cmd_creator])

    topic_name = config_ini['ROS']['camera_topic_name']
    try:
        start_node(topic_name, user_data=[vps_queue, drivingController, debug_imgs])
    except rospy.ROSInterruptException:
        print('interrupte command')
        pass
