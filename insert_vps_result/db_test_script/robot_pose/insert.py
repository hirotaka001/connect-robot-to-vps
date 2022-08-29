#!/bin/python3

import psycopg2
import time
import math
import random

connection = psycopg2.connect("host=172.19.73.134 port=5432 dbname=rbag user=rbag password=rbag123456")
cur = connection.cursor()
print("connect db [rbag]")

idx = 0
z = 1.2

# database 削除
print("delete current table [robot_pose]...")
cur.execute("truncate table robot_pose restart identity")

# ランダムな値を書き込み続ける
while(True):
    dia = math.sin(math.radians(idx))
    x = 600
    y = 100 * dia
    idx = idx + 1

    cur.execute("INSERT INTO robot_pose (posx,posy,posz,rotx,roty,rotz,rotw) values (%s,%s,%s, %s,%s,%s,%s)", (x,y,z,idx,idx+1,idx+2,idx+3))

    # commit
    connection.commit()

    #print("insert data ", x, y, z)
    time.sleep(1)


    print("[after]")
    cur.execute("select * from robot_pose")
    for row in cur:
        print(row)

# close
cur.close()
connection.close()
