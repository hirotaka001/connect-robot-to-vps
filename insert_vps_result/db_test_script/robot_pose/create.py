#!/bin/python3

import psycopg2

connection = psycopg2.connect("host=172.19.73.134 port=5432 dbname=rbag user=rbag password=rbag123456")
cur = connection.cursor()

cur.execute("CREATE TABLE robot_pose(id serial, time timestamp with time zone NOT NULL DEFAULT now(), posx float NOT NULL, posy float NOT NULL, posz float NOT NULL, rotx float NOT NULL, roty float NOT NULL, rotz float NOT NULL, rotw float NOT NULL, PRIMARY KEY (id))")

#print("[after]")
#cur.execute("select * robot_position")

# commit
connection.commit()

# close
cur.close()
connection.close()
