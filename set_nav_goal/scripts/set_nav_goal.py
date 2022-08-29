#!/usr/bin/env python3

import rospy
import actionlib
import tf
from nav_msgs.msg import Odometry
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# waypoints = [
#     [(0.0, 1.0, 0.0),(0.0,0.0, 0.677109689418,  0.735882102308)],
#     [(-1.97500002384,  0.370000123978, 0.0),(0.0,0.0 ,1.0,             7.54978995489e-08)],
#     [(-2.35499978065, -0.009999709203, 0.0),(0.0,0.0, -0.652555875509, 0.757740608215)]
# ]

# 1.49927 -1.68537 3.00154
# 2.2891 -1.45013 -2.84132
# 2.71078 -1.83939 2.37524
# 0.77896 -1.48631 -0.441921

# waypoints = [  
#     [(1.1,  0.1, 0.0),(0.0,0.0 ,0.08, 0.99)],
#     [(1.8, 0.35, 0.0),(0.0,0.0, 0.09, 0.99)],
# ]

waypoints = [  
    [(0.484,  0.059, 0.0),(0.0,0.0 ,-5.92, 0.99)],
    [(1.57, 0.159, 0.0),(0.0,0.0, 0.009, 0.99)],
    [(2.624, 0.239, 0.0),(0.0,0.0, 0.1538, 0.988)],
    [(4.014, 1.089, 0.0),(0.0,0.0, 0.707, 0.707)],
    [(3.644, 2.609, 0.0),(0.0,0.0, 0.999, -4.37)],
    [(2.164, 2.549, 0.0),(0.0,0.0, -0.698, 0.715)],
    [(2.104, 0.559, 0.0),(0.0,0.0, -0.707, 0.707)],
    [(0.074, 0.220, 0.0),(0.0,0.0, -0.999, 0.018)],
]


def goal_pose(pose):
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = 'map'
    goal_pose.target_pose.pose.position.x = pose[0][0]
    goal_pose.target_pose.pose.position.y = pose[0][1]
    goal_pose.target_pose.pose.position.z = pose[0][2]
    goal_pose.target_pose.pose.orientation.x = pose[1][0]
    goal_pose.target_pose.pose.orientation.y = pose[1][1]
    goal_pose.target_pose.pose.orientation.z = pose[1][2]
    goal_pose.target_pose.pose.orientation.w = pose[1][3]

    return goal_pose

link = 'base_link'
if __name__ == '__main__':

    print('START set_nav_goal')
    rospy.init_node('patrol')
    listener = tf.TransformListener()

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    listener.waitForTransform("map", link, rospy.Time(), rospy.Duration(4.0))

    # pose = [(0.3430511475,1.70947879314,0.0),(0.0, 0.0, 0.712479235583, 0.701693194255)]
    # goal = goal_pose(pose)
    # client.send_goal(goal)

    while True:
        for pose in waypoints: 
            print('waypoint:', pose)

            now = rospy.Time.now()
            listener.waitForTransform("odom", "base_footprint", now, rospy.Duration(4.0))
            position2, quaternion2 = listener.lookupTransform("odom", "base_footprint", now)

            pos, quat = pose
            print('p:', pos, position2)
            pos += tuple(position2)
            print(pos, position2)
            #quat *= quaternion2
            new_pose = [pos, quat]

            goal = goal_pose(new_pose)
            client.send_goal(goal)
            while True:
                now = rospy.Time.now()
                listener.waitForTransform("map", link, now, rospy.Duration(4.0))

                # current position(map coordinate) from tf
                position, quaternion = listener.lookupTransform("map", link, now)

                # check distance to waypoint. if less than 30cm, go to next waypoint
                if(math.sqrt((position[0]-goal.target_pose.pose.position.x)**2 + (position[1]-goal.target_pose.pose.position.y)**2 ) <= 0.3):
                    print("next!!")
                    break

                else:
                    rospy.sleep(0.25)

    print('END set_nav_goal')


