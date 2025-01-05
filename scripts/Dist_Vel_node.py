#!/usr/bin/env python3

import rospy
import math
import time
from assignment_2_2024.msg import RobotPoseVelocity
from assignment_2_2024.srv import DistSpeed, DistSpeedResponse

def compute_robot_metrics(data):
    global last_time, update_interval, current_distance, avg_speed
    now = time.time()

    if now - last_time >= update_interval:
        target_x = rospy.get_param("des_pos_x", 0.0)
        target_y = rospy.get_param("des_pos_y", 0.0)

        current_distance = math.sqrt((target_x - data.x) ** 2 + (target_y - data.y) ** 2)
        avg_speed = math.sqrt(data.vel_x ** 2 + data.vel_z ** 2)

        rospy.loginfo(f"Distance to target: {current_distance:.2f} m")
        rospy.loginfo(f"Average speed: {avg_speed:.2f} m/s")
        last_time = now

def provide_metrics(req):
    global current_distance, avg_speed
    return DistSpeedResponse(distance=current_distance, average_speed=avg_speed)

if __name__ == "__main__":
    rospy.init_node('Dist_Vel_node')

    last_time = time.time()
    update_interval = 1.0  # seconds
    current_distance = 0.0
    avg_speed = 0.0

    rospy.Service('dist_vel_from_target', DistSpeed, provide_metrics)
    rospy.Subscriber('/robot_status', RobotPoseVelocity, compute_robot_metrics)

    rospy.spin()
