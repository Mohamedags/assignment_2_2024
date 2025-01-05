#!/usr/bin/env python3

import rospy
import actionlib
from nav_msgs.msg import Odometry
from assignment_2_2024.msg import RobotPoseVelocity
from assignment_2_2024.msg import PlanningAction, PlanningGoal
from assignment_2_2024.srv import LastTarget, LastTargetResponse

class ActionClientNode:
    def __init__(self):
        rospy.init_node('Action_client')

        # Initialize action client
        self.goal_client = actionlib.SimpleActionClient('/reaching_goal', PlanningAction)
        rospy.loginfo("Connecting to the action server...")
        self.goal_client.wait_for_server()
        rospy.loginfo("Connected to the action server.")

        # Publisher for robot status
        self.robot_state_pub = rospy.Publisher('/robot_status', RobotPoseVelocity, queue_size=10)
        
        # Subscriber for odometry updates
        rospy.Subscriber('/odom', Odometry, self.process_odometry)

        # Service for retrieving the last goal
        self.last_target_service = rospy.Service('/last_goal', LastTarget, self.handle_last_target_request)

        # Variables to store the last goal coordinates
        self.latest_x = None
        self.latest_y = None

    def send_goal(self, x, y):
        goal = PlanningGoal()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        self.goal_client.send_goal(goal)
        self.latest_x = x
        self.latest_y = y
        rospy.loginfo(f"Goal dispatched: x={x}, y={y}")

    def cancel_goal(self):
        rospy.loginfo("Cancelling the current goal.")
        self.goal_client.cancel_goal()

    def process_odometry(self, odometry_data):
        state_msg = RobotPoseVelocity()
        state_msg.x = odometry_data.pose.pose.position.x
        state_msg.y = odometry_data.pose.pose.position.y
        state_msg.vel_x = odometry_data.twist.twist.linear.x
        state_msg.vel_z = odometry_data.twist.twist.angular.z
        self.robot_state_pub.publish(state_msg)

    def handle_last_target_request(self, request):
        rospy.loginfo(f"Returning last target coordinates: x={self.latest_x}, y={self.latest_y}")
        return LastTargetResponse(target_x=self.latest_x, target_y=self.latest_y)

    def goal_loop(self):
        while not rospy.is_shutdown():
            try:
                x = float(input("Enter the x-coordinate for the goal: "))
                y = float(input("Enter the y-coordinate for the goal: "))
                self.send_goal(x, y)

                rospy.loginfo("Waiting for the goal to complete...")
                self.goal_client.wait_for_result()
                status = self.goal_client.get_state()

                if status == actionlib.GoalStatus.SUCCEEDED:
                    rospy.loginfo("Goal achieved successfully.")
                elif status == actionlib.GoalStatus.PREEMPTED:
                    rospy.logwarn("Goal was canceled.")
                else:
                    rospy.logerr(f"Goal failed with status: {status}")

                if input("Set another goal? (yes/no): ").lower() != 'yes':
                    rospy.loginfo("Exiting goal loop.")
                    break

            except ValueError:
                rospy.logerr("Invalid input. Please enter numerical values for x and y.")

if __name__ == '__main__':
    try:
        node = ActionClientNode()
        node.goal_loop()
    except rospy.ROSInterruptException:
        pass
