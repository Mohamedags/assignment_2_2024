#!/usr/bin/env python3

import rospy
from assignment_2_2024.srv import LastTarget, LastTargetResponse
from assignment_2_2024.msg import PlanningActionGoal  # Goal message type

class TargetServiceNode:
    def __init__(self):
        self.last_x = 0.0
        self.last_y = 0.0

        rospy.Service('/last_goal', LastTarget, self.handle_last_target_request)
        rospy.Subscriber('/reaching_goal/goal', PlanningActionGoal, self.update_last_target)  # Listen to /reaching_goal/goal
        rospy.loginfo("Target service initialized.")

    def update_last_target(self, goal_msg):
        """
        Callback to process goal messages from the action server.
        Extracts 'target_pose' from the goal message.
        """
        rospy.loginfo("New goal received.")
        
        # Correctly access the position of the target_pose
        if hasattr(goal_msg, 'target_pose') and hasattr(goal_msg.target_pose, 'pose'):
            self.last_x = goal_msg.target_pose.pose.position.x
            self.last_y = goal_msg.target_pose.pose.position.y
            rospy.loginfo(f"Updated last target: x={self.last_x}, y={self.last_y}")
        else:
            rospy.logwarn("The goal message does not contain 'target_pose.pose'. Unable to update last target.")

    def handle_last_target_request(self, req):
        """
        Service callback to provide the last target coordinates upon request.
        """
        rospy.loginfo(f"Request received for last target: x={self.last_x}, y={self.last_y}")
        return LastTargetResponse(target_x=self.last_x, target_y=self.last_y)

if __name__ == "__main__":
    rospy.init_node('Target_service')
    service_node = TargetServiceNode()
    rospy.spin()

