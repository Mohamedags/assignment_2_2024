"""
.. module:: assignment_2_2024 
  :noindex:
  :platform: Unix
  :synopsis: python module for the assignment_2_2024 package

.. moduleauthor:: Mohamedags

Version:
    1.0

Date:
    28/03/2025

Details:
    - **Subscribes to**: `/reaching_goal/goal`
    - **Provides Service**: `/last_goal`

Description:
    This module implements a ROS service that listens for navigation goals sent to the action server.
    It stores the last received goal and provides it upon request through a service.
"""

#!/usr/bin/env python3

import rospy
from assignment_2_2024.srv import LastTarget, LastTargetResponse
from assignment_2_2024.msg import PlanningActionGoal  # Goal message type

class TargetServiceNode:
    """
    A ROS node that tracks and provides the last received navigation goal.
    """
    def __init__(self):
        """
        Initializes the target service node, sets up a service and subscriber.
        """
        self.last_x = 0.0
        self.last_y = 0.0

        rospy.Service('/last_goal', LastTarget, self.handle_last_target_request)
        rospy.Subscriber('/reaching_goal/goal', PlanningActionGoal, self.update_last_target)
        rospy.loginfo("Target service initialized.")

    def update_last_target(self, goal_msg):
        """
        Callback to process goal messages from the action server.
        Extracts 'target_pose' from the goal message and updates the last known coordinates.

        Args:
            goal_msg (PlanningActionGoal): The received goal message containing target position.
        """
        rospy.loginfo("New goal received.")

        if hasattr(goal_msg, 'target_pose') and hasattr(goal_msg.target_pose, 'pose'):
            self.last_x = goal_msg.target_pose.pose.position.x
            self.last_y = goal_msg.target_pose.pose.position.y
            rospy.loginfo(f"Updated last target: x={self.last_x}, y={self.last_y}")
        else:
            rospy.logwarn("The goal message does not contain 'target_pose.pose'. Unable to update last target.")

    def handle_last_target_request(self, req):
        """
        Service callback that returns the last stored goal coordinates.

        Args:
            req (LastTargetRequest): Service request (unused, as the response does not depend on the request data).

        Returns:
            LastTargetResponse: Response containing the last recorded target coordinates.
        """
        rospy.loginfo(f"Request received for last target: x={self.last_x}, y={self.last_y}")
        return LastTargetResponse(target_x=self.last_x, target_y=self.last_y)

if __name__ == "__main__":
    rospy.init_node('Target_service')
    service_node = TargetServiceNode()
    rospy.spin()

