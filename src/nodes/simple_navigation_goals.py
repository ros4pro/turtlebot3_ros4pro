#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import (
    MoveBaseActionGoal,
    MoveBaseAction,
    MoveBaseGoal,
    MoveBaseActionFeedback,
)
from actionlib_msgs.msg import GoalStatus
import math
from math import radians, degrees

from geometry_msgs.msg import Point
import tf
import traceback
import time
import sys

from nav_msgs.msg import Path


class InitException(Exception):
    pass


# Useful doc :
# http://docs.ros.org/jade/api/actionlib/html/classactionlib_1_1simple__action__client_1_1SimpleActionClient.html


class SimpleNavigationGoals:
    """Interface with the move_base actionlb. Allows to set a position goal (x, y, theta) that will be followed by a robot, using a predefined GlobalPlanner and LocalPlanner

    Raises:
        InitException --
    """

    def __init__(self, hmi_wrapper=None):
        rospy.loginfo("Starting simple_navigation_goals...")

        # Created the client
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for the action server to come up...")
        success = self.move_base.wait_for_server(rospy.Duration(5))
        if not (success):
            rospy.logerr(
                "The move_base action server did not respond in time, exiting."
            )
            raise InitException("Failed SimpleNavigationGoals init")
        rospy.Subscriber("move_base/feedback", MoveBaseActionFeedback, self._feedback)

        # Defines the goal position
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.stamp = rospy.Time.now()
        # Set up the frame parameters
        self.goal.target_pose.header.frame_id = "map"

        # A (x, y, theta) list saving the current position that is feedbacked from AMCL. This is only updated while the movement has not finished
        self.feedback_pos = [0, 0, 0]

        # Length of the last path computed by the global planner
        self.remaining_distance = -1
        rospy.Subscriber("/move_base/GlobalPlanner/plan", Path, self._compute_remaining_distance)
        # No need to fetch the result of the move_base action_lib, it's empty.

        if (hmi_wrapper == None) :
            rospy.logwarn(
                "hmi_wrapper NOT SPECIFIED"
            )
        self.hmi = hmi_wrapper

    def _shutdown(self):
        rospy.loginfo("Cancelling all SimpleNavigationGoals")
        self.move_base.cancel_all_goals()

    def _feedback(self, feedback):
        data = feedback.feedback.base_position.pose

        self.feedback_pos[0] = data.position.x
        self.feedback_pos[1] = data.position.y
        quaternion = (
            data.orientation.x,
            data.orientation.y,
            data.orientation.z,
            data.orientation.w,
        )
        self.feedback_pos[2] = tf.transformations.euler_from_quaternion(quaternion)[2]
        rospy.logdebug(
            "x:{}, y:{}, theta:{}".format(
                self.feedback_pos[0], self.feedback_pos[1], self.feedback_pos[2]
            )
        )

    def _compute_remaining_distance(self, path):      
        tmp = 0
        i = 1
        if (len(path.poses) < 1) :
            tmp = -1
        while i < len(path.poses):
            tmp += math.sqrt(math.pow(path.poses[i-1].pose.position.x - path.poses[i].pose.position.x, 2) + math.pow(path.poses[i-1].pose.position.y - path.poses[i].pose.position.y, 2))
            i+=1
        self.remaining_distance = tmp

    def go_to(self, x, y, theta, wait_for_result=60, frame="map", blocking=True):
        """Sets a x, y, theta goal for the move_base to follow.

        Arguments:
            x {float} -- In meters
            y {float} -- In meters
            theta {float} -- In rads

        Keyword Arguments:
            wait_for_result {float} -- Time in seconds that the robot has to arrive to the goal before it's considered a failure (default: {60})
            frame {str} -- frame considered for the coordinates system (default: {"map"})

        Returns:
            Bool -- True if the goal was reached, False otherwise
        """

        rospy.loginfo("Asked to go to x={}, y={} and theta={}".format(x, y, theta))
        if (self.hmi != None) :
            self.hmi.working()
        self.remaining_distance = -1
        # Set up the frame parameters
        self.goal.target_pose.header.frame_id = frame
        self.goal.target_pose.header.stamp = rospy.Time.now()

        # Moving towards the goal
        self.goal.target_pose.pose.position = Point(x, y, 0)

        # Using Euler angles because our rotation is simple
        quat = tf.transformations.quaternion_from_euler(0, 0, theta)

        self.goal.target_pose.pose.orientation.x = quat[0]
        self.goal.target_pose.pose.orientation.y = quat[1]
        self.goal.target_pose.pose.orientation.z = quat[2]
        self.goal.target_pose.pose.orientation.w = quat[3]

        self.move_base.send_goal(self.goal)

        if (blocking == False) :
            return True
        self.move_base.wait_for_result(rospy.Duration(wait_for_result))
        if (self.hmi != None) :
            self.hmi.blank()
        if self.move_base.get_state() == GoalStatus.SUCCEEDED:
            rospy.loginfo(
                "Destination reached. The destination was x={}, y={} and theta={}".format(
                    x, y, theta
                )
            )
            return True

        else:
            rospy.loginfo(
                "The robot failed to reach the destination. State '{}'".format(
                    self.move_base.get_state()
                )
            )
            return False

    def is_arrived(self):
        """Returns True if move_base's state is RECALLED or REJECTED or PREEMPTED or ABORTED or SUCCEEDED or LOST
        Returns False otherwise (PENDING or ACTIVE)
        """
        state = self.move_base.get_state()
        rospy.loginfo("State = {}".format(state))
        if (state == GoalStatus.PENDING or state == GoalStatus.ACTIVE) :
            return False
        return True   

    # uint8 PENDING         = 0   # The goal has yet to be processed by the action server
    # uint8 ACTIVE          = 1   # The goal is currently being processed by the action server
    # uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing
    #                             #   and has since completed its execution (Terminal State)
    # uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)
    # uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due
    #                             #    to some failure (Terminal State)
    # uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,
    #                             #    because the goal was unattainable or invalid (Terminal State)
    # uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing
    #                             #    and has not yet completed execution
    # uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,
    #                             #    but the action server has not yet confirmed that the goal is canceled
    # uint8 RECALLED        = 8   # The goal received a cancel request before it started executing
    #                             #    and was successfully cancelled (Terminal State)
    # uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be
    #                             #    sent over the wire by an action server     

    def cancel_goal(self):
        """Cancels the current goal
        """
        if (self.hmi != None) :
            self.hmi.blank()
        self.move_base.cancel_goal()

    def cancel_all_goals(self):
        """Cancels all the goals
        """
        if (self.hmi != None) :
            self.hmi.blank()
        self.move_base.cancel_all_goals()


if __name__ == "__main__":
    try:
        rospy.init_node("simple_navigation_goals", anonymous=False)

        nav_goals = SimpleNavigationGoals()
        # What to do if shut down (e.g. ctrl + C or failure)
        rospy.on_shutdown(nav_goals._shutdown)

        while True:
            if not (nav_goals.go_to(1.5, -2.98, 0)):
                break
            if not (nav_goals.go_to(1.5, -2.98, math.pi / 2)):
                break
            if not (nav_goals.go_to(-2.33, -9.86, 0)):
                break

        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr(traceback.format_exc())

    rospy.loginfo("simple_navigation_goals node terminated.")
