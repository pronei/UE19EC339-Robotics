#!/usr/bin/env python3

from __future__ import print_function

import rospy
import sys

# import our interfaces from srv and msg folders
from action_tutorial.srv import CounterGoal, CounterResult
from action_tutorial.msg import Counter

# TODO: change this
ASSIGNMENT = False


# the command to run should be line this `rosrun action_tutorial counter_action_client.py <stop_value> <start_value> <rate>`
def usage():
    str = "%s <counter_end_value>"%sys.argv[0]
    return str if not ASSIGNMENT else str + " <counter_start_value> <rate_of_counting>"


# callback for the feedback from server
def counter_feedback_callback(msg):
    rospy.loginfo(rospy.get_caller_id() + "the value of the counter is currently %d", msg.counter_value)


# setup node and subscribe to /feedback_topic
def setup_node():
    rospy.init_node("counter_client", anonymous=True)
    rospy.Subscriber("/feedback_topic", Counter, counter_feedback_callback)


# TODO: add parameters to this function to take start and rate
def counter_goal_client(count):
    rospy.wait_for_service("counter_goal_server")

    try:
        counterHandle = rospy.ServiceProxy("counter_goal_server", CounterGoal)
        # TODO: add arguments to counterHandle call to take start and rate
        goalResponse = counterHandle(count)
        if not goalResponse.result:
            print("Server failed to accept goal request!")
        return goalResponse.result
    except rospy.ServiceException as e:
        print("Service call for goal failed: %s"%e)
        return False


def counter_result_client():
    rospy.wait_for_service("counter_result_server")

    try:
        resultHandle = rospy.ServiceProxy("counter_result_server", CounterResult)
        resultResponse = resultHandle(True)
        print("Server has finished till %d..."%resultResponse.counted)
        return resultResponse.counted
    except rospy.ServiceException as e:
        print("Service call for result failed: %s"%e)
        return False


if __name__ == "__main__":
    # TODO: Change the number of arguments; read in the start and rate of counting from the command line
    if len(sys.argv) == 2:
        # `count` is the end value of the counter
        count = int(sys.argv[1])
    else:
        print(usage())
        sys.exit(1)
    
    setup_node()
    # TODO: Add format specifier for start value and change accordingly
    print("Requesting a count to %d starting from "%(count))

    # the server has accepted our request to start counting
    if counter_goal_client(count):
        # ask the server to start publishing on /feedback_topic
        counter_result_client()
        rospy.spin()
    else:
        sys.exit(1)
