#!/usr/bin/env python3

from __future__ import print_function

import rospy
import sys
from action_tutorial.srv import CounterGoal, CounterResult
from action_tutorial.msg import Counter


def usage():
    return "%s <value_to_count_to>"%sys.argv[0]


def counter_feedback_callback(msg):
    rospy.loginfo(rospy.get_caller_id() + "the value of the counter is currently %d", msg.counter_value)


def setup_node():
    rospy.init_node("counter_client", anonymous=True)
    rospy.Subscriber("/feedback_topic", Counter, counter_feedback_callback)


def counter_goal_client(count):
    rospy.wait_for_service("counter_goal_server")

    try:
        counterHandle = rospy.ServiceProxy("counter_goal_server", CounterGoal)
        goalResponse = counterHandle(count)
        if not goalResponse.result:
            print("Server failed to accept goal request")
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
    if len(sys.argv) == 2:
        count = int(sys.argv[1])
    else:
        print(usage())
        sys.exit(1)
    
    setup_node()
    print("Requesting a count to %d"%count)

    # the server has accepted our request to start counting
    if counter_goal_client(count):
        # ask the server to start publishing on /feedback_topic
        counter_result_client()
        rospy.spin()
    else:
        sys.exit(1)
