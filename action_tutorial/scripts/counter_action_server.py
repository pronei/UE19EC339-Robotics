#!/usr/bin/env python3

from __future__ import print_function

import rospy

# import our interfaces from srv and msg folders
from action_tutorial.srv import CounterGoal, CounterGoalResponse
from action_tutorial.srv import CounterResult, CounterResultResponse
from action_tutorial.msg import Counter


counter_value = 0
counter_limit = 1
counterTimer = None
publishFeedback = False


def increment_counter_callback(event):
    global counter_value
    counter_value += 1


def handle_counter_goal(req):
    global counter_value, counter_limit, counterTimer
    rospy.loginfo("Accepted goal request to count till %d", req.count)
    # set up a timer to increment the value of counter
    # TODO: change `counter_value` to the start value read from the req
    counter_value = 0
    counter_limit = req.count
    # TODO: change the argument in `Duration` to that specified by the supplied rate in the req
    counterTimer = rospy.Timer(rospy.Duration(secs=1), increment_counter_callback)
    return CounterGoalResponse(True)


def handle_counter_result(req):
    rospy.loginfo("Accepted request to start publishing feedback")
    global publishFeedback, counter_value, counter_limit, counterTimer
    publishFeedback = True
    while counter_value <= counter_limit:
        pass
    return CounterResultResponse(counter_limit)


def action_server():
    rospy.init_node("counter_server")
    goalService = rospy.Service("counter_goal_server", CounterGoal, handle_counter_goal)
    resultService = rospy.Service("counter_result_server", CounterResult, handle_counter_result)
    feedbackPub = rospy.Publisher("/feedback_topic", Counter, queue_size=10)
    global counter_value, counter_limit, publishFeedback
    # run at 10Hz
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if counter_value > counter_limit:
            rospy.loginfo("Done counting to %d!", counter_limit)
            publishFeedback = False
            counterTimer.shutdown()
            counter_value = 0
            counter_limit = 1
        if publishFeedback:
            feedbackPub.publish(Counter(counter_value))
        rate.sleep()


if __name__ == "__main__":
    action_server()
