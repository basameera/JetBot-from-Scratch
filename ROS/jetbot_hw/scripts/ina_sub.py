#!/usr/bin/env python
# usage: rosrun jetbot_hw ina_sub.py
import rospy
from std_msgs.msg import String

ROS_TOPIC = 'jetbot_hw/ina'


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + '| %s', data.data)


def subscriber():
    rospy.init_node('ina_subscriber', anonymous=True)
    rospy.Subscriber(ROS_TOPIC, String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    subscriber()
