#!/usr/bin/env python
import numpy as np
import rospy
from turtle_patrol.srv import Patrol  # Import service type


def patrol_client():
    # Initialize the client node
    rospy.init_node('turtle1_patrol_client')
    # Wait until patrol service is ready
    rospy.wait_for_service('/turtle1/patrol')
    try:
        # Acquire service proxy
        patrol_proxy = rospy.ServiceProxy(
            '/turtle1/patrol', Patrol)
        vel = 2.0  # Linear velocity
        omega = 1.0  # Angular velocity
        rospy.loginfo('Command turtle1 to patrol')
        # Call patrol service via the proxy
        patrol_proxy(vel, omega)
    except rospy.ServiceException as e:
        rospy.loginfo(e)


if __name__ == '__main__':
    patrol_client()

