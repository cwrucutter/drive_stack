#!/usr/bin/env python

'''
Copyright 2015 William Baskin

/*****************************************
 LICENSE SUMMARY

 This package is licensed under the
    MIT License. Please see the LICENSE.md
    file in the root folder for the
    complete license.

 *****************************************/

 Path

 This is a class that generates high level goals in the map frame
 for a robot to achieve. It is designed to sync with a
 Leader node that takes the published goals, and then generates
 Odometry targets between them for the robot to follow (think carrot
 on a stick).

 Interface:
 srv/path/goal - returns current goal
 srv/path/next - moves path to next goal, returns that goal
 srv/path/start - returns starting goal
 srv/path/back - moves path to previous goal, returns that goal
 msg/path/current - publishes current goal
'''

import rospy
import math
from nav_msgs.msg import Odometry
from drive_stack.srv import Goal, GoalResponse

from utils import heading_to_quaternion
from utils import easy_Odom
from utils import drange as range

from math import pi as pi

class Path(object):
    """
    Path class: see module doc string

    no constructor arguments

    Example Usage:

    p = Path()
    p.run_server()
    """

    # pylint: disable=too-many-instance-attributes
    # These attributes represent the path, frame, and pub/sub

    def __init__(self, triple='simple'):
        # Initialize the ros node
        rospy.init_node('path', anonymous=True)
        self.MAX_SPEED = rospy.get_param('~max_speed', 1.0)
        print('MAX_SPEED = ' + str(self.MAX_SPEED))

        self.path = []
        self.load_path_from_yaml()

        self.index = 0

        self.rolling_index = -1

        # ROS publishers, etc. defined on start_server
        self.goal = None
        self.start = None
        self.start_pub = None
        self.back = None
        self.next = None
        self.current = None
        self.rolling = None

    # Load path from yaml

    def load_path_from_yaml(self):
        """
        Load the path from the parameter server
        """
        path = rospy.get_param('~waypoints')
        for p in path:
            self.path.append({"goal": easy_Odom(x=p["x"], y=p["y"], v=p["v"], heading=p["heading"], frame=p["frame"]), "direction": p["direction"]})


    # Pub/Sub/Service functionality

    def goal_callback(self, req=None):
        """
        return the current goal. callback for service
        """
        goal = self.path[self.index+1]["goal"]
        direction = self.path[self.index+1]["direction"]
        dist_tolerance = 1.0
        angle_tolerance = 1.0
        return GoalResponse(goal, direction, dist_tolerance, angle_tolerance)

    def next_callback(self, req=None):
        """
        return the current goal after stepping forward one. callback for service
        """
        if len(self.path) > self.index+2:
            self.index += 1
        return self.goal_callback()

    def start_callback(self, req=None):
        """
        return the current start point. callback for service
        """
        goal = self.path[self.index]["goal"]
        direction = self.path[self.index]["direction"]
        dist_tolerance = 1.0
        angle_tolerance = 1.0
        return GoalResponse(goal, direction, dist_tolerance, angle_tolerance)

    def back_callback(self, req=None):
        """
        return the start point after stepping back one. callback for service
        """
        self.index += -1
        if self.index < 0:
            self.index = 0
        return self.goal_callback()

    def next_rolling_pub(self):
        """
        return the next item in the path. rolling for RViz visuzalization
        """
        self.rolling_index += 1
        self.rolling_index = self.rolling_index % len(self.path)
        # rospy.loginfo('rlli i: '+str(self.path[self.rolling_index].pose.pose.position.x)+
        #     ','+str(self.path[self.rolling_index].pose.pose.position.y))
        return self.path[self.rolling_index]

    # Server/running management

    def wait_for_services(self):
        """
        wait for necessary services to implement interfaces
        """
        # OVERRIDE this method to have the node wait for a service or services
        #  before offering its own and beginning publishing. Be careful, because
        #  multiple nodes waiting on each other will maintain blocking calls
        #  indefinitely.
        pass

    def init_server(self):
        """
        start the ROS node, including offering services.
        """
        # pylint: disable=line-too-long
        # it makes sense for the service initiations to happen on one line
        self.goal = rospy.Service('/path/goal', Goal, self.goal_callback)
        self.next = rospy.Service('/path/next', Goal, self.next_callback)
        self.start = rospy.Service('/path/start', Goal, self.start_callback)
        self.back = rospy.Service('/path/back', Goal, self.back_callback)
        self.current = rospy.Publisher('/path/current', Odometry, queue_size=1)
        self.start_pub = rospy.Publisher('/path/start_goal', Odometry, queue_size=1)
        self.rolling = rospy.Publisher('/path/rolling', Odometry, queue_size=1)

        # for i in range(0, len(self.path)):
        #     rospy.loginfo('path: '+str(i)+' '+str(self.path[i].pose.pose.position.y))


    def publish_path_interface(self):
        """
        publish all path-interface related publishing requirements
        """
        self.current.publish(self.goal_callback("This").goal)
        self.start_pub.publish(self.start_callback().goal)
        # print 'rolling pub'
        self.rolling.publish(self.next_rolling_pub()["goal"])

    def run_server(self):
        """
        run the node. waits for services, inits server, runs publishers
        """
        self.wait_for_services()
        self.init_server()
        rospy.loginfo('path: server running')
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            self.publish_path_interface()
            rate.sleep()

if __name__ == '__main__':
    # pylint: disable=invalid-name
    # path = Path('figure_eight')
    path = Path('III-2017-01-29')
    path.run_server()
