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
        self.MAX_SPEED = rospy.get_param('~max_speed', 1.0)
        print('MAX_SPEED = ' + str(self.MAX_SPEED))

        self.path = []
        if triple == 'simple':
            # start
            self.path.append(easy_Odom(x=0, y=0, v=0.0, heading=0.0, frame='map'))
            # out
            self.path.append(easy_Odom(x=0.5, y=0, v=0.0, heading=0.0, frame='map'))

        # Figure-8 for x:8m x y:6m box. (forms a 6m-8m-10m right triangle)
        elif triple == 'f8_8x6':
            # Start in middle facing top left beacon (8mx6m point)
            self.path.append(easy_Odom(x=4.0000, y=3.0000, v=0.5, heading= 0.6458, frame='map'))
            # Draw partial CW circle with 1.5m radius on right
            self.path.append(easy_Odom(x=5.6000, y=4.2000, v=0.5, heading= 0.6458, frame='map'))
            self.path.append(easy_Odom(x=6.3653, y=4.4939, v=0.5, heading= 0.0873, frame='map'))
            self.path.append(easy_Odom(x=7.1708, y=4.3416, v=0.5, heading=-0.4712, frame='map'))
            self.path.append(easy_Odom(x=7.7760, y=3.7886, v=0.5, heading=-1.0123, frame='map'))
            self.path.append(easy_Odom(x=8.0000, y=3.0000, v=0.5, heading=-1.5708, frame='map'))
            self.path.append(easy_Odom(x=7.7760, y=2.2114, v=0.5, heading=-2.1293, frame='map'))
            self.path.append(easy_Odom(x=7.1708, y=1.6584, v=0.5, heading=-2.6704, frame='map'))
            self.path.append(easy_Odom(x=6.3653, y=1.5061, v=0.5, heading= 3.0543, frame='map'))
            self.path.append(easy_Odom(x=5.6000, y=1.8000, v=0.5, heading= 2.4958, frame='map'))
            # Done with partial CW circle on right -> go back to middle
            self.path.append(easy_Odom(x=4.0000, y=3.0000, v=0.5, heading= 2.4958, frame='map'))
            # Draw partial CCW circle with 1.5m radius on left
            self.path.append(easy_Odom(x=2.4000, y=4.2000, v=0.5, heading= 2.4958, frame='map'))
            self.path.append(easy_Odom(x=1.6347, y=4.4939, v=0.5, heading= 3.0543, frame='map'))
            self.path.append(easy_Odom(x=0.8292, y=4.3416, v=0.5, heading=-2.6704, frame='map'))
            self.path.append(easy_Odom(x=0.2240, y=3.7886, v=0.5, heading=-2.1293, frame='map'))
            self.path.append(easy_Odom(x=0.0000, y=3.0000, v=0.5, heading=-1.5708, frame='map'))
            self.path.append(easy_Odom(x=0.2240, y=2.2114, v=0.5, heading=-1.0123, frame='map'))
            self.path.append(easy_Odom(x=0.8292, y=1.6584, v=0.5, heading=-0.4712, frame='map'))
            self.path.append(easy_Odom(x=1.6347, y=1.5061, v=0.5, heading= 0.0873, frame='map'))
            self.path.append(easy_Odom(x=2.4000, y=1.8000, v=0.5, heading= 0.6458, frame='map'))
            # Done with partial CCW circle on left -> go back to middle
            self.path.append(easy_Odom(x=4.0000, y=3.0000, v=0.5, heading= 0.6458, frame='map'))
            # Stop

        elif triple == 'I-2017-01-28':
            # start
            self.path.append(easy_Odom(x=1.75, y=2, v=0.5, heading=pi/2, frame='map'))
            # offset
            self.path.append(easy_Odom(x=1.75, y=3, v=0.5, heading=pi/2, frame='map'))
            # out to obstacle
            self.path.append(easy_Odom(x=1.75, y=10.2, v=0.5, heading=pi/2, frame='map'))
            # around obs 1
            self.path.append(easy_Odom(x=1.5, y=11.0, v=0.5, heading=5*pi/8, frame='map'))
            # around obs 2
            self.path.append(easy_Odom(x=1.5, y=12.0, v=0.5, heading=pi/4, frame='map'))
            # top round off
            self.path.append(easy_Odom(x=2.0, y=12.5, v=0.5, heading=0, frame='map'))
            # around obs 3
            self.path.append(easy_Odom(x=2.5, y=12.0, v=0.5, heading=-pi/4, frame='map'))
            # around obs 4
            self.path.append(easy_Odom(x=2.5, y=11.0, v=0.5, heading=-5*pi/8, frame='map'))
            # back from obstacle
            self.path.append(easy_Odom(x=2.25, y=10.2, v=0.5, heading=-pi/2, frame='map'))
            # back
            self.path.append(easy_Odom(x=2.25, y=2, v=0.5, heading=-pi/2, frame='map'))
            # turn 1
            self.path.append(easy_Odom(x=2.5, y=1.5, v=0.5, heading=-pi/2, frame='map'))
            # turn 2
            self.path.append(easy_Odom(x=2, y=1, v=0.5, heading=pi, frame='map'))
            # turn 3
            self.path.append(easy_Odom(x=1.5, y=1.5, v=0.5, heading=pi/2, frame='map'))
            # back to start
            # self.path.append(easy_Odom(x=1.75, y=2, v=0.5, heading=pi/2, frame='map'))

        elif triple == 'III-2017-01-29':
            # 01 - outside left - begin
            self.path.append(easy_Odom(x=2.375, y=1.5, v=self.MAX_SPEED, heading=pi/2, frame='map'))
            # 01 - outside left - end
            self.path.append(easy_Odom(x=2.375, y=12.0, v=self.MAX_SPEED, heading=pi/2, frame='map'))
            # 01 to 02 - turn
            self.path.append(easy_Odom(x=3.5, y=13.25, v=self.MAX_SPEED, heading=0.0, frame='map'))
            # 02 - outside right - begin
            self.path.append(easy_Odom(x=4.625, y=12.0, v=self.MAX_SPEED, heading=-pi/2, frame='map'))
            # 02 - outside right - end
            self.path.append(easy_Odom(x=4.625, y=3.0, v=self.MAX_SPEED, heading=-pi/2, frame='map'))
            # 02 to 03 - turn
            self.path.append(easy_Odom(x=3.875, y=2.25, v=self.MAX_SPEED, heading=-pi, frame='map'))
            # 03 - inside left - begin
            self.path.append(easy_Odom(x=3.125, y=3.0, v=self.MAX_SPEED, heading=pi/2, frame='map'))
            # 03 - inside left - end
            self.path.append(easy_Odom(x=3.125, y=11.5, v=self.MAX_SPEED, heading=pi/2, frame='map'))
            # 03 to 04 - turn 1
            self.path.append(easy_Odom(x=2.8125, y=12.0, v=self.MAX_SPEED, heading=3*pi/4, frame='map'))
            # 03 to 04 - turn 2
            self.path.append(easy_Odom(x=2.5, y=12.5, v=self.MAX_SPEED, heading=pi/2, frame='map'))
            # 03 to 04 - turn 3
            self.path.append(easy_Odom(x=3.0, y=13.0, v=self.MAX_SPEED, heading=pi/4, frame='map'))
            # 03 to 04 - turn 4
            self.path.append(easy_Odom(x=3.5, y=13.5, v=self.MAX_SPEED, heading=0.0, frame='map'))
            # 03 to 04 - turn 5
            self.path.append(easy_Odom(x=4.0, y=13.0, v=self.MAX_SPEED, heading=-pi/4, frame='map'))
            # 03 to 04 - turn 6
            self.path.append(easy_Odom(x=4.5, y=12.5, v=self.MAX_SPEED, heading=-pi/2, frame='map'))
            # 03 to 04 - turn 7
            self.path.append(easy_Odom(x=4.1875, y=12.0, v=self.MAX_SPEED, heading=-3*pi/4, frame='map'))
            # 04 - inside right - begin
            self.path.append(easy_Odom(x=3.875, y=11.5, v=self.MAX_SPEED, heading=-pi/2, frame='map'))
            # 04 - inside right - end
            self.path.append(easy_Odom(x=3.875, y=3.0, v=self.MAX_SPEED, heading=-pi/2, frame='map'))
            # 04 to 05 - turn
            self.path.append(easy_Odom(x=3.125, y=2.25, v=self.MAX_SPEED, heading=-pi, frame='map'))
            # 05 - outside left - begin
            self.path.append(easy_Odom(x=2.375, y=3.0, v=self.MAX_SPEED, heading=pi/2, frame='map'))
            # 05 - outside left - end
            self.path.append(easy_Odom(x=2.375, y=12.0, v=self.MAX_SPEED, heading=pi/2, frame='map'))
            # 05 to 06 - turn
            self.path.append(easy_Odom(x=3.5, y=13.25, v=self.MAX_SPEED, heading=0.0, frame='map'))
            # 06 - outside right - begin
            self.path.append(easy_Odom(x=4.625, y=12.0, v=self.MAX_SPEED, heading=-pi/2, frame='map'))
            # 06 - outside right - end
            self.path.append(easy_Odom(x=4.625, y=3.0, v=self.MAX_SPEED, heading=-pi/2, frame='map'))
            # 06 - finish in garage
            self.path.append(easy_Odom(x=3.0, y=2.0, v=self.MAX_SPEED, heading=-pi, frame='map'))

        elif triple == 'coop':
            # start
            self.path.append(easy_Odom(x=1.75, y=2, v=0.5, heading=pi/2, frame='map'))
            # offset
            self.path.append(easy_Odom(x=1.75, y=3, v=0.5, heading=pi/2, frame='map'))
            # out to obstacle
            self.path.append(easy_Odom(x=1.75, y=4.5, v=0.5, heading=pi/2, frame='map'))
            # around obs 1
            self.path.append(easy_Odom(x=1.5, y=5.5, v=0.5, heading=5*pi/8, frame='map'))
            # around obs 2
            self.path.append(easy_Odom(x=1.5, y=6.0, v=0.5, heading=pi/4, frame='map'))
            # top round off
            self.path.append(easy_Odom(x=2.0, y=6.5, v=0.5, heading=0, frame='map'))
            # around obs 3
            self.path.append(easy_Odom(x=2.5, y=6.0, v=0.5, heading=-pi/2, frame='map'))
            # around obs 4
            self.path.append(easy_Odom(x=2.5, y=5.5, v=0.5, heading=-5*pi/8, frame='map'))
            # back from obstacle
            self.path.append(easy_Odom(x=2.25, y=4.5, v=0.5, heading=-pi/2, frame='map'))
            # back
            self.path.append(easy_Odom(x=2.25, y=1.5, v=0.5, heading=-pi/2, frame='map'))

        # Figure-8 for ShowCASE Demo
        elif triple == 'figure_eight':
            # CW Circle
            self.path.append(easy_Odom(x=4.0, y=1.5, v=0.5, heading=-pi/2.0, frame='map'))
            self.path.append(easy_Odom(x=2.75, y=0.25, v=0.5, heading=pi, frame='map'))
            self.path.append(easy_Odom(x=1.5, y=1.5, v=0.5, heading=pi/2.0, frame='map'))
            self.path.append(easy_Odom(x=2.75, y=2.75, v=0.5, heading=0.0, frame='map'))
            # CCW Circle
            self.path.append(easy_Odom(x=4.0, y=1.5, v=0.5, heading=-pi/2.0, frame='map'))
            self.path.append(easy_Odom(x=5.25, y=0.25, v=0.5, heading=0.0, frame='map'))
            self.path.append(easy_Odom(x=6.5, y=1.5, v=0.5, heading=pi/2.0, frame='map'))
            self.path.append(easy_Odom(x=5.25, y=2.75, v=0.5, heading=pi, frame='map'))
            # CW Circle
            self.path.append(easy_Odom(x=4.0, y=1.5, v=0.5, heading=-pi/2.0, frame='map'))
            self.path.append(easy_Odom(x=2.75, y=0.25, v=0.5, heading=pi, frame='map'))
            self.path.append(easy_Odom(x=1.5, y=1.5, v=0.5, heading=pi/2.0, frame='map'))
            self.path.append(easy_Odom(x=2.75, y=2.75, v=0.5, heading=0.0, frame='map'))
            # CCW Circle
            self.path.append(easy_Odom(x=4.0, y=1.5, v=0.5, heading=-pi/2.0, frame='map'))
            self.path.append(easy_Odom(x=5.25, y=0.25, v=0.5, heading=0.0, frame='map'))
            self.path.append(easy_Odom(x=6.5, y=1.5, v=0.5, heading=pi/2.0, frame='map'))
            self.path.append(easy_Odom(x=5.25, y=2.75, v=0.5, heading=pi, frame='map'))
            # CW Circle
            self.path.append(easy_Odom(x=4.0, y=1.5, v=0.5, heading=-pi/2.0, frame='map'))
            self.path.append(easy_Odom(x=2.75, y=0.25, v=0.5, heading=pi, frame='map'))
            self.path.append(easy_Odom(x=1.5, y=1.5, v=0.5, heading=pi/2.0, frame='map'))
            self.path.append(easy_Odom(x=2.75, y=2.75, v=0.5, heading=0.0, frame='map'))
            # CCW Circle
            self.path.append(easy_Odom(x=4.0, y=1.5, v=0.5, heading=-pi/2.0, frame='map'))
            self.path.append(easy_Odom(x=5.25, y=0.25, v=0.5, heading=0.0, frame='map'))
            self.path.append(easy_Odom(x=6.5, y=1.5, v=0.5, heading=pi/2.0, frame='map'))
            self.path.append(easy_Odom(x=5.25, y=2.75, v=0.5, heading=pi, frame='map'))
            # CW Circle
            self.path.append(easy_Odom(x=4.0, y=1.5, v=0.5, heading=-pi/2.0, frame='map'))
            self.path.append(easy_Odom(x=2.75, y=0.25, v=0.5, heading=pi, frame='map'))
            self.path.append(easy_Odom(x=1.5, y=1.5, v=0.5, heading=pi/2.0, frame='map'))
            self.path.append(easy_Odom(x=2.75, y=2.75, v=0.5, heading=0.0, frame='map'))
            # CCW Circle
            self.path.append(easy_Odom(x=4.0, y=1.5, v=0.5, heading=-pi/2.0, frame='map'))
            self.path.append(easy_Odom(x=5.25, y=0.25, v=0.5, heading=0.0, frame='map'))
            self.path.append(easy_Odom(x=6.5, y=1.5, v=0.5, heading=pi/2.0, frame='map'))
            self.path.append(easy_Odom(x=5.25, y=2.75, v=0.5, heading=pi, frame='map'))
            # CW Circle
            self.path.append(easy_Odom(x=4.0, y=1.5, v=0.5, heading=-pi/2.0, frame='map'))
            self.path.append(easy_Odom(x=2.75, y=0.25, v=0.5, heading=pi, frame='map'))
            self.path.append(easy_Odom(x=1.5, y=1.5, v=0.5, heading=pi/2.0, frame='map'))
            self.path.append(easy_Odom(x=2.75, y=2.75, v=0.5, heading=0.0, frame='map'))
            # CCW Circle
            self.path.append(easy_Odom(x=4.0, y=1.5, v=0.5, heading=-pi/2.0, frame='map'))
            self.path.append(easy_Odom(x=5.25, y=0.25, v=0.5, heading=0.0, frame='map'))
            self.path.append(easy_Odom(x=6.5, y=1.5, v=0.5, heading=pi/2.0, frame='map'))
            self.path.append(easy_Odom(x=5.25, y=2.75, v=0.5, heading=pi, frame='map'))
            # CW Circle
            self.path.append(easy_Odom(x=4.0, y=1.5, v=0.5, heading=-pi/2.0, frame='map'))
            self.path.append(easy_Odom(x=2.75, y=0.25, v=0.5, heading=pi, frame='map'))
            self.path.append(easy_Odom(x=1.5, y=1.5, v=0.5, heading=pi/2.0, frame='map'))
            self.path.append(easy_Odom(x=2.75, y=2.75, v=0.5, heading=0.0, frame='map'))
            # CCW Circle
            self.path.append(easy_Odom(x=4.0, y=1.5, v=0.5, heading=-pi/2.0, frame='map'))
            self.path.append(easy_Odom(x=5.25, y=0.25, v=0.5, heading=0.0, frame='map'))
            self.path.append(easy_Odom(x=6.5, y=1.5, v=0.5, heading=pi/2.0, frame='map'))
            self.path.append(easy_Odom(x=5.25, y=2.75, v=0.5, heading=pi, frame='map'))
            # CW Circle
            self.path.append(easy_Odom(x=4.0, y=1.5, v=0.5, heading=-pi/2.0, frame='map'))
            self.path.append(easy_Odom(x=2.75, y=0.25, v=0.5, heading=pi, frame='map'))
            self.path.append(easy_Odom(x=1.5, y=1.5, v=0.5, heading=pi/2.0, frame='map'))
            self.path.append(easy_Odom(x=2.75, y=2.75, v=0.5, heading=0.0, frame='map'))
            # CCW Circle
            self.path.append(easy_Odom(x=4.0, y=1.5, v=0.5, heading=-pi/2.0, frame='map'))
            self.path.append(easy_Odom(x=5.25, y=0.25, v=0.5, heading=0.0, frame='map'))
            self.path.append(easy_Odom(x=6.5, y=1.5, v=0.5, heading=pi/2.0, frame='map'))
            self.path.append(easy_Odom(x=5.25, y=2.75, v=0.5, heading=pi, frame='map'))
            # CW Circle
            self.path.append(easy_Odom(x=4.0, y=1.5, v=0.5, heading=-pi/2.0, frame='map'))
            self.path.append(easy_Odom(x=2.75, y=0.25, v=0.5, heading=pi, frame='map'))
            self.path.append(easy_Odom(x=1.5, y=1.5, v=0.5, heading=pi/2.0, frame='map'))
            self.path.append(easy_Odom(x=2.75, y=2.75, v=0.5, heading=0.0, frame='map'))
            # CCW Circle
            self.path.append(easy_Odom(x=4.0, y=1.5, v=0.5, heading=-pi/2.0, frame='map'))
            self.path.append(easy_Odom(x=5.25, y=0.25, v=0.5, heading=0.0, frame='map'))
            self.path.append(easy_Odom(x=6.5, y=1.5, v=0.5, heading=pi/2.0, frame='map'))
            self.path.append(easy_Odom(x=5.25, y=2.75, v=0.5, heading=pi, frame='map'))
            # CW Circle
            self.path.append(easy_Odom(x=4.0, y=1.5, v=0.5, heading=-pi/2.0, frame='map'))
            self.path.append(easy_Odom(x=2.75, y=0.25, v=0.5, heading=pi, frame='map'))
            self.path.append(easy_Odom(x=1.5, y=1.5, v=0.5, heading=pi/2.0, frame='map'))
            self.path.append(easy_Odom(x=2.75, y=2.75, v=0.5, heading=0.0, frame='map'))
            # CCW Circle
            self.path.append(easy_Odom(x=4.0, y=1.5, v=0.5, heading=-pi/2.0, frame='map'))
            self.path.append(easy_Odom(x=5.25, y=0.25, v=0.5, heading=0.0, frame='map'))
            self.path.append(easy_Odom(x=6.5, y=1.5, v=0.5, heading=pi/2.0, frame='map'))
            self.path.append(easy_Odom(x=5.25, y=2.75, v=0.5, heading=pi, frame='map'))
            # CW Circle
            self.path.append(easy_Odom(x=4.0, y=1.5, v=0.5, heading=-pi/2.0, frame='map'))
            self.path.append(easy_Odom(x=2.75, y=0.25, v=0.5, heading=pi, frame='map'))
            self.path.append(easy_Odom(x=1.5, y=1.5, v=0.5, heading=pi/2.0, frame='map'))
            self.path.append(easy_Odom(x=2.75, y=2.75, v=0.5, heading=0.0, frame='map'))
            # CCW Circle
            self.path.append(easy_Odom(x=4.0, y=1.5, v=0.5, heading=-pi/2.0, frame='map'))
            self.path.append(easy_Odom(x=5.25, y=0.25, v=0.5, heading=0.0, frame='map'))
            self.path.append(easy_Odom(x=6.5, y=1.5, v=0.5, heading=pi/2.0, frame='map'))
            self.path.append(easy_Odom(x=5.25, y=2.75, v=0.5, heading=pi, frame='map'))
            # CW Circle
            self.path.append(easy_Odom(x=4.0, y=1.5, v=0.5, heading=-pi/2.0, frame='map'))
            self.path.append(easy_Odom(x=2.75, y=0.25, v=0.5, heading=pi, frame='map'))
            self.path.append(easy_Odom(x=1.5, y=1.5, v=0.5, heading=pi/2.0, frame='map'))
            self.path.append(easy_Odom(x=2.75, y=2.75, v=0.5, heading=0.0, frame='map'))
            # CCW Circle
            self.path.append(easy_Odom(x=4.0, y=1.5, v=0.5, heading=-pi/2.0, frame='map'))
            self.path.append(easy_Odom(x=5.25, y=0.25, v=0.5, heading=0.0, frame='map'))
            self.path.append(easy_Odom(x=6.5, y=1.5, v=0.5, heading=pi/2.0, frame='map'))
            self.path.append(easy_Odom(x=5.25, y=2.75, v=0.5, heading=pi, frame='map'))
            #back to start
            self.path.append(easy_Odom(x=4.0, y=1.5, v=0.5, heading=-pi/2.0, frame='map'))

        elif triple == 'roltatedI':
            # start
            self.path.append(easy_Odom(x=3.0, y=2.0, v=0.65, heading=pi*3/4.0, frame='map'))
            self.path.append(easy_Odom(x=0.5, y=4.5, v=0.35, heading=pi*3/4.0, frame='map'))
            self.path.append(easy_Odom(x=0.5, y=5.0, v=0.35, heading=pi*1/4.0, frame='map'))
            self.path.append(easy_Odom(x=1.0, y=5.0, v=0.55, heading=pi*-1/4.0, frame='map'))
            self.path.append(easy_Odom(x=2.5, y=3.5, v=0.55, heading=pi*-1/4.0, frame='map'))
            self.path.append(easy_Odom(x=2.75, y=2.75, v=0.35, heading=pi*-1/4.0, frame='map'))
            self.path.append(easy_Odom(x=3.25, y=2.7, v=0.35, heading=pi*1/4.0, frame='map'))
            self.path.append(easy_Odom(x=3.5, y=3.5, v=0.55, heading=pi*3/4.0, frame='map'))
            self.path.append(easy_Odom(x=0.5, y=6.5, v=0.55, heading=pi*3/4.0, frame='map'))
        elif triple == 'halfI':
            # start
            self.path.append(easy_Odom(x=2, y=2, v=0.5, heading=pi/2, frame='map'))
            self.path.append(easy_Odom(x=1.75, y=3, v=0.5, heading=pi/2, frame='map'))
            # out 1
            self.path.append(easy_Odom(x=1.75, y=7, v=0.5, heading=pi/2, frame='map'))
            # out 2
            self.path.append(easy_Odom(x=1.5, y=7.5, v=0.5, heading=pi/2, frame='map'))
            # over
            self.path.append(easy_Odom(x=2, y=8, v=0.5, heading=0.0, frame='map'))
            # turned around
            self.path.append(easy_Odom(x=2.5, y=7.5, v=0.5, heading=-pi/2, frame='map'))
            # back 1
            self.path.append(easy_Odom(x=2.25, y=7, v=0.5, heading=-pi/2, frame='map'))
            # back 2
            self.path.append(easy_Odom(x=2.25, y=2, v=0.5, heading=-pi/2, frame='map'))
        elif triple == 'I':
            # start
            self.path.append(easy_Odom(x=1.75, y=2, v=0.5, heading=pi/2, frame='map'))
            # offset
            self.path.append(easy_Odom(x=1.75, y=3, v=0.5, heading=pi/2, frame='map'))
            # out 1
            self.path.append(easy_Odom(x=1.75, y=13, v=0.5, heading=pi/2, frame='map'))
            # out 2
            self.path.append(easy_Odom(x=1.5, y=13.5, v=0.5, heading=pi/2, frame='map'))
            # over
            self.path.append(easy_Odom(x=2, y=14, v=0.5, heading=0.0, frame='map'))
            # turned around
            self.path.append(easy_Odom(x=2.5, y=13.5, v=0.5, heading=-pi/2, frame='map'))
            # back 1
            self.path.append(easy_Odom(x=2.25, y=13, v=0.5, heading=-pi/2, frame='map'))
            # back 2
            self.path.append(easy_Odom(x=2.25, y=2, v=0.5, heading=-pi/2, frame='map'))
            # turn 1
            self.path.append(easy_Odom(x=2.5, y=1.5, v=0.5, heading=-pi/2, frame='map'))
            # turn 2
            self.path.append(easy_Odom(x=2, y=1, v=0.5, heading=pi, frame='map'))
            # turn 3
            self.path.append(easy_Odom(x=1.5, y=1.5, v=0.5, heading=pi/2, frame='map'))
            # back to start
            self.path.append(easy_Odom(x=1.75, y=2, v=0.5, heading=pi/2, frame='map'))
        elif triple == 'III' or triple == 'hamburger' or triple == 'zigzag':
            # start
            self.path.append(easy_Odom(x=3.25, y=1, v=0.5, heading=pi/2, frame='map'))
            # start 2
            self.path.append(easy_Odom(x=3.25, y=2, v=0.5, heading=pi/2, frame='map'))
            
            # back and forth
            path_width = float(.7)

            # top left
            self.path.append(easy_Odom(x=3.25, y=13-3*path_width/2, v=0.5, heading=pi/2, frame='map'))

            # bubble
            self.path.append(easy_Odom(x=3, y = 12.5, v=0.5, heading=pi/2, frame='map'))
            self.path.append(easy_Odom(x=3.5, y= 13, v=0.5, heading=0.0, frame='map'))
            self.path.append(easy_Odom(x=4, y = 12.5, v=0.5, heading=-pi/2, frame='map'))

            # top right
            self.path.append(easy_Odom(x=3.75, y=13-3*path_width/2, v=0.5, heading=-pi/2, frame='map'))

            # inside out spiral
            # center back
            self.path.append(easy_Odom(x=3.75, y=3+path_width, v=0.5, heading=-pi/2, frame='map'))
            self.path.append(easy_Odom(x=3.75, y=3+path_width/2, v=0.5, heading=pi, frame='map'))
            
            # left 1
            self.path.append(easy_Odom(x=3.25-path_width, y=3+path_width, v=0.5, heading=pi/2, frame='map'))
            self.path.append(easy_Odom(x=3.25-path_width, y=13-3*path_width/2, v=0.5, heading=pi/2, frame='map'))
            # left 1 top
            self.path.append(easy_Odom(x=3.5, y=13-path_width/2, v=0.5, heading=0.0, frame='map'))

            # right 1
            self.path.append(easy_Odom(x=3.75+path_width, y=13-3*path_width/2, v=0.5, heading=-pi/2, frame='map'))
            self.path.append(easy_Odom(x=3.75+path_width, y=3+3*path_width/2, v=0.5, heading=-pi/2, frame='map'))
            # right 1 bottom
            self.path.append(easy_Odom(x=3.5, y=3+path_width/2, v=0.5, heading=pi, frame='map'))

            # left 2
            self.path.append(easy_Odom(x=3.25-path_width, y=3+path_width/2, v=0.5, heading=pi, frame='map'))
            self.path.append(easy_Odom(x=3.25-2*path_width, y=3+3*path_width/2, v=0.5, heading=pi/2, frame='map'))
            self.path.append(easy_Odom(x=3.25-2*path_width, y=13-3*path_width/2, v=0.5, heading=pi/2, frame='map'))
            # left 2 top
            self.path.append(easy_Odom(x=3.25-path_width, y=13-path_width/2, v=0.5, heading=0.0, frame='map'))
            self.path.append(easy_Odom(x=3.5, y=13-path_width/2, v=0.5, heading=0.0, frame='map'))

            # right 2
            self.path.append(easy_Odom(x=3.75+2*path_width, y=13-5*path_width/2, v=0.5, heading=-pi/2, frame='map'))
            self.path.append(easy_Odom(x=3.75+2*path_width, y=2.0, v=0.5, heading=-pi/2, frame='map'))
            
            
            # garage
            self.path.append(easy_Odom(x=4.5, y=1, v=0.5, heading=pi, frame='map'))
            self.path.append(easy_Odom(x=3.0, y=1, v=0.5, heading=pi, frame='map'))

        elif triple == 'IxI':
            # start
            self.path.append(easy_Odom(x=1, y=1, v=0.5, heading=pi/2, frame='map'))
            # start 2
            self.path.append(easy_Odom(x=1, y=2, v=0.5, heading=pi/2, frame='map'))
            # back and forth
            path_width = float(.75)

            # lateral passes
            for i in range(3.0, 11.5, 2*path_width):
                self.path.append(easy_Odom(x=2.65, y=i+path_width/2, v=0.5, heading=0.0, frame='map'))
                self.path.append(easy_Odom(x=5, y=i+path_width/2, v=0.5, heading=0.0, frame='map'))
                self.path.append(easy_Odom(x=5+path_width/2, y=i+path_width, v=0.5, heading=pi/2, frame='map'))
                self.path.append(easy_Odom(x=5, y=i+path_width*3/2, v=0.5, heading=pi, frame='map'))
                self.path.append(easy_Odom(x=2.65, y=i+path_width*3/2, v=0.5, heading=pi, frame='map'))
                self.path.append(easy_Odom(x=2.65-path_width/2, y=i+2*path_width, v=0.5, heading=pi/2, frame='map'))

            # last across
            ## self.path.append(easy_Odom(x=2, y=12.5, v=0.5, heading=0.0, frame='map'))
            self.path.append(easy_Odom(x=3.5-path_width, y=13-path_width/2, v=0.5, heading=0.0, frame='map'))
            self.path.append(easy_Odom(x=3.5, y=13-3*path_width/2, v=0.5, heading=-pi/2, frame='map'))

            # inside out spiral
            # center
            self.path.append(easy_Odom(x=3.5, y=3+path_width, v=0.5, heading=-pi/2, frame='map'))
            self.path.append(easy_Odom(x=3.5-path_width/2, y=3+path_width/2, v=0.5, heading=pi, frame='map'))
            
            # left 1
            self.path.append(easy_Odom(x=3.5-path_width, y=3+path_width, v=0.5, heading=pi/2, frame='map'))
            self.path.append(easy_Odom(x=3.5-path_width, y=13-3*path_width/2, v=0.5, heading=pi/2, frame='map'))
            # left 1 top
            self.path.append(easy_Odom(x=3.5, y=13-path_width/2, v=0.5, heading=0.0, frame='map'))

            # right 1
            self.path.append(easy_Odom(x=3.5+path_width, y=13-3*path_width/2, v=0.5, heading=-pi/2, frame='map'))
            self.path.append(easy_Odom(x=3.5+path_width, y=3+3*path_width/2, v=0.5, heading=-pi/2, frame='map'))
            # right 1 bottom
            self.path.append(easy_Odom(x=3.5, y=3+path_width/2, v=0.5, heading=pi, frame='map'))

            # left 2
            self.path.append(easy_Odom(x=3.5-path_width, y=3+path_width/2, v=0.5, heading=pi, frame='map'))
            self.path.append(easy_Odom(x=2.25, y=3+3*path_width/2, v=0.5, heading=pi/2, frame='map'))
            self.path.append(easy_Odom(x=2.25, y=13-3*path_width/2, v=0.5, heading=pi/2, frame='map'))
            # left 2 top
            self.path.append(easy_Odom(x=3.5-path_width, y=13-path_width/2, v=0.5, heading=0.0, frame='map'))
            self.path.append(easy_Odom(x=3.5, y=13-path_width/2, v=0.5, heading=0.0, frame='map'))

            # right 2
            self.path.append(easy_Odom(x=3.5+2*path_width, y=13-5*path_width/2, v=0.5, heading=-pi/2, frame='map'))
            self.path.append(easy_Odom(x=3.5+2*path_width, y=3.0, v=0.5, heading=-pi/2, frame='map'))
            
            
            # garage
            self.path.append(easy_Odom(x=4.0, y=2, v=0.5, heading=pi, frame='map'))
            self.path.append(easy_Odom(x=3.0, y=2, v=0.5, heading=pi, frame='map'))

        elif triple == 'snownado':
            # start
            self.path.append(easy_Odom(x=2, y=1, v=0.5, heading=pi/2, frame='map'))
            # start 2
            self.path.append(easy_Odom(x=2, y=2, v=0.5, heading=pi/2, frame='map'))
            # back and forth
            path_width = float(.75)

            # lateral passes
            for i in range(3.0, 12.1, path_width):
                self.path.append(easy_Odom(x=2.25, y=i-path_width/2, v=0.5, heading=pi/2, frame='map')) # 1
                self.path.append(easy_Odom(x=2.25+path_width, y=i+path_width/2, v=0.5, heading=0, frame='map')) # 2
                self.path.append(easy_Odom(x=4.75-path_width, y=i+path_width/2, v=0.5, heading=0, frame='map')) # 3
                self.path.append(easy_Odom(x=4.75, y=i-path_width/2, v=0.5, heading=-pi/2, frame='map')) # 4
                self.path.append(easy_Odom(x=4.75-path_width, y=i-3*path_width/2, v=0.5, heading=pi, frame='map')) # 5
                self.path.append(easy_Odom(x=2.25+2*path_width, y=i-3*path_width/2, v=0.5, heading=pi, frame='map')) # 6

            # last across
            self.path.append(easy_Odom(x=2, y=12.5, v=0.5, heading=0.0, frame='map'))
            self.path.append(easy_Odom(x=3.5-path_width, y=13-path_width/2, v=0.5, heading=0.0, frame='map'))
            self.path.append(easy_Odom(x=3.5, y=13-3*path_width/2, v=0.5, heading=-pi/2, frame='map'))
            # finish without spiral
            self.path.append(easy_Odom(x=3.0, y=2.5, v=0.5, heading=-pi/2, frame='map'))

            # # inside out spiral
            # # center
            # self.path.append(easy_Odom(x=3.5, y=3+path_width, v=0.5, heading=-pi/2, frame='map'))
            # self.path.append(easy_Odom(x=3.5-path_width/2, y=3+path_width/2, v=0.5, heading=pi, frame='map'))
            
            # # left 1
            # self.path.append(easy_Odom(x=3.5-path_width, y=3+path_width, v=0.5, heading=pi/2, frame='map'))
            # self.path.append(easy_Odom(x=3.5-path_width, y=13-3*path_width/2, v=0.5, heading=pi/2, frame='map'))
            # # left 1 top
            # self.path.append(easy_Odom(x=3.5, y=13-path_width/2, v=0.5, heading=0.0, frame='map'))

            # # right 1
            # self.path.append(easy_Odom(x=3.5+path_width, y=13-3*path_width/2, v=0.5, heading=-pi/2, frame='map'))
            # self.path.append(easy_Odom(x=3.5+path_width, y=3+3*path_width/2, v=0.5, heading=-pi/2, frame='map'))
            # # right 1 bottom
            # self.path.append(easy_Odom(x=3.5, y=3+path_width/2, v=0.5, heading=pi, frame='map'))

            # # left 2
            # self.path.append(easy_Odom(x=3.5-path_width, y=3+path_width/2, v=0.5, heading=pi, frame='map'))
            # self.path.append(easy_Odom(x=3.5-2*path_width, y=3+3*path_width/2, v=0.5, heading=pi/2, frame='map'))
            # self.path.append(easy_Odom(x=3.5-2*path_width, y=13-3*path_width/2, v=0.5, heading=pi/2, frame='map'))
            # # left 2 top
            # self.path.append(easy_Odom(x=3.5-path_width, y=13-path_width/2, v=0.5, heading=0.0, frame='map'))
            # self.path.append(easy_Odom(x=3.5, y=13-path_width/2, v=0.5, heading=0.0, frame='map'))

            # # right 2
            # self.path.append(easy_Odom(x=3.5+2*path_width, y=13-5*path_width/2, v=0.5, heading=-pi/2, frame='map'))
            # self.path.append(easy_Odom(x=3.5+2*path_width, y=3.0, v=0.5, heading=-pi/2, frame='map'))
            
            
            # # garage
            # self.path.append(easy_Odom(x=4.0, y=2, v=0.5, heading=pi, frame='map'))
            # self.path.append(easy_Odom(x=3.0, y=2, v=0.5, heading=pi, frame='map'))

        elif triple == 'demo':
            self.path.append(easy_Odom(x=3.5, y=1, v=0.5, heading=pi/2, frame='map'))
            self.path.append(easy_Odom(x=3.5, y=12.5, v=0.5, heading=pi/2, frame='map'))
            self.path.append(easy_Odom(x=4.0, y=13, v=0.5, heading=0.0, frame='map'))
            self.path.append(easy_Odom(x=4.5, y=12.5, v=0.5, heading=-pi/2, frame='map'))
            self.path.append(easy_Odom(x=4.5, y = 2, v=0.5, heading=-pi/2, frame='map'))

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


    # Pub/Sub/Service functionality

    def goal_callback(self, req=None):
        """
        return the current goal. callback for service
        """
        return GoalResponse(self.path[self.index+1])

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
        return GoalResponse(self.path[self.index])

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
        rospy.init_node('default_path')
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
        self.rolling.publish(self.next_rolling_pub())

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
