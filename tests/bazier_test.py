#!/usr/bin/env python

import rospy
import rospkg
import sys
from nav_msgs.msg import Odometry

rospack = rospkg.RosPack()
ros_pkg_path = rospack.get_path('drive_stack')
module_path = ros_pkg_path + "/scripts"
if module_path not in sys.path:
    sys.path.insert(0, module_path)

from leader_bezier import *

bl = BezierLeader()

p0 = [0,0]
p1 = [1,0]
p2 = [1,0]
p3 = [1,1]

# bl.bezier(p0,p1,p2,p3,50,True)

start = Odometry()
end = Odometry()

start.pose.pose.position.x = 0.4387
start.pose.pose.position.y = 0.3816
start.pose.pose.orientation = heading_to_quaternion(1.6683)
start.twist.twist.linear.x = 1.5904

end.pose.pose.position.x = 0.1869
end.pose.pose.position.y = 0.4898
end.pose.pose.orientation = heading_to_quaternion(-0.3419)
end.twist.twist.linear.x = 0.0 #1.2926

bl.connect_waypoints(start,end,21,True)
