#!/usr/bin/env python
"""
Simple Driver
Just point towards the next waypoint and go.

process_position is the key method here
"""

import driver
import rospy

import math
import sys
# from driver import heading_to_quaternion, quaternion_to_heading
# from driver import dot_product, cross_product, scale, unit
# import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from utils import heading_to_quaternion, quaternion_to_heading, minimize_angle

class SimpleDriver(driver.Driver):
    """
    See module docstring
    """

    # pylint: disable=no-self-use
    # incorrectly identifies helpers as no-self-use

    def __init__(self):
        super(SimpleDriver, self).__init__()
        self.last_odom = None
        self.dtheta_int = 0.0 # for integral control
        self.state = 'startup'

    def get_params(self):
        self.max_accel = rospy.get_param('~max_accel', 1.0)
        self.max_v = rospy.get_param('~max_v', 0.5)
        self.max_alpha = rospy.get_param('~max_alpha', 4.0)
        self.max_omega = rospy.get_param('~max_omega', 4.0)

    def process_position(self, odom):
        """
        Uses a linear correction based on the error from the desired
        position
        """
        if self.state == 'startup':
            # this will exit the callback and not respond the the EKF
            #  information if it is still running the startup open loop
            return None
        # rospy.loginfo('process_position')
        # rospy.loginfo('odom'+str(odom))
        ## TODO check:
        odom.header.frame_id = 'map'
        self.last_pose_data.publish(odom)

        if self.last_odom is None:
            self.last_odom = odom

        next_goal = self.lead_goal().goal

        # rospy.loginfo('goal'+str(next_goal))

        while self.advance_next_goal(odom, next_goal):
            # if you are .04 m or less from the goal, move forward
            # NOTE: this might change.
            # for example, you may want to look at the next_goal two points
            #  ahead and see if you want to skip the current one, or if you are
            #  ahead of the current one in the direction that you want to go
            next_goal = self.lead_next().goal

        # errors along axis "x", off axis "y", heading "theta"
        along, off, heading = self.calc_errors(odom, next_goal)
        # rospy.loginfo('aoh'+ str( (along, off, heading)) )

        angular_vel = self.calc_angular_velocity(odom, next_goal)

        linear_vel = self.max_v

        # angular_vel = next_goal.twist.twist.angular.z
        # linear_vel = next_goal.twist.twist.linear.x

        # linear and angular velocity are now within dx/dt, d2x/dt2 limits
        twist_out = Twist()
        twist_out.linear.x = linear_vel
        twist_out.angular.z = angular_vel

        if math.isnan(linear_vel) or math.isnan(angular_vel):
            linear_vel = 0
            angular_vel = 0
            twist_out.linear.x = linear_vel
            twist_out.angular.z = angular_vel
            self.cmd_vel.publish(twist_out)
            rospy.loginfo('Error in driver calculation')
            sys.exit(0)

        self.cmd_vel.publish(twist_out)

    def calc_adjusted_heading(self, heading, off):
        # sign conventions:
        # axis: x axis is parallel to goal, y axis is to- the left when facing
        #  the goal direction, z-axis is oriented up
        # positive heading error - rotated counter clockwise from goal
        # positve offset error - positive y-axis

        # 4.8284 is an arbitrary constant that results in the correction being
        #  75% of 90 degrees when the offset is .5 meters
        heading_from_off = math.atan(4.8284*off)

        adjusted_heading = heading + heading_from_off

        while adjusted_heading > math.pi*2:
            adjusted_heading = adjusted_heading - math.pi*2
        while adjusted_heading < math.pi*-2:
            adjusted_heading = adjusted_heading + math.pi*2
        if adjusted_heading > math.pi:
            adjusted_heading = math.pi*-2 + adjusted_heading
        if adjusted_heading < -math.pi:
            adjusted_heading = math.pi*2 + adjusted_heading

        return adjusted_heading


    def calc_angular_velocity(self, odom, next_goal):
        dx = next_goal.pose.pose.position.x - odom.pose.pose.position.x
        dy = next_goal.pose.pose.position.y - odom.pose.pose.position.y
        theta_des = math.atan2(dy,dx)
        theta_act = quaternion_to_heading(odom.pose.pose.orientation)
        dtheta = minimize_angle(theta_des-theta_act)
        self.dtheta_int += dtheta
        k_p = 1.0
        k_i = 0.03
        omega = odom.twist.twist.angular.z + k_p * dtheta + k_i * self.dtheta_int
        # return self.check_angular_limits(odom, omega)
        return omega

    def calc_linear_velocity(self, along, off, angular_vel, goal_vel, odom):
        linear_vel = goal_vel

        scaling_factor = (self.max_omega - (abs(angular_vel) - .1))/(self.max_omega)
        scaling_factor = min(1, max(scaling_factor , 0.3))
        # scaling_factor = 1
        # rospy.loginfo('s: %f g: %f' % (scaling_factor, goal_vel,))
        linear_vel = goal_vel*scaling_factor

        florence = .15

        if linear_vel < florence and goal_vel > florence: 
            # if the scaling factor brought it down to 0ish
            # give it a min speed
            linear_vel = florence
        elif linear_vel < florence and goal_vel < florence:
            # if goal was already pretty low, don't mess with it
            linear_vel = goal_vel
        else:
            # linear vel is above a slow speed, so we don't need to worry
            pass

        return self.check_linear_limits(odom, linear_vel)

    def check_linear_limits(self, odom, linear_vel):
        """
        Make sure the given linear velocity fits within accel/speed limits
        """
        # limit maximum acceleration
        if odom.twist.twist.linear.x + self.max_accel < linear_vel:
            linear_vel = odom.twist.twist.linear.x + self.max_accel
        elif linear_vel < odom.twist.twist.linear.x - self.max_accel:
            linear_vel = odom.twist.twist.linear.x - self.max_accel

        # limit maximum speed
        if linear_vel > self.max_v:
            linear_vel = self.max_v
        elif linear_vel < -1.0*self.max_v:
            linear_vel = -1.0*self.max_v

        return linear_vel

    def check_angular_limits(self, odom, angular_vel):
        """
        Make sure the given angular velocity fits within accel/speed limits
        """
        # limit maximum angular acceleration
        if odom.twist.twist.angular.z + self.max_alpha < angular_vel:
            angular_vel = odom.twist.twist.angular.z + self.max_alpha
        elif angular_vel < odom.twist.twist.angular.z - self.max_alpha:
            angular_vel = odom.twist.twist.angular.z - self.max_alpha

        # limit maximum angular speed
        if angular_vel > self.max_omega:
            angular_vel = self.max_omega
        elif angular_vel < -1.0*self.max_omega:
            angular_vel = -1.0*self.max_omega

        return angular_vel

    def advance_next_goal(self, odom, current):
        along = self.calc_errors(odom, current)[0]
        return along >= 0.5

    def run_node(self):
        """
        Runs the ROS node (initialization, cyclical pub/sub)
        """
        self.wait_for_services(node_name='driver_simple')
        self.init_node()
        self.get_params()
        self.position = rospy.Subscriber('/odom_map', Odometry, self.process_position)
        self.state = 'running'
        rospy.spin()

if __name__ == '__main__':
    # pylint: disable=invalid-name
    # ignoring the trivial naming of the SimpleDriver class for startup
    # and run
    d = SimpleDriver()
    d.run_node()
