#!/usr/bin/env python

import leader
import rospy
import math
import numpy as np
from geometry_msgs.msg import Point, Vector3
from nav_msgs.msg import Odometry

from utils import easy_Odom, heading_to_quaternion, quaternion_to_heading

class BezierLeader(leader.Leader):
    """
    Derived Leader class that generates targets using cubic Bezier curves.
    Overridden class attributes are generate_initial_path() and
    generate_next_path().
    """
    def generate_initial_path(self):
        """
        Path creation for node
        """
        rospy.loginfo('generating generate_initial_path')
        # Note: this is called once during node initialization
        end = self.path_goal().goal # Odometry
        start = self.path_start().goal # Odometry

        self.targets = self.connect_waypoints(start, end, 21)

        self.index = 0

    def generate_next_path(self):
        """
        generate a new path, either forwards or backwards (rvs == True)
        """
        end = self.path_next().goal
        start = self.path_start().goal

        self.targets = self.connect_waypoints(start, end, 21)

        if rvs:
            self.index = len(self.targets)-2
        else:
            self.index = 0

    def connect_waypoints(self, start, end, N, plot=False):
        pt0 = [start.pose.pose.position.x,
               start.pose.pose.position.y]
        pt3 = [end.pose.pose.position.x,
               end.pose.pose.position.y]
        pt1, pt2 = self.generate_control_points(start, end)
        return self.state_to_odom_list(self.bezier(pt0, pt1, pt2, pt3, N, plot))

    def generate_control_points(self, start, end):
        # Correction factor
        cf = 1.0
        x_pre = start.pose.pose.position.x
        y_pre = start.pose.pose.position.y
        theta_pre = quaternion_to_heading(start.pose.pose.orientation)
        v_pre = start.twist.twist.linear.x

        pt1 = [x_pre + v_pre*math.cos(theta_pre)/cf,
               y_pre + v_pre*math.sin(theta_pre)/cf]

        x_targ = end.pose.pose.position.x
        y_targ = end.pose.pose.position.y
        theta_targ = quaternion_to_heading(end.pose.pose.orientation)
        v_targ = end.twist.twist.linear.x

        pt2 = [x_targ - v_targ*math.cos(theta_targ)/cf,
               y_targ - v_targ*math.sin(theta_targ)/cf]

        return pt1, pt2

    def bezier(self, pt0, pt1, pt2, pt3, N, plot=False):
        # divide the time
        t = np.linspace(0.0, 1.0, num=N, endpoint=True)
        # Calculate position (s)
        s_x = ( np.power(1-t, 3) * pt0[0] +
                3 * np.multiply(np.power(1-t, 2), t) * pt1[0] +
                3 * np.multiply(1-t, np.power(t, 2)) * pt2[0] +
                np.multiply(np.power(t,3),pt3[0]) )
        s_y = ( np.power(1-t, 3) * pt0[1] +
                3 * np.multiply(np.power(1-t, 2), t) * pt1[1] +
                3 * np.multiply(1-t, np.power(t, 2)) * pt2[1] +
                np.power(t, 3) * pt3[1] )
        # Calculate velocity (v)
        v_x = ( 3 * np.power(1-t, 2) * (pt1[0] - pt0[0]) +
                6 * np.multiply(1-t, t) * (pt2[0] - pt1[0]) +
                3 * np.power(t, 2) * (pt3[0] - pt2[0]) )
        v_y = ( 3 * np.power(1-t, 2) * (pt1[1] - pt0[1]) +
                6 * np.multiply(1-t, t) * (pt2[1] - pt1[1]) +
                3 * np.power(t, 2) * (pt3[1] - pt2[1]) )
        # Calulate acceleration (a)
        a_x = ( 6 * (1-t) * (pt2[0] - 2 * pt1[0] + pt0[0]) +
                6 * t * (pt3[0] - 2 * pt2[0] + pt1[0]) )
        a_y = ( 6 * (1-t) * (pt2[1] - 2 * pt1[1] + pt0[1]) +
                6 * t * (pt3[1] - 2 * pt2[1] + pt1[1]) )

        v = np.sqrt(np.power(v_x,2) + np.power(v_y,2))
        theta = np.arctan2(v_y,v_x)
        # Curvature - https://stackoverflow.com/questions/46762955
        num = np.multiply(v_x, a_y) - np.multiply(v_y, a_x)
        denom = np.power(np.power(v_x,2) + np.power(v_y,2), 3/2.0)
        # Ensure 0/0 outputs 0 (not and error)
        kappa = np.divide(num, denom, out=np.zeros_like(num), where=denom!=0)
        omega = np.multiply(v,kappa)

        if plot:
            import matplotlib.pyplot as plt
            plt.figure(0)
            plt.subplot(311)
            plt.plot(t,s_x,'r.',t,s_y,'b.')
            plt.ylabel('s_x and s_y (m)')
            plt.xlabel('t (s)')
            plt.subplot(312)
            plt.plot(t,v_x,'r.',t,v_y,'b.')
            plt.ylabel('v_x and v_y (m/2)')
            plt.xlabel('t (s)')
            plt.subplot(313)
            plt.plot(t,a_x,'r.',t,a_y,'b.')
            plt.ylabel('a_x and a_y (m/s^2)')
            plt.xlabel('t (s)')

            plt.figure(1)
            plt.subplot(411)
            plt.plot(t,s_x,'r.',t,s_y,'b.')
            plt.ylabel('s_x and s_y (m)')
            plt.xlabel('t (s)')
            plt.subplot(412)
            plt.plot(t,v,'r.')
            plt.ylabel('v (m/s)')
            plt.xlabel('t (s)')
            plt.subplot(413)
            plt.plot(t,theta,'r.')
            plt.ylabel('theta (radians)')
            plt.xlabel('t (s)')
            plt.subplot(414)
            plt.plot(t,omega,'r.')
            plt.ylabel('omega (radians/s)')
            plt.xlabel('t (s)')

            plt.figure(2)
            plt.plot(s_x,s_y,'r.')
            plt.ylabel('y (m)')
            plt.xlabel('x (m)')

            plt.show()

        return s_x, s_y, theta, v, omega

    def state_to_odom_list(self, state):
        targets = []
        for x, y, theta, v, omega in zip(state[0], state[1], state[2],
                                         state[3], state[4]):
            odo = Odometry()
            odo.header.frame_id = 'map'
            odo.pose.pose.position = Point(x=x, y=y)
            odo.pose.pose.orientation = heading_to_quaternion(theta)
            odo.twist.twist.linear = Vector3(x=v)
            odo.twist.twist.angular = Vector3(z=omega)
            targets.append(odo)
        return targets



if __name__ == '__main__':
    # pylint: disable=invalid-name
    # leader is a fine name, it's not a constant
    leader = BezierLeader()
    leader.run_server()
