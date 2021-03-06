#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree
from std_msgs.msg import Int32

import numpy as np

import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 100 # 200 # Number of waypoints we will publish. You can change this number
MAX_ACCEL_DECEL = 1.0  # mts/sec
NUM_WPS_VEHICLE_CENTER = 3 #5
DECELERATE_FROM_DIST = 15  # mts

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb)
        # rospy.Subscriber('/obstacle_waypoint', Lane, self.waypoints_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # Member variables
        self.pose = None
        self.base_lane = None
        self.waypoints_2d = None
        self.waypoint_tree = None
        self.stopLine_wp_idx = None
        self.curr_velocity = None

        #rospy.spin()
        self.loop()

    def loop(self):
        rate = rospy.Rate(15) # 50
        while not rospy.is_shutdown():
            if self.pose and self.base_lane:
                # Get closest waypoint
                closest_waypoint_idx = self.get_closest_waypoint_idx()
                self.publish_waypoints(closest_waypoint_idx)
            rate.sleep()

    def get_closest_waypoint_idx(self):
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        closest_idx = self.waypoint_tree.query([x,y],1)[1]

        # Check if closest is ahead or behind vehicle
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx-1]

        # Eqn for hyperplane through closest_coords
        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x, y])

        val = np.dot(cl_vect - prev_vect, pos_vect - cl_vect)

        if val > 0:
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)
        return closest_idx

    def publish_waypoints(self, closest_idx):
        # lane = Lane()
        # lane.header = self.base_lane.header
        # lane.waypoints = self.base_lane.waypoints[closest_idx:closest_idx + LOOKAHEAD_WPS]
        # self.final_waypoints_pub.publish(lane)

        final_lane = self.generate_lane(closest_idx)
        self.final_waypoints_pub.publish(final_lane)

    def generate_lane(self, closest_idx):
        lane = Lane()
        lane.header = self.base_lane.header

        # closest_idx = self.get_closest_waypoint_idx()
        farthest_idx = closest_idx + LOOKAHEAD_WPS
        base_waypoints_sub = self.base_lane.waypoints[closest_idx:farthest_idx]

        if (not self.stopLine_wp_idx) or (self.stopLine_wp_idx == -1) or (self.stopLine_wp_idx >= farthest_idx):
            lane.waypoints = base_waypoints_sub
            # do not use new acceleration, honor target speed set for a WP in WPs load.
            # lane.waypoints = self.accelerate_waypoints(base_waypoints_sub)
        else:
            lane.waypoints = self.decelerate_waypoints(base_waypoints_sub, closest_idx)

        return lane

    def accelerate_waypoints(self, waypoints):
        temp = []

        if not self.curr_velocity:
            curr_v = 0.
        else:
            curr_v = self.curr_velocity

        for i, wp in enumerate(waypoints):
            p = Waypoint()
            p.pose = wp.pose

            curr_v = curr_v + MAX_ACCEL_DECEL

            # wp.twist.twist.linear.x has target/max  vel.
            if curr_v > wp.twist.twist.linear.x:
                curr_v = wp.twist.twist.linear.x

            p.twist.twist.linear.x = curr_v
            temp.append(p)

        return temp

    def decelerate_waypoints(self, waypoints, closest_idx):
        temp = []

        stop_idx = max(self.stopLine_wp_idx - closest_idx - NUM_WPS_VEHICLE_CENTER, 0)

        curr_v = self.curr_velocity

        for i, wp in enumerate(waypoints):
            p = Waypoint()
            p.pose = wp.pose

            dist = self.distance(waypoints, i, stop_idx)

            # if dist > DECELERATE_FROM_DIST:
            #     # wp.twist.twist.linear.x has target/max  vel.
            #     curr_v = wp.twist.twist.linear.x
            # elif dist > 0:
            #     if not self.curr_velocity:
            #         curr_v = math.sqrt(2 * MAX_ACCEL_DECEL * dist)
            #     else:
            #         curr_v = curr_v - (curr_v / dist)
            #
            #     if (curr_v < 1.) or (i >= stop_idx):
            #         curr_v = 0.
            # else:
            #     curr_v = 0.

            curr_v = math.sqrt(2 * MAX_ACCEL_DECEL * dist)

            if (curr_v < 1.) or (i >= stop_idx):
                curr_v = 0.

            p.twist.twist.linear.x = min(curr_v, wp.twist.twist.linear.x)
            temp.append(p)

        return temp

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, lane):
        waypoints = lane.waypoints
        self.base_lane = lane
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        # Callback for "/traffic_waypoint" message.
        self.stopLine_wp_idx = msg.data

    def velocity_cb(self, msg):
        self.curr_velocity = msg.twist.linear.x

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
