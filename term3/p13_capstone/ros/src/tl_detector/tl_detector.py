#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import yaml
from scipy.spatial import KDTree
import numpy as np
import threading

STATE_COUNT_THRESHOLD = 3
STATE_CONF_THRESHOLD = 0.5 # 0.5

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None

        self.base_waypoints = None
        self.waypoints_2d = None
        self.waypoint_tree = None

        self.camera_image = None
        self.lights = []
        self.is_site = False
        self.has_image = False
        self.lock = threading.RLock()

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub4 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.is_site = self.config['is_site']
        rospy.loginfo("RoS environment is Site: {0}".format(self.is_site))

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier(self.is_site, STATE_CONF_THRESHOLD)
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        # rospy.spin()
        # Lets us handle image processing in this separate non-callback thread.
        self.loop()

    def loop(self):
        rate = rospy.Rate(10)  # 15

        while not rospy.is_shutdown():
            if self.has_image:
                light_wp, state = self.process_traffic_lights()

                #self.lock.acquire()
                #self.has_image = False
                #self.lock.release()

                '''
                Publish upcoming red lights at camera frequency.
                Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
                of times till we start using it. Otherwise the previous stable state is
                used.
                '''
                if self.state != state:
                    self.state_count = 0
                    self.state = state
                elif self.state_count >= STATE_COUNT_THRESHOLD:
                    self.last_state = self.state
                    light_wp = light_wp if (state == TrafficLight.RED) or (state == TrafficLight.YELLOW) else -1
                    self.last_wp = light_wp
                    #rospy.loginfo("Light(0=R 1=Y 2=G 4=U): {0} WP:{1} ".format(state, light_wp))
                    self.upcoming_red_light_pub.publish(Int32(light_wp))
                else:
                    self.upcoming_red_light_pub.publish(Int32(self.last_wp))

                self.state_count += 1

            rate.sleep()


    def pose_cb(self, msg):
        self.pose = msg


    def waypoints_cb(self, lane):
        self.base_waypoints = lane
        
        waypoints = lane.waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)


    def traffic_cb(self, msg):
        self.lights = msg.lights


    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        # rospy.loginfo("Got image.")
        self.camera_image = msg

        #if not self.has_image:
        #    self.lock.acquire()
        self.has_image = True
        #    self.lock.release()

        # Process image in non-callback thread, to avoid processing stale images.
        # light_wp, state = self.process_traffic_lights()

    def get_closest_waypoint(self, x, y):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        closest_idx = self.waypoint_tree.query([x, y], 1)[1]

        # Check if closest is ahead or behind vehicle
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx - 1]

        # Eqn for hyperplane through closest_coords
        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x, y])

        val = np.dot(cl_vect - prev_vect, pos_vect - cl_vect)

        if val > 0:
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)
        return closest_idx


    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if (not self.has_image):
            self.prev_light_loc = None
            return False

        # enc = None
        # if hasattr(self.camera_image, 'encoding'):
        #     img_encoding = self.camera_image.encoding
        #     if (img_encoding == "rgb8") or (img_encoding == "bgr8"):
        #         enc = img_encoding
        #
        # if enc is None:
        #     if self.is_site:
        #         enc = "bgr8"
        #     else:
        #         enc = "rgb8"

        enc = "rgb8" # as we have trained both site and simulator models using JPG images.
        #rospy.loginfo("enc " + enc)
        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, enc)  # rgb8 vs bgr8 (openCV bgr8)

        # Get classification
        light_class = self.light_classifier.get_classification(cv_image)

        #rospy.loginfo("Light state: {0}. Object Det:{1} (0=R 1=Y 2=G 4=U)".format(light.state, light_class))

        return light_class
        # return light.state


    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        closest_light = None
        line_wp_idx = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        # is_site = self.config['is_site']

        stop_line_positions = self.config['stop_line_positions']
        if (self.pose):
            car_position_idx = self.get_closest_waypoint(self.pose.pose.position.x, self.pose.pose.position.y)

            # find the closest visible traffic light (if one exists)
            diff = len(self.base_waypoints.waypoints)
            for i, light in enumerate(self.lights):
                # get stop line waypoint index.
                line = stop_line_positions[i]
                temp_wp_idx = self.get_closest_waypoint(line[0], line[1])
                # find closest
                d = temp_wp_idx - car_position_idx
                if d >= 0 and d < diff:
                    diff = d
                    closest_light = light
                    line_wp_idx = temp_wp_idx

        if closest_light:
            state = self.get_light_state(closest_light)
            # rospy.loginfo("c-pos:{0} sl-pos:{1}".format(car_position_idx, line_wp_idx))
            return line_wp_idx, state

        return -1, TrafficLight.UNKNOWN


if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
