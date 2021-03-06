from styx_msgs.msg import TrafficLight
import tensorflow as tf
import numpy as np
import datetime
import rospy

class TLClassifier(object):

    def __init__(self, is_site, conf_threshold):
        if is_site:
            OBJ_DET_MODEL_PATH = r'light_classification/models/site_frozen_inference_graph.pb'
        else:
            OBJ_DET_MODEL_PATH = r'light_classification/models/sim_frozen_inference_graph.pb'

        self.graph = tf.Graph()
        self.threshold = conf_threshold # 0.4 # 0.5
        self.Cntr = 0

        with self.graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(OBJ_DET_MODEL_PATH, 'rb') as fid:
                od_graph_def.ParseFromString(fid.read())
                tf.import_graph_def(od_graph_def, name='')

            self.image_tensor = self.graph.get_tensor_by_name('image_tensor:0')
            self.boxes = self.graph.get_tensor_by_name('detection_boxes:0')
            self.scores = self.graph.get_tensor_by_name('detection_scores:0')
            self.classes = self.graph.get_tensor_by_name('detection_classes:0')
            self.num_detections = self.graph.get_tensor_by_name('num_detections:0')

        self.sess = tf.Session(graph=self.graph)


    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        with self.graph.as_default():
            img_expand = np.expand_dims(image, axis=0)
            start = datetime.datetime.now()
            (boxes, scores, classes, num_detections) = self.sess.run(
                [self.boxes, self.scores, self.classes, self.num_detections],
                feed_dict={self.image_tensor: img_expand})
            end = datetime.datetime.now()
            time_diff = end - start
            # rospy.loginfo("Inference took time: {0}".format(time_diff.total_seconds()))

        boxes = np.squeeze(boxes)
        scores = np.squeeze(scores)
        classes = np.squeeze(classes).astype(np.int32)

        rospy.loginfo("Obj Det top score: {0}. Class (1=G 2=R 3=Y 4=U): {1}".format(scores[0], classes[0]))
        #rospy.loginfo("Obj Det top score: {0}. Class (1=G 2=R 3=Y 4=U): {1} c: {2}".format(scores[0], classes[0], self.Cntr))
        #cv2.imwrite("tmpImgs/{0:05d}_{1}".format(self.Cntr, classes[0]) + ".jpg", image)
        #self.Cntr = self.Cntr + 1

        # Object Det: 1=G 2=R 3=Y 4=U
        # TrafficLght Const: 0=R 1=Y 2=G 4=U
        if scores[0] > self.threshold:
            if classes[0] == 1:
                #rospy.loginfo("Obj Det Ret Green(1=>2): {0}.".format(TrafficLight.GREEN))
                return TrafficLight.GREEN
            elif classes[0] == 2:
                #rospy.loginfo("Obj Det Ret Red(2=>0): {0}.".format(TrafficLight.RED))
                return TrafficLight.RED
            elif classes[0] == 3:
                #rospy.loginfo("Obj Det Ret Yellow(3=>1): {0}.".format(TrafficLight.YELLOW))
                return TrafficLight.YELLOW
        #else:
            #rospy.loginfo("Obj Det score below threshold of {0}, returning unknown 4.".format(self.threshold))

        return TrafficLight.UNKNOWN
