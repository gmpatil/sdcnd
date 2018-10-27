# Programming a Real Self-Driving Car
Team: Girish Patil

Base code is cloned from [this](https://github.com/udacity/CarND-Capstone)  Udacity GitHub repository 

### Introduction
Goal of this project is to program self-driving car to drive around provided track in Unity simulator as well make Carla, Udacity's self-driving car 
drive on real test track. 
At high level tasks involve,
- Update Perception module to detect traffic lights, and stop the vehicle when red light is detected.
- Update Planning module to update target velocities for the waypoints.
- Update Control module's DBW (Drive By Wire) node to convert Twist commands to commands to control steering, throttle and break using various controllers.  

### Architecture
Below diagram depicts how RoS nodes interact with each other.
Code runs on [RoS](http://www.ros.org/) platform and nodes are written in Python and C++.

![RoS Graph](imgs/final-project-ros-graph-v2.png)

### Traffic Light Detection
Traffic light detection and classifier nodes use [Tensorflow's Object Detection API](https://github.com/tensorflow/models/tree/master/research/object_detection) 
to infer color of the traffic light.

Transfer Learning technique is used to train the Tensorflow Object Detection model to detect and classify traffic lights. 

Pre-trained [ssd_inception_v2_coco](https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/detection_model_zoo.md)
object detection model is used as a base model to train two separate models, one for Unity simulator environment and another for test track 
site/Carla to transfer learning/training to detect and classify traffic lights.

Due to version compatibility (Python 2.7.12, tensorflow 1.3.0) November 2017 version of [ssd_inception_v2_coco](http://download.tensorflow.org/models/object_detection/ssd_inception_v2_coco_2017_11_17.tar.gz)
model is used. 

JPEG (rgb8) images captured directly in Unity simulator environment and from Carla's camera image feed recorded 
during site track driving in RosBag are used to train the respective object detection models.    

 Thanks to [Alex Lechner](https://github.com/alex-lechner) and [Vatsal S](https://github.com/coldKnight/) for sharing 
 labeled training data set.
   

