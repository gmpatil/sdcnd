import numpy as np
import cv2
import matplotlib.pyplot as plt
from Camera import Camera
from VideoProcessor import VideoProcessor


CALIBRATION_IMG_DIR = "./camera_cal/"
TEST_IMG_DIR = "./test_images/"





def main():
    cam = Camera(load=True)
    # ret, mtx, dist, rvecs, tvecs = cam.calibrate_camera(save=True)
    # cam.undistort_calibration_images()
    # cam.undistort_test_images()

    # # img = cv2.imread(TEST_IMG_DIR + "test4.jpg")
    # # img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    # # img = cam.undistort(img)
    # #
    # # video = VideoProcessor(cam)
    # bin = video.thresholded_binary(img)

    img = cv2.imread(TEST_IMG_DIR + "straight_lines1.jpg")
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img = cam.undistort(img)

    plt.imshow(img)
    plt.show()

if __name__ == '__main__':
    main()
