import numpy as np
import cv2
import matplotlib.pyplot as plt
from Camera import Camera

CALIBRATION_IMG_DIR = "./camera_cal/"
TEST_IMG_DIR = "./test_images/"


def thresholded_binary(img, s_thresh=(170, 255), sx_thresh=(20, 100)):
    h_thresh = (15, 100)

    img = np.copy(img)
    # Convert to HSV color space and separate the V channel
    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HLS).astype(np.float)
    h_channel = hsv[:, :, 0]
    s_channel = hsv[:, :, 2]
    # Sobel x
    sobelx = cv2.Sobel(s_channel, cv2.CV_64F, 1, 0)  # Take the derivative in x
    abs_sobelx = np.absolute(sobelx)  # Absolute x derivative to accentuate lines away from horizontal
    scaled_sobel = np.uint8(255 * abs_sobelx / np.max(abs_sobelx))

    # Threshold x gradient
    sxbinary = np.zeros_like(scaled_sobel)
    sxbinary[(scaled_sobel >= sx_thresh[0]) & (scaled_sobel <= sx_thresh[1])] = 1

    # Threshold color channel
    s_binary = np.zeros_like(s_channel)
    s_binary[(s_channel >= s_thresh[0]) & (s_channel <= s_thresh[1])] = 1

    # Threshold color channel
    h_binary = np.zeros_like(h_channel)

    h_binary[(h_channel >= h_thresh[0]) & (h_channel  <= h_thresh[1])] = 1

    # Stack each channel
    # Note color_binary[:, :, 0] is all 0s, effectively an all black image. It might
    # be beneficial to replace this channel with something else.
    color_binary = np.dstack((np.zeros_like(sxbinary), sxbinary, s_binary))
    # color_binary = np.dstack((h_binary, sxbinary, s_binary))

    return color_binary


def main():
    cam = Camera(load=True)
    # ret, mtx, dist, rvecs, tvecs = cam.calibrate_camera(save=True)
    # cam.undistort_calibration_images()
    # cam.undistort_test_images()

    img = cv2.imread(TEST_IMG_DIR + "test1.jpg")
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img = cam.undistort(img)
    bin = thresholded_binary(img)
    plt.imshow(bin)
    plt.show()

if __name__ == '__main__':
    main()
