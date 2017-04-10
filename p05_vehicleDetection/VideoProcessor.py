import numpy as np
import cv2

import matplotlib.pyplot as plt
import io

TEST_IMG_DIR = "./test_images/"


class VideoProcessor(object):
    '''
    Class to process the video images one by one.
    '''

    def __init__(self):

        self.first_time = True # so that we can initialize


    def pipeline(self, img):
        (imgSeries, Minv) = self.get_binary_warped_image(img)   # ([undist, bin, binWarped], Minv)
        outImg = self.poly_fit_line(imgSeries[-1])  #blank binWarped
        imgSeries[-1] = outImg  # binWarped with windows, lane lines
        left_curverad = self.line_left.measure_radius()
        right_curverad = self.line_right.measure_radius()
        camera_offset = self.get_camera_offset()

        img = self.draw_on_orig_new(imgSeries[0], imgSeries[2], Minv)  # undist image with polyfill overlay.

        warped_bin_debug = cv2.resize(imgSeries[-1], (0, 0), fx=0.3, fy=0.3)  # binWarped with windows, lane lines
        img[:250, :, :] = img[:250, :, :] * .4
        (h, w, _) = warped_bin_debug.shape

        img[20:20 + h, 20:20 + w, :] = warped_bin_debug

        txt_x_loc = 20 + 20 + w + w + 20
        cv2.putText(img, 'Curvature: L {0:05}m, R {1:05}m'.format(int(left_curverad), int(right_curverad)),
                    (txt_x_loc, 80), cv2.FONT_HERSHEY_SIMPLEX, .8, (255, 255, 255), 2)

        cv2.putText(img, 'Vehicle Offset: {0:.3f} m'.format(camera_offset),
                    (txt_x_loc, 140), cv2.FONT_HERSHEY_SIMPLEX, .8, (255, 255, 255), 2)

        return img


