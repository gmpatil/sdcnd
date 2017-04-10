import numpy as np
import cv2

import matplotlib.pyplot as plt
import io

from skimage.feature import hog


TEST_IMG_DIR = "./test_images/"


class VideoProcessor(object):
    '''
    Class to process the video images one by one.
    '''

    def __init__(self):

        self.first_time = True # so that we can initialize


    def pipeline(self, img):

        return img

    def convert_color(self, img, conv='RGB2YCrCb'):

        if conv == 'RGB2HLS':
            return cv2.cvtColor(img, cv2.COLOR_RGB2HLS)
        if conv == 'RGB2HSV':
            return cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        if conv == 'RGB2YCrCb':
            return cv2.cvtColor(img, cv2.COLOR_RGB2YCrCb)
        if conv == 'BGR2YCrCb':
            return cv2.cvtColor(img, cv2.COLOR_BGR2YCrCb)
        if conv == 'RGB2LUV':
            return cv2.cvtColor(img, cv2.COLOR_RGB2LUV)

    def bin_spatial(self, img, size=(32, 32)):
        color1 = cv2.resize(img[:, :, 0], size).ravel()
        color2 = cv2.resize(img[:, :, 1], size).ravel()
        color3 = cv2.resize(img[:, :, 2], size).ravel()
        return np.hstack((color1, color2, color3))

    def color_hist(self, img, nbins=32):  # bins_range=(0, 256)
        # Compute the histogram of the color channels separately
        channel1_hist = np.histogram(img[:, :, 0], bins=nbins)
        channel2_hist = np.histogram(img[:, :, 1], bins=nbins)
        channel3_hist = np.histogram(img[:, :, 2], bins=nbins)
        # Concatenate the histograms into a single feature vector
        hist_features = np.concatenate((channel1_hist[0], channel2_hist[0], channel3_hist[0]))
        # Return the individual histograms, bin_centers and feature vector
        return hist_features

    def get_hog_features(self, img, orient, pix_per_cell, cell_per_block,
                         vis=False, feature_vec=True):
        # Call with two outputs if vis==True
        if vis == True:
            features, hog_image = hog(img, orientations=orient,
                                      pixels_per_cell=(pix_per_cell, pix_per_cell),
                                      cells_per_block=(cell_per_block, cell_per_block),
                                      transform_sqrt=False,
                                      visualise=vis, feature_vector=feature_vec)
            return features, hog_image
        # Otherwise call with one output
        else:
            features = hog(img, orientations=orient,
                           pixels_per_cell=(pix_per_cell, pix_per_cell),
                           cells_per_block=(cell_per_block, cell_per_block),
                           transform_sqrt=False,
                           visualise=vis, feature_vector=feature_vec)
            return features


    def add_heat(self, heatmap, bbox_list):
        # Iterate through list of bboxes
        for box in bbox_list:
            # Add += 1 for all pixels inside each bbox
            # Assuming each "box" takes the form ((x1, y1), (x2, y2))
            heatmap[box[0][1]:box[1][1], box[0][0]:box[1][0]] += 1

        # Return updated heatmap
        return heatmap  # Iterate through list of bboxes


    def apply_threshold(self, heatmap, threshold):
        # Zero out pixels below the threshold
        heatmap[heatmap <= threshold] = 0
        # Return thresholded map
        return heatmap


    def draw_labeled_bboxes(self, img, labels):
        # Iterate through all detected cars
        for car_number in range(1, labels[1] + 1):
            # Find pixels with each car_number label value
            nonzero = (labels[0] == car_number).nonzero()  # nonzero returns tuple of arrays, one for each dimension.
            # Identify x and y values of those pixels
            nonzeroy = np.array(nonzero[0])
            nonzerox = np.array(nonzero[1])
            # Define a bounding box based on min/max x and y
            bbox = ((np.min(nonzerox), np.min(nonzeroy)), (np.max(nonzerox), np.max(nonzeroy)))
            # Draw the box on the image
            cv2.rectangle(img, bbox[0], bbox[1], (0, 0, 255), 6)
        # Return the image
        return img
