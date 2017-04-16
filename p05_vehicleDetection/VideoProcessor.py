import numpy as np
import cv2

import matplotlib.pyplot as plt
import io
import pickle
import time

from skimage.feature import hog
from scipy.ndimage.measurements import label

TEST_IMG_DIR = "./test_images/"
HEAT_MAX = 25
HEAT_MIN = 0
HEAT_THRESHOLD = 5

class VideoProcessor(object):
    '''
    Class to process the video images one by one.
    '''

    def __init__(self):

        self.first_time = True # so that we can initialize

        # Model was training using below features and parameters.
        dist_pickle = pickle.load(open("svc_pickle.p", "rb"))
        self.dist_pickle = dist_pickle
        self.svc = dist_pickle["svc"]
        self.X_scaler = dist_pickle["scaler"]
        self.orient = dist_pickle["orient"]
        self.pix_per_cell = dist_pickle["pix_per_cell"]
        self.cell_per_block = dist_pickle["cell_per_block"]
        self.spatial_size = dist_pickle["spatial_size"]
        self.hist_bins = dist_pickle["hist_bins"]

        self.color_space = 'YCrCb'  # ''RGB'  # Can be RGB, HSV, LUV, HLS, YUV, YCrCb  ( YCrCb = LUV? > HLS > HSV > > YUV > RGB)
        self.hog_channel = "ALL"  # 2  # Can be 0, 1, 2, or "ALL" ( "ALL" > 0 > 1> 2)
        self.spatial_feat = True  # Spatial features on or off
        self.hist_feat = True  # Histogram features on or off
        self.hog_feat = True  # HOG features on or off

        self.heat = None
        self.norm = plt.Normalize(vmin=HEAT_MIN, vmax=HEAT_MAX)

    def pipeline(self, img):

        ret_img, bin_img_heatmaps, bin_img_windows = self.find_cars_with_heatmap(img)


        norm_heatmap = self.norm(bin_img_heatmaps)
        # heatmap_img = (np.dstack((norm_heatmap * 255, norm_heatmap * 255 * 0.7, norm_heatmap * 255 * 0.7))).astype(np.uint8)
        # heatmap_img = (np.dstack((norm_heatmap * 255, norm_heatmap * 255, norm_heatmap * 255))).astype(np.uint8)
        heatmap_img = (np.dstack((norm_heatmap * 255, norm_heatmap, norm_heatmap))).astype(np.uint8)

        # bin_win = np.zeros_like(img[:, :, :]).astype(np.ubyte)

        bin_img_win_resized = cv2.resize(bin_img_windows, (0, 0), fx=0.3, fy=0.3)  #
        bin_img_heatmap_resized = cv2.resize(heatmap_img, (0, 0), fx=0.3, fy=0.3)  #
        ret_img[:250, :, :] = ret_img[:250, :, :] * .4
        (h, w, _) = bin_img_win_resized.shape

        ret_img[20:20 + h, 20:20 + w, :] = bin_img_win_resized

        ret_img[20:20 + h, 40 + w:40 + w + w, :] = bin_img_heatmap_resized

        # txt_x_loc = 20 + 20 + w + w + 20
        # cv2.putText(img, 'Curvature: L {0:05}m, R {1:05}m'.format(int(left_curverad), int(right_curverad)),
        #             (txt_x_loc, 80), cv2.FONT_HERSHEY_SIMPLEX, .8, (255, 255, 255), 2)
        #
        # cv2.putText(img, 'Vehicle Offset: {0:.3f} m'.format(camera_offset),
        #             (txt_x_loc, 140), cv2.FONT_HERSHEY_SIMPLEX, .8, (255, 255, 255), 2)
        return ret_img

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

    def convert_rgb_to_color(self, img, color_space):
        if color_space != 'RGB':
            if color_space == 'HSV':
                feature_image = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
            elif color_space == 'LUV':
                feature_image = cv2.cvtColor(img, cv2.COLOR_RGB2LUV)
            elif color_space == 'HLS':
                feature_image = cv2.cvtColor(img, cv2.COLOR_RGB2HLS)
            elif color_space == 'YUV':
                feature_image = cv2.cvtColor(img, cv2.COLOR_RGB2YUV)
            elif color_space == 'YCrCb':
                feature_image = cv2.cvtColor(img, cv2.COLOR_RGB2YCrCb)
        else:
            feature_image = np.copy(img)
        return feature_image

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

    # Define a function to extract features from a single image window
    # This function is very similar to extract_features()
    # just for a single image rather than list of images
    def single_img_features(self, img, color_space='RGB', spatial_size=(32, 32),
                            hist_bins=32, orient=9,
                            pix_per_cell=8, cell_per_block=2, hog_channel=0,
                            spatial_feat=True, hist_feat=True, hog_feat=True):
        # 1) Define an empty list to receive features
        img_features = []
        # 2) Apply color conversion if other than 'RGB'
        feature_image = self.convert_rgb_to_color(img, color_space)
        # 3) Compute spatial features if flag is set
        if spatial_feat == True:
            spatial_features = self.bin_spatial(feature_image, size=spatial_size)
            # 4) Append features to list
            img_features.append(spatial_features)
        # 5) Compute histogram features if flag is set
        if hist_feat == True:
            hist_features = self.color_hist(feature_image, nbins=hist_bins)
            # 6) Append features to list
            img_features.append(hist_features)
        # 7) Compute HOG features if flag is set
        if hog_feat == True:
            if hog_channel == 'ALL':
                hog_features = []
                for channel in range(feature_image.shape[2]):
                    hog_features.extend(self.get_hog_features(feature_image[:, :, channel],
                                                         orient, pix_per_cell, cell_per_block,
                                                         vis=False, feature_vec=True))
            else:
                hog_features = self.get_hog_features(feature_image[:, :, hog_channel], orient,
                                                pix_per_cell, cell_per_block, vis=False, feature_vec=True)
            # 8) Append features to list
            img_features.append(hog_features)

        # 9) Return concatenated array of features
        return np.concatenate(img_features)

    # Define a function that takes an image,
    # start and stop positions in both x and y,
    # window size (x and y dimensions),
    # and overlap fraction (for both x and y)
    def slide_window(self, img, x_start_stop=[None, None], y_start_stop=[None, None],
                     xy_window=(64, 64), xy_overlap=(0.5, 0.5)):
        # If x and/or y start/stop positions not defined, set to image size
        if x_start_stop[0] == None:
            x_start_stop[0] = 0
        if x_start_stop[1] == None:
            x_start_stop[1] = img.shape[1]
        if y_start_stop[0] == None:
            y_start_stop[0] = 0
        if y_start_stop[1] == None:
            y_start_stop[1] = img.shape[0]
        # Compute the span of the region to be searched
        xspan = x_start_stop[1] - x_start_stop[0]
        yspan = y_start_stop[1] - y_start_stop[0]
        # Compute the number of pixels per step in x/y
        nx_pix_per_step = np.int(xy_window[0] * (1 - xy_overlap[0]))
        ny_pix_per_step = np.int(xy_window[1] * (1 - xy_overlap[1]))
        # Compute the number of windows in x/y
        nx_buffer = np.int(xy_window[0] * (xy_overlap[0]))
        ny_buffer = np.int(xy_window[1] * (xy_overlap[1]))
        nx_windows = np.int((xspan - nx_buffer) / nx_pix_per_step)
        ny_windows = np.int((yspan - ny_buffer) / ny_pix_per_step)
        # Initialize a list to append window positions to
        window_list = []
        # Loop through finding x and y window positions
        # Note: you could vectorize this step, but in practice
        # you'll be considering windows one by one with your
        # classifier, so looping makes sense
        for ys in range(ny_windows):
            for xs in range(nx_windows):
                # Calculate window position
                startx = xs * nx_pix_per_step + x_start_stop[0]
                endx = startx + xy_window[0]
                starty = ys * ny_pix_per_step + y_start_stop[0]
                endy = starty + xy_window[1]

                # Append window position to list
                window_list.append(((startx, starty), (endx, endy)))
        # Return the list of windows
        return window_list


    # Define a function you will pass an image
    # and the list of windows to be searched (output of slide_windows())
    def search_windows(self, img, windows):
        svc = self.svc
        X_scaler = self.X_scaler
        color_space = self.color_space
        spatial_size = self.spatial_size
        hist_bins = self.hist_bins
        orient = self.orient
        pix_per_cell = self.pix_per_cell
        cell_per_block = self.cell_per_block
        hog_channel = self.hog_channel
        spatial_feat = self.spatial_feat
        hist_feat = self.hist_feat
        hog_feat = self.hog_feat

        # 1) Create an empty list to receive positive detection windows
        on_windows = []
        # 2) Iterate over all windows in the list
        for window in windows:
            # 3) Extract the test window from original image
            test_img = cv2.resize(img[window[0][1]:window[1][1], window[0][0]:window[1][0]], (64, 64))
            # 4) Extract features for that window using single_img_features()
            features = self.single_img_features(test_img, color_space=color_space,
                                           spatial_size=spatial_size, hist_bins=hist_bins,
                                           orient=orient, pix_per_cell=pix_per_cell,
                                           cell_per_block=cell_per_block,
                                           hog_channel=hog_channel, spatial_feat=spatial_feat,
                                           hist_feat=hist_feat, hog_feat=hog_feat)
            # 5) Scale extracted features to be fed to classifier
            test_features = X_scaler.transform(np.array(features).reshape(1, -1))
            # 6) Predict using your classifier
            prediction = svc.predict(test_features)
            # 7) If positive (prediction == 1) then save the window
            if prediction == 1:
                on_windows.append(window)
        # 8) Return windows for positive detections
        return on_windows

    def add_heat(self, heatmap, bbox_list):
        # Iterate through list of bboxes
        for box in bbox_list:
            # Add += 1 for all pixels inside each bbox
            # Assuming each "box" takes the form ((x1, y1), (x2, y2))
            heatmap[box[0][1]:box[1][1], box[0][0]:box[1][0]] += 1

        # Return updated heatmap
        return heatmap


    def apply_threshold(self, heatmap, threshold):
        # Zero out pixels below the threshold
        heatmap[heatmap <= threshold] = 0
        # Return thresholded map
        return heatmap

    # Define a function to draw bounding boxes
    def draw_boxes(self, img, bboxes, color=(0, 0, 255), thick=6):
        # Make a copy of the image
        imcopy = np.copy(img)
        # Iterate through the bounding boxes
        for bbox in bboxes:
            # Draw a rectangle given bbox coordinates
            cv2.rectangle(imcopy, bbox[0], bbox[1], color, thick)
        # Return the image copy with boxes drawn
        return imcopy

    def draw_labeled_bboxes(self, img, labels):
        # Iterate through all detected cars
        for car_number in range(1, labels[1] + 1):
            # Find pixels with each car_number label value
            nonzero = (labels[0] == car_number).nonzero()  # nonzero returns tuple of arrays, one for each dimension.
            # Identify x and y values of those pixels
            nonzeroy = np.array(nonzero[0])
            nonzerox = np.array(nonzero[1])
            # Define a bounding box based on min/max x and y
            xl = np.min(nonzerox)
            xh = np.max(nonzerox)
            yl = np.min(nonzeroy)
            yh = np.max(nonzeroy)

            # print ("### X/y = {}".format( int( (yh-yl)/(xh-xl))))

            if (int( (yh-yl)/(xh-xl)) < 2):
                bbox = ((xl, yl), (xh, yh))

                # Draw the box on the image
                cv2.rectangle(img, bbox[0], bbox[1], (0, 0, 255), 6)
        # Return the image
        return img

    def find_cars_at_scale(self, ctrans_tosearch, ystart, ystop, scale):
        '''
        
        :param ctrans_tosearch: image segment to be scanned already in choosen color space  
        :param ystart: y start in the original image
        :param ystop:  y stop in the original image
        :param scale: scale factor, 1 for 64x64, 1.5 for 96x96, 0.5 for 32x32
         
        :return: box_list in original image co-ordinates using ystart.
        '''
        svc = self.svc
        X_scaler = self.X_scaler
        spatial_size = self.spatial_size
        hist_bins = self.hist_bins
        orient = self.orient
        pix_per_cell = self.pix_per_cell
        cell_per_block = self.cell_per_block

        # hog_channel = self.hog_channel
        # spatial_feat = self.spatial_feat
        # hist_feat = self.hist_feat
        # hog_feat = self.hog_feat

        if scale != 1:
            imshape = ctrans_tosearch.shape
            ctrans_tosearch = cv2.resize(ctrans_tosearch, (np.int(imshape[1] / scale), np.int(imshape[0] / scale)))

        ch1 = ctrans_tosearch[:, :, 0]
        ch2 = ctrans_tosearch[:, :, 1]
        ch3 = ctrans_tosearch[:, :, 2]

        # Define blocks and steps as above
        nxblocks = (ch1.shape[1] // pix_per_cell) - 1
        nyblocks = (ch1.shape[0] // pix_per_cell) - 1
        # nfeat_per_block = orient * cell_per_block ** 2
        # 64 was the orginal sampling rate, with 8 cells and 8 pix per cell
        window = 64
        nblocks_per_window = (window // pix_per_cell) - 1
        cells_per_step = 2  # Instead of overlap, define how many cells to step
        nxsteps = (nxblocks - nblocks_per_window) // cells_per_step
        nysteps = (nyblocks - nblocks_per_window) // cells_per_step

        # Compute individual channel HOG features for the entire image
        hog1 = self.get_hog_features(ch1, orient, pix_per_cell, cell_per_block, feature_vec=False)
        hog2 = self.get_hog_features(ch2, orient, pix_per_cell, cell_per_block, feature_vec=False)
        hog3 = self.get_hog_features(ch3, orient, pix_per_cell, cell_per_block, feature_vec=False)

        box_list = []
        nv_box_list = []

        for xb in range(nxsteps):
            for yb in range(nysteps):
                ypos = yb * cells_per_step
                xpos = xb * cells_per_step
                # Extract HOG for this patch
                hog_feat1 = hog1[ypos:ypos + nblocks_per_window, xpos:xpos + nblocks_per_window].ravel()
                hog_feat2 = hog2[ypos:ypos + nblocks_per_window, xpos:xpos + nblocks_per_window].ravel()
                hog_feat3 = hog3[ypos:ypos + nblocks_per_window, xpos:xpos + nblocks_per_window].ravel()
                hog_features = np.hstack((hog_feat1, hog_feat2, hog_feat3))

                xleft = xpos * pix_per_cell
                ytop = ypos * pix_per_cell

                # Extract the image patch
                #subimg = cv2.resize(ctrans_tosearch[ytop:ytop + window, xleft:xleft + window], (64, 64)) # already 64x64, resize needed?
                subimg = ctrans_tosearch[ytop:ytop + window, xleft:xleft + window]

                # Get color features
                spatial_features = self.bin_spatial(subimg, size=spatial_size)
                hist_features = self.color_hist(subimg, nbins=hist_bins)

                # Scale features and make a prediction
                test_features = X_scaler.transform(
                    np.hstack((spatial_features, hist_features, hog_features)).reshape(1, -1))
                # test_features = X_scaler.transform(np.hstack((shape_feat, hist_feat)).reshape(1, -1))
                test_prediction = svc.predict(test_features)

                if test_prediction == 1:
                    xbox_left = np.int(xleft * scale)
                    ytop_draw = np.int(ytop * scale)
                    win_draw = np.int(window * scale)
                    box_list.append([(xbox_left, ytop_draw + ystart),
                                     (xbox_left + win_draw, ytop_draw + win_draw + ystart) ]) # each "box" takes the form ((x1, y1), (x2, y2))
                    # cv2.rectangle(draw_img, (xbox_left, ytop_draw + ystart),
                    #              (xbox_left + win_draw, ytop_draw + win_draw + ystart), (0, 0, 255), 6)
                # else:
                #     xbox_left = np.int(xleft * scale)
                #     ytop_draw = np.int(ytop * scale)
                #     win_draw = np.int(window * scale)
                #     nv_box_list.append([(xbox_left, ytop_draw + ystart),
                #                      (xbox_left + win_draw, ytop_draw + win_draw + ystart) ])

        return box_list, nv_box_list


    def draw_windows_all_scales(self, img, jpg_image=True):
        # t = time.time()

        color_space = self.color_space

        ystart = 400
        ystop = 656
        scale = 1.5

        draw_img = np.copy(img)

        img_tosearch = img[ystart:ystop, :, :]

        if (jpg_image):
            img_tosearch = img_tosearch.astype(np.float32) / 255 # If test image is JPG, MPEG stream. bcoz we trained in PNG

        ctrans_tosearch = self.convert_rgb_to_color(img_tosearch, color_space)

        box_list, nv_box_list = self.find_cars_at_scale(ctrans_tosearch, ystart, ystop, scale)
        draw_img = self.draw_boxes(draw_img, box_list + nv_box_list, color=(255, 0, 0), thick=3)

        ystop = 528
        scale = 1.0
        ctrans_tosearch = ctrans_tosearch[0:128, :, :]
        box_list, nv_box_list = self.find_cars_at_scale(ctrans_tosearch, ystart, ystop, scale)
        #draw_img = self.draw_boxes(draw_img, box_list + nv_box_list, color=(0, 255, 0), thick=2)


        ystop = 464
        scale = 0.75
        ctrans_tosearch = ctrans_tosearch[0:64, :, :]
        box_list, nv_box_list =  self.find_cars_at_scale(ctrans_tosearch, ystart, ystop, scale)
        # draw_img = self.draw_boxes(draw_img, box_list + nv_box_list, color=(0, 0, 255), thick=2)


        fig = plt.figure()
        plt.subplot(111)
        plt.imshow(draw_img)
        plt.title('Sliding Windows and Scales')
        plt.show()


    # Define a single function that can extract features using hog sub-sampling and make predictions
    def find_cars_with_heatmap(self, img, jpg_image=True):
        # t = time.time()

        color_space = self.color_space

        ystart = 400
        ystop = 656
        scale = 1.5

        draw_img = np.copy(img)

        img_tosearch = img[ystart:ystop, :, :]

        if (jpg_image):
            img_tosearch = img_tosearch.astype(np.float32) / 255 # If test image is JPG, MPEG stream. bcoz we trained in PNG

        ctrans_tosearch = self.convert_rgb_to_color(img_tosearch, color_space)

        box_list, nv_box_list = self.find_cars_at_scale(ctrans_tosearch, ystart, ystop, scale)

        ystop = 528
        scale = 1.0
        ctrans_tosearch = ctrans_tosearch[0:128, :, :]
        box_list_t, nv_box_list_t = self.find_cars_at_scale(ctrans_tosearch, ystart, ystop, scale)
        box_list = box_list + box_list_t
        # nv_box_list = nv_box_list + nv_box_list_t

        ystop = 464
        scale = 0.75
        ctrans_tosearch = ctrans_tosearch[0:64, :, :]
        box_list_t, nv_box_list_t =  self.find_cars_at_scale(ctrans_tosearch, ystart, ystop, scale)
        box_list = box_list + box_list_t

        # print("### number of boxes predicted {}".format(len(box_list)))

        bin_win = np.zeros_like(img[:, :, :]).astype(np.ubyte)
        bin_win = self.draw_boxes(bin_win, box_list)
        # draw_img = self.draw_boxes(draw_img, nv_box_list, color=(255, 0, 0), thick=2 )

        if (self.first_time):
            self.first_time = False
            self.heat = np.zeros_like(img[:, :, 0]).astype(np.float)


        self.heat -= 1
        self.heat[self.heat < HEAT_MIN] = HEAT_MIN  # set -ve ones to zeros.
        self.heat[self.heat > HEAT_MAX] = HEAT_MAX

        # Add heat to each box in box list
        heat = self.add_heat(self.heat, box_list)

        # Apply threshold to help remove false positives
        heat = self.apply_threshold(heat, HEAT_THRESHOLD)

        # Visualize the heatmap when displaying
        heatmap = np.clip(heat, 0, 255)

        # Find final boxes from heatmap using label function
        labels = label(heatmap)

        # print("### # of Labels: {}".format(labels[1]))

        # draw_img = self.draw_labeled_bboxes(np.copy(draw_img), labels)
        draw_img = self.draw_labeled_bboxes(draw_img, labels)

        # t2 = time.time()
        # print(round(t2 - t, 2), 'Seconds to predict a image SVC...')
        #
        # fig = plt.figure()
        # plt.subplot(131)
        # plt.imshow(draw_img)
        # plt.title('Car Positions')
        # plt.subplot(132)
        # plt.imshow(heatmap, cmap='hot')
        # plt.title('Heat Map')
        # plt.subplot(133)
        # plt.imshow(bin_win)
        # plt.title('Bin Windows')
        #
        # fig.tight_layout()
        # plt.show()

        return draw_img, heatmap,bin_win
