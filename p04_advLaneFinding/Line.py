import numpy as np
import cv2

# Set the width of the windows +/- margin
margin = 100
# Set minimum number of pixels found to recenter window
minpix = 50
# Choose the number of sliding windows
nwindows = 9
# number of lines to average.
nprev_lins = 5

class Line():
    def __init__(self):
        # was the line detected in the last iteration?
        self.detected = False
        # x values of the last n fits of the line
        self.recent_xfitted = []
        #average x values of the fitted line over the last n iterations
        self.bestx = None
        #polynomial coefficients averaged over the last n iterations
        self.best_fit = None
        #polynomial coefficients for the most recent fit
        self.current_fit = [np.array([False])]
        #radius of curvature of the line in some units
        self.radius_of_curvature = None
        #distance in meters of vehicle center from the line
        self.line_base_pos = None
        #difference in fit coefficients between last and new fits
        self.diffs = np.array([0,0,0], dtype='float')
        #x values for detected line pixels
        self.allx = None
        #y values for detected line pixels
        self.ally = None

        # Refactored
        self.nonzeroy = None #Array of Y co-ord for current non zero line pixel on wrpIm
        self.nonzerox = None

        self.ploty = None

        self.lane_fit = None
        self.lane_fitx = None
        self.lane_inds = None

        self.prev_lines_fit = np.empty(shape=[0,3])


    def valid_line(self, prev_lane_fit, curr_lane_fit, curr_lane_pos, ploty):
        ret = True

        y_eval = np.max(ploty)
        curr_x_at_ymax = curr_lane_fit[0] * y_eval ** 2 + curr_lane_fit[1] * y_eval + curr_lane_fit[2]
        curr_x_at_ymin = curr_lane_fit[2]

        if (prev_lane_fit != None):
            prev_x_at_ymax = prev_lane_fit[0] * y_eval ** 2 + prev_lane_fit[1] * y_eval + prev_lane_fit[2]
            prev_x_at_ymin = prev_lane_fit[2]

            if (abs (prev_x_at_ymax - curr_x_at_ymax) > (2 * margin)):
                ret = False

            if (abs (prev_x_at_ymin - curr_x_at_ymin) > (2 * margin)):
                ret = False

        if (abs (curr_lane_pos - curr_x_at_ymin) > (2 * margin)):
            ret = False

        return ret

    def poly_fit_line(self, line_current_pos, binary_warped, out_img):

        laney, lanex = self.find_lane_thru_windows(line_current_pos, binary_warped, out_img)

        ploty = np.linspace(0, binary_warped.shape[0] - 1, binary_warped.shape[0])

        # Fit a second order polynomial to each
        if (len(laney) > 0):
            lane_fit = np.polyfit(laney, lanex, 2)
            if (self.valid_line(self.lane_fit, lane_fit, line_current_pos, ploty)):
                self.line_base_pos = line_current_pos

                if (self.prev_lines_fit.shape[0] > nprev_lins):
                    self.prev_lines_fit = np.vstack([self.prev_lines_fit[1:], lane_fit])
                else:
                    self.prev_lines_fit = np.vstack([self.prev_lines_fit, lane_fit])

                self.lane_fit = np.average(self.prev_lines_fit, axis = 0)
        else:
            # lane_fit = [0, 0, self.line_base_pos]
            self.lane_fit = np.average(self.prev_lines_fit, axis=0)

        # visualize
        # Generate x and y values for plotting
        lane_fitx = self.lane_fit[0] * ploty ** 2 + self.lane_fit[1] * ploty + self.lane_fit[2]

        self.lane_fitx = lane_fitx
        self.ploty = ploty

        out_img[laney, lanex] = [255, 0, 0] # pixels from binary images as lane line

        # plt.imshow(out_img)
        # plt.plot(left_fitx, ploty, color='yellow')
        # plt.plot(right_fitx, ploty, color='yellow')
        # plt.xlim(0, 1280)
        # plt.ylim(720, 0)
        # plt.show()

        line_pts = np.stack((lane_fitx, ploty), axis=1)  # lane fit

        cv2.polylines(out_img, [line_pts.astype('int32').reshape((-1,1,2))], isClosed=False, color=[255,255,0]
                      , thickness=3, lineType=cv2.LINE_AA)

        # plt.imshow(out_img)
        # plt.show()

        return out_img

    def find_lane_thru_windows(self, line_current_pos, binary_warped, out_img):
        # Choose the number of sliding windows
        nwindows = 9
        # Set height of windows
        window_height = np.int(binary_warped.shape[0] / nwindows)
        # Identify the x and y positions of all nonzero pixels in the image
        nonzero = binary_warped[:, :, 1].nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        # Create empty lists to receive left and right lane pixel indices
        lane_inds = []
        # Step through the windows one by one
        for window in range(nwindows):
            # Identify window boundaries in x and y (and right and left)
            win_y_low = binary_warped.shape[0] - (window + 1) * window_height
            win_y_high = binary_warped.shape[0] - window * window_height
            win_xleft_low = line_current_pos - margin
            win_xleft_high = line_current_pos + margin

            # Draw the windows on the visualization image
            cv2.rectangle(out_img, (win_xleft_low, win_y_low), (win_xleft_high, win_y_high), (0, 255, 0), 2)
            # Identify the nonzero pixels in x and y within the window
            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xleft_low) & (
                nonzerox < win_xleft_high)).nonzero()[0]

            # Append these indices to the lists
            lane_inds.append(good_left_inds)
            # If you found > minpix pixels, recenter next window on their mean position
            if len(good_left_inds) > minpix:
                line_current_pos = np.int(np.mean(nonzerox[good_left_inds]))

        lane_inds = np.concatenate(lane_inds) # Concatenate the arrays of indices

        # Extract left and right line pixel positions
        lanex = nonzerox[lane_inds]
        laney = nonzeroy[lane_inds]
        return laney, lanex


    def measure_radius(self):

        # Define y-value where we want radius of curvature
        # I'll choose the maximum y-value, corresponding to the bottom of the image
        y_eval = np.max(self.ploty)

        # Define conversions in x and y from pixels space to meters
        ym_per_pix = 30 / 720  # meters per pixel in y dimension
        xm_per_pix = 3.7 / 700  # meters per pixel in x dimension

        # Fit new polynomials to x,y in world space
        lane_fit_cr = np.polyfit(self.ploty * ym_per_pix, self.lane_fitx * xm_per_pix, 2)

        # Calculate the new radii of curvature
        lane_curverad = ((1 + (
        2 * lane_fit_cr[0] * y_eval * ym_per_pix + lane_fit_cr[1]) ** 2) ** 1.5) / np.absolute(
            2 * lane_fit_cr[0])

        if (lane_curverad != lane_curverad):
            print("lane_curverad is NaN: y_eval {} lane_fit_cr {}".format(y_eval, lane_fit_cr))
            lane_curverad = 0.0

        return lane_curverad


            # import collections
#
# class AveragingBuffer(object):
#     def __init__(self, maxlen):
#         assert( maxlen>1)
#         self.q=collections.deque(maxlen=maxlen)
#         self.xbar=0.0
#     def append(self, x):
#         if len(self.q)==self.q.maxlen:
#             # remove first item, update running average
#             d=self.q.popleft()
#             self.xbar=self.xbar+(self.xbar-d)/float(len(self.q))
#         # append new item, update running average
#         self.q.append(x)
#         self.xbar=self.xbar+(x-self.xbar)/float(len(self.q))
#
#
# if __name__=="__main__":
#     import scipy
#     ab=AveragingBuffer(10)
#     for i in range(32):
#         ab.append(scipy.rand())
#         print (ab.xbar, scipy.average(ab.q), len(ab.q))