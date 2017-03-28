import numpy as np
import cv2
from Line import Line

import matplotlib.pyplot as plt
from Camera import Camera
import io

CALIBRATION_IMG_DIR = "./camera_cal/"
TEST_IMG_DIR = "./test_images/"


class VideoProcessor(object):
    '''
    Class to process the video images one by one.
    '''

    def __init__(self, camera):
        '''
        :param camera: instance of Camera, initialized/loaded with camera calibration parameters.
        '''
        self.cam = camera

        self.first_time = True # so that we can initialize

        self.nonzeroy = None
        self.nonzerox = None

        self.ploty = None

        self.left_fit = None
        self.left_fitx = None
        self.left_lane_inds = None

        self.right_fit = None
        self.right_fitx = None
        self.right_lane_inds = None

    def thresholded_binary(self, img, s_thresh=(170, 255), sx_thresh=(20, 100)):
        h_thresh = (15, 100)

        img2 = np.copy(img)
        # Convert to HSV color space and separate the V channel
        hsv = cv2.cvtColor(img2, cv2.COLOR_RGB2HLS).astype(np.float)
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
        sh_binary = np.zeros_like(h_channel)

        # S AND H
        sh_binary[((h_channel >= h_thresh[0]) & (h_channel <= h_thresh[1])) &
                  ((s_channel >= s_thresh[0]) & (s_channel <= s_thresh[1]) )] = 1

        # # S OR H
        # sh_binary[((h_channel >= h_thresh[0]) & (h_channel <= h_thresh[1])) |
        #           ((s_channel >= s_thresh[0]) & (s_channel <= s_thresh[1]) )] = 1

        # Stack each channel
        color_binary = np.dstack((np.zeros_like(sxbinary), sxbinary, sh_binary))
        # color_binary = np.dstack((np.zeros_like(sxbinary), sxbinary, s_binary))
        # color_binary = np.dstack((h_binary, sxbinary, s_binary))

        return color_binary

    def warper(self, img):
        img_size = (img.shape[1], img.shape[0])

        # src = np.float32(
        #     [[(img_size[0] / 2) - 55, img_size[1] / 2 + 100],
        #      [((img_size[0] / 6) - 10), img_size[1]],
        #      [(img_size[0] * 5 / 6) + 60, img_size[1]],
        #      [(img_size[0] / 2 + 55), img_size[1] / 2 + 100]])

        # src = np.float32(
        #     [[559.767, 477.491],    # 0.437 * 1280, 0.663 * 720
        #      [294.809, 659.737],    # 0.230 * 1280, 0.916 * 720
        #      [1011.18, 659.737],    # 0.790 * 1280, 0.916 * 720
        #      [726.593, 477.491]])   # 0.568 * 1280, 0.663 * 720

        src = np.float32(
            [[(img_size[0] * 0.437), img_size[1] * 0.663],
             [(img_size[0] * 0.230), img_size[1] * 0.916],
             [(img_size[0] * 0.790), img_size[1] * 0.916],
             [(img_size[0] * 0.568), img_size[1] * 0.663]])

        dst = np.float32(
            [[(img_size[0] / 4), 0],
             [(img_size[0] / 4), img_size[1]],
             [(img_size[0] * 3 / 4), img_size[1]],
             [(img_size[0] * 3 / 4), 0]])

        # Compute and apply perpective transform
        img_size = (img.shape[1], img.shape[0])
        M = cv2.getPerspectiveTransform(src, dst)
        warped = cv2.warpPerspective(img, M, img_size, flags=cv2.INTER_NEAREST)  # keep same size as input image
        Minv = cv2.getPerspectiveTransform(dst, src)
        return (warped, Minv)

    def get_binary_warped_image(self, img):
        '''
        Takes an image, creates binary image and warped image and returns all the 3.
        :param img:
        :return: list of undistorted image, binary image and warped/perspective changed binary image
        '''
        ret = []
        img = self.cam.undistort(img)
        bin = self.thresholded_binary(img)
        (binp, Minv) = self.warper(bin)
        ret.append(img)
        ret.append(bin)
        ret.append(binp)

        # fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(4 * 3, 4))
        # ax1.imshow(img)
        # ax2.imshow(bin)
        # ax3.imshow(binp)
        # plt.show()

        return (ret, Minv)

    def poly_fit_line(self, binary_warped):
        histogram = np.sum(binary_warped[int(binary_warped.shape[0] / 2):, :], axis=0)

        # Create an output image to draw on and  visualize the result
        if (binary_warped.shape[-1] == 3):
            out_img = np.zeros_like(binary_warped, np.uint8)
        else:
            out_img = np.dstack((binary_warped, binary_warped, binary_warped)) * 255
        # Find the peak of the left and right halves of the histogram
        # These will be the starting point for the left and right lines
        midpoint = np.int(histogram.shape[0] / 2)
        leftx_base = np.argmax(histogram[:midpoint,1])
        rightx_base = np.argmax(histogram[midpoint:,1]) + midpoint

        if (self.first_time):
            self.first_time = False

            self.line_left = Line()
            self.line_right = Line()

        out_img = self.line_left.poly_fit_line(leftx_base, binary_warped, out_img)
        out_img = self.line_right.poly_fit_line(rightx_base, binary_warped, out_img)

        return out_img


    # def poly_fit_first(self, binary_warped):
    #     '''
    #     From SDCND Term1 course section "Advanced Lane Finding", lesson 31 "Finding Lane Line"
    #
    #     :param binary_warped: Binary warped image
    #     :return:
    #     '''
    #     # binary_warped = cv2.cvtColor(np.array(binary_warped, dtype=np.uint8), cv2.COLOR_RGB2GRAY)
    #
    #     # Assuming you have created a warped binary image called "binary_warped"
    #     # Take a histogram of the bottom half of the image
    #     histogram = np.sum(binary_warped[int(binary_warped.shape[0] / 2):, :], axis=0)
    #
    #     # Create an output image to draw on and  visualize the result
    #     if (binary_warped.shape[-1] == 3):
    #         out_img = np.zeros_like(binary_warped, np.uint8)
    #     else:
    #         out_img = np.dstack((binary_warped, binary_warped, binary_warped)) * 255
    #     # Find the peak of the left and right halves of the histogram
    #     # These will be the starting point for the left and right lines
    #     midpoint = np.int(histogram.shape[0] / 2)
    #     leftx_base = np.argmax(histogram[:midpoint,1])
    #     rightx_base = np.argmax(histogram[midpoint:,1]) + midpoint
    #
    #     # Choose the number of sliding windows
    #     nwindows = 9
    #     # Set height of windows
    #     window_height = np.int(binary_warped.shape[0] / nwindows)
    #     # Identify the x and y positions of all nonzero pixels in the image
    #     nonzero = binary_warped[:,:,1].nonzero()
    #     nonzeroy = np.array(nonzero[0])
    #     nonzerox = np.array(nonzero[1])
    #     # Current positions to be updated for each window
    #     leftx_current = leftx_base
    #     rightx_current = rightx_base
    #     # Set the width of the windows +/- margin
    #     margin = 100
    #     # Set minimum number of pixels found to recenter window
    #     minpix = 50
    #     # Create empty lists to receive left and right lane pixel indices
    #     left_lane_inds = []
    #     right_lane_inds = []
    #
    #     # Step through the windows one by one
    #     for window in range(nwindows):
    #         # Identify window boundaries in x and y (and right and left)
    #         win_y_low = binary_warped.shape[0] - (window + 1) * window_height
    #         win_y_high = binary_warped.shape[0] - window * window_height
    #         win_xleft_low = leftx_current - margin
    #         win_xleft_high = leftx_current + margin
    #         win_xright_low = rightx_current - margin
    #         win_xright_high = rightx_current + margin
    #         # Draw the windows on the visualization image
    #         cv2.rectangle(out_img, (win_xleft_low, win_y_low), (win_xleft_high, win_y_high), (0, 255, 0), 2)
    #         cv2.rectangle(out_img, (win_xright_low, win_y_low), (win_xright_high, win_y_high), (0, 255, 0), 2)
    #         # Identify the nonzero pixels in x and y within the window
    #         good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xleft_low) & (
    #         nonzerox < win_xleft_high)).nonzero()[0]
    #         good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xright_low) & (
    #         nonzerox < win_xright_high)).nonzero()[0]
    #         # Append these indices to the lists
    #         left_lane_inds.append(good_left_inds)
    #         right_lane_inds.append(good_right_inds)
    #         # If you found > minpix pixels, recenter next window on their mean position
    #         if len(good_left_inds) > minpix:
    #             leftx_current = np.int(np.mean(nonzerox[good_left_inds]))
    #         if len(good_right_inds) > minpix:
    #             rightx_current = np.int(np.mean(nonzerox[good_right_inds]))
    #
    #     # Concatenate the arrays of indices
    #     left_lane_inds = np.concatenate(left_lane_inds)
    #     right_lane_inds = np.concatenate(right_lane_inds)
    #
    #     # Extract left and right line pixel positions
    #     leftx = nonzerox[left_lane_inds]
    #     lefty = nonzeroy[left_lane_inds]
    #     rightx = nonzerox[right_lane_inds]
    #     righty = nonzeroy[right_lane_inds]
    #
    #     # Fit a second order polynomial to each
    #     if (len(lefty) > 0):
    #         self.left_fit = np.polyfit(lefty, leftx, 2)
    #     else:
    #         self.left_fit = [1,1,1]
    #
    #     if (len(righty) > 0):
    #         self.right_fit = np.polyfit(righty, rightx, 2)
    #     else:
    #         self.right_fit = [1,1,1]
    #
    #     self.nonzeroy = nonzeroy
    #     self.nonzerox = nonzerox
    #
    #     self.left_lane_inds = left_lane_inds
    #     self.right_lane_inds = right_lane_inds
    #
    #
    #     # plt.imshow(out_img)
    #     # plt.show()
    #     return out_img
    #
    #
    # def visualize_polyfit_first(self, binary_warped, out_img):
    #     # Generate x and y values for plotting
    #     left_fit = self.left_fit
    #     right_fit = self.right_fit
    #
    #     nonzeroy = self.nonzeroy
    #     nonzerox = self.nonzerox
    #
    #     left_lane_inds = self.left_lane_inds
    #     right_lane_inds = self.right_lane_inds
    #
    #     ploty = np.linspace(0, binary_warped.shape[0] - 1, binary_warped.shape[0])
    #     left_fitx = left_fit[0] * ploty ** 2 + left_fit[1] * ploty + left_fit[2]
    #     right_fitx = right_fit[0] * ploty ** 2 + right_fit[1] * ploty + right_fit[2]
    #
    #     self.left_fitx = left_fitx
    #     self.right_fitx = right_fitx
    #     self.ploty = ploty
    #
    #     out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [255, 0, 0]
    #     # out_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [0, 0, 255]
    #     out_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [255, 0, 0]
    #
    #     # plt.imshow(out_img)
    #     # plt.plot(left_fitx, ploty, color='yellow')
    #     # plt.plot(right_fitx, ploty, color='yellow')
    #     # plt.xlim(0, 1280)
    #     # plt.ylim(720, 0)
    #     # plt.show()
    #
    #     left_line_pts = np.stack((left_fitx, ploty), axis=1)
    #     right_line_pts = np.stack((right_fitx, ploty), axis=1)
    #
    #     cv2.polylines(out_img, [left_line_pts.astype('int32').reshape((-1,1,2))], isClosed=False, color=[255,255,0]
    #                   , thickness=3, lineType=cv2.LINE_AA)
    #     cv2.polylines(out_img, [right_line_pts.astype('int32').reshape((-1,1,2))], isClosed=False, color=[255,255,0]
    #                   , thickness=3, lineType=cv2.LINE_AA)
    #
    #     # plt.imshow(out_img)
    #     # plt.show()
    #     return out_img
    #
    #
    # def poly_fit(self, binary_warped):
    #     '''
    #     From SDCND Term1 course section "Advanced Lane Finding", lesson 31 "Finding Lane Line"
    #
    #     :return: blank image of binary_warped shape.
    #     '''
    #     # Assume you now have a new warped binary image
    #     # from the next frame of video (also called "binary_warped")
    #     # It's now much easier to find line pixels!
    #
    #     left_fit = self.left_fit
    #     right_fit = self.right_fit
    #
    #     # Create an output image to draw on and  visualize the result
    #     if (binary_warped.shape[-1] == 3):
    #         out_img = np.zeros_like(binary_warped, np.uint8)
    #     else:
    #         out_img = np.dstack((binary_warped, binary_warped, binary_warped)) * 255
    #
    #     nonzero = binary_warped[:, :, 1].nonzero()
    #     nonzeroy = np.array(nonzero[0])
    #     nonzerox = np.array(nonzero[1])
    #     margin = 100
    #     left_lane_inds = (
    #     (nonzerox > (left_fit[0] * (nonzeroy ** 2) + left_fit[1] * nonzeroy + left_fit[2] - margin)) & (
    #     nonzerox < (left_fit[0] * (nonzeroy ** 2) + left_fit[1] * nonzeroy + left_fit[2] + margin)))
    #     right_lane_inds = (
    #     (nonzerox > (right_fit[0] * (nonzeroy ** 2) + right_fit[1] * nonzeroy + right_fit[2] - margin)) & (
    #     nonzerox < (right_fit[0] * (nonzeroy ** 2) + right_fit[1] * nonzeroy + right_fit[2] + margin)))
    #
    #     # Again, extract left and right line pixel positions
    #     leftx = nonzerox[left_lane_inds]
    #     lefty = nonzeroy[left_lane_inds]
    #     rightx = nonzerox[right_lane_inds]
    #     righty = nonzeroy[right_lane_inds]
    #     # Fit a second order polynomial to each
    #
    #     # Generate x and y values for plotting
    #     ploty = np.linspace(0, binary_warped.shape[0] - 1, binary_warped.shape[0])
    #
    #
    #     if (len(lefty) > 100 and len(leftx) > 100):
    #         left_fit = np.polyfit(lefty, leftx, 2)
    #         left_fitx = left_fit[0] * ploty ** 2 + left_fit[1] * ploty + left_fit[2]
    #
    #         self.left_fit = left_fit
    #         self.left_fitx = left_fitx
    #     else:
    #         print ("Not sufficient pixel for left line {} , {} ".format(len(lefty), len(leftx)))
    #
    #     if (len(righty) > 100 and len(rightx) > 100):
    #         right_fit = np.polyfit(righty, rightx, 2)
    #         right_fitx = right_fit[0] * ploty ** 2 + right_fit[1] * ploty + right_fit[2]
    #
    #         self.right_fit = right_fit
    #         self.right_fitx = right_fitx
    #     else:
    #         print ("Not sufficient pixel for right line {} , {} ".format(len(righty), len(rightx)))
    #
    #
    #     self.ploty = ploty
    #
    #     self.nonzeroy = nonzeroy
    #     self.nonzerox = nonzerox
    #
    #     self.left_lane_inds = left_lane_inds
    #     self.right_lane_inds = right_lane_inds
    #
    #     return out_img
    #
    #
    # def visualize_polyfit(self, binary_warped, out_img):
    #     '''
    #
    #     :param binary_warped:
    #     :param out_img:
    #     :return: Warped binary image with Windows, lane lines identified
    #     '''
    #     margin = 100
    #
    #     nonzeroy = self.nonzeroy
    #     nonzerox = self.nonzerox
    #
    #     left_lane_inds = self.left_lane_inds
    #     right_lane_inds = self.right_lane_inds
    #
    #     left_fitx = self.left_fitx
    #     right_fitx = self.right_fitx
    #     ploty = self.ploty
    #
    #     # Create an image to draw on and an image to show the selection window
    #     if (binary_warped.shape[-1] == 3):
    #         out_img = np.zeros_like(binary_warped, np.uint8)
    #     else:
    #         out_img = np.dstack((binary_warped, binary_warped, binary_warped)) * 255
    #
    #     window_img = np.zeros_like(out_img)
    #     # Color in left and right line pixels
    #     out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [255, 0, 0]
    #     # out_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [0, 0, 255]
    #     out_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [255, 0, 0]
    #
    #     # Generate a polygon to illustrate the search window area
    #     # And recast the x and y points into usable format for cv2.fillPoly()
    #     left_line_window1 = np.array([np.transpose(np.vstack([left_fitx - margin, ploty]))])
    #     left_line_window2 = np.array([np.flipud(np.transpose(np.vstack([left_fitx + margin, ploty])))])
    #     left_line_pts = np.hstack((left_line_window1, left_line_window2))
    #     right_line_window1 = np.array([np.transpose(np.vstack([right_fitx - margin, ploty]))])
    #     right_line_window2 = np.array([np.flipud(np.transpose(np.vstack([right_fitx + margin, ploty])))])
    #     right_line_pts = np.hstack((right_line_window1, right_line_window2))
    #
    #     # Draw the lane onto the warped blank image
    #     cv2.fillPoly(window_img, np.int_([left_line_pts]), (0, 255, 0))
    #     cv2.fillPoly(window_img, np.int_([right_line_pts]), (0, 255, 0))
    #     result = cv2.addWeighted(out_img, 1, window_img, 0.3, 0)
    #
    #     # plt.imshow(result)
    #     # plt.plot(left_fitx, ploty, color='yellow')
    #     # plt.plot(right_fitx, ploty, color='yellow')
    #     # plt.xlim(0, 1280)
    #     # plt.ylim(720, 0)
    #     # plt.show()
    #
    #     left_line_pts = np.stack((left_fitx, ploty), axis=1)
    #     right_line_pts = np.stack((right_fitx, ploty), axis=1)
    #
    #     cv2.polylines(result, [left_line_pts.astype('int32').reshape((-1,1,2))], isClosed=False, color=[255,255,0]
    #                   , thickness=3, lineType=cv2.LINE_AA)
    #     cv2.polylines(result, [right_line_pts.astype('int32').reshape((-1,1,2))], isClosed=False, color=[255,255,0]
    #                   , thickness=3, lineType=cv2.LINE_AA)
    #
    #     return result

    def measure_radius(self):
        left_fit = self.left_fit
        right_fit = self.right_fit
        ploty = self.ploty

        leftx = self.left_fitx
        rightx = self.right_fitx

        # Define y-value where we want radius of curvature
        # I'll choose the maximum y-value, corresponding to the bottom of the image
        y_eval = np.max(ploty)
        left_curverad = ((1 + (2 * left_fit[0] * y_eval + left_fit[1]) ** 2) ** 1.5) / np.absolute(2 * left_fit[0])
        right_curverad = ((1 + (2 * right_fit[0] * y_eval + right_fit[1]) ** 2) ** 1.5) / np.absolute(2 * right_fit[0])
        # print(left_curverad, right_curverad)
        # Example values: 1926.74 1908.48

        # Define conversions in x and y from pixels space to meters
        ym_per_pix = 30 / 720  # meters per pixel in y dimension
        xm_per_pix = 3.7 / 700  # meters per pixel in x dimension

        # Fit new polynomials to x,y in world space
        left_fit_cr = np.polyfit(ploty * ym_per_pix, leftx * xm_per_pix, 2)
        right_fit_cr = np.polyfit(ploty * ym_per_pix, rightx * xm_per_pix, 2)
        # Calculate the new radii of curvature
        left_curverad = ((1 + (2 * left_fit_cr[0] * y_eval * ym_per_pix + left_fit_cr[1]) ** 2) ** 1.5) / np.absolute(
            2 * left_fit_cr[0])
        right_curverad = ((1 + (2 * right_fit_cr[0] * y_eval * ym_per_pix + right_fit_cr[1]) ** 2) ** 1.5) / np.absolute(
            2 * right_fit_cr[0])
        # Now our radius of curvature is in meters
        # print(left_curverad, 'm', right_curverad, 'm')

        # Example values: 632.1 m    626.2 m
        return (left_curverad,  right_curverad)

    # def draw_on_orig(self, undist, warped, Minv):
    #     '''
    #
    #     :param undist:
    #     :param warped:
    #     :param Minv:
    #     :return: undist image with polyfill overlay.
    #     '''
    #     left_fit = self.left_fit
    #     right_fit = self.right_fit
    #     ploty = self.ploty
    #
    #     left_fitx = self.left_fitx
    #     right_fitx = self.right_fitx
    #
    #
    #     # Create an image to draw the lines on
    #     # warp_zero = np.zeros_like(warped).astype(np.uint8)
    #     # color_warp = np.dstack((warp_zero, warp_zero, warp_zero))
    #
    #     if (warped.shape[-1] == 3):
    #         color_warp = np.zeros_like(warped, np.uint8)
    #     else:
    #         color_warp = np.dstack((warped, warped, warped)) * 255
    #
    #     # Recast the x and y points into usable format for cv2.fillPoly()
    #     pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
    #     pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))])
    #     pts = np.hstack((pts_left, pts_right))
    #
    #     # Draw the lane onto the warped blank image
    #     cv2.fillPoly(color_warp, np.int_([pts]), (0, 255, 0))
    #
    #     # Warp the blank back to original image space using inverse perspective matrix (Minv)
    #     # newwarp = cv2.warpPerspective(color_warp, Minv, (image.shape[1], image.shape[0]))
    #     newwarp = cv2.warpPerspective(color_warp, Minv, (undist.shape[1], undist.shape[0]))
    #     # Combine the result with the original image
    #     result = cv2.addWeighted(undist, 1, newwarp, 0.3, 0)
    #     # plt.imshow(result)
    #     # plt.show()
    #     return result


    def draw_on_orig_new(self, undist, warped, Minv):
        '''

        :param undist:
        :param warped:
        :param Minv:
        :return: undist image with polyfill overlay.
        '''
        ploty = self.line_left.ploty

        left_fitx = self.line_left.lane_fitx
        right_fitx = self.line_right.lane_fitx


        # Create an image to draw the lines on
        # warp_zero = np.zeros_like(warped).astype(np.uint8)
        # color_warp = np.dstack((warp_zero, warp_zero, warp_zero))

        if (warped.shape[-1] == 3):
            color_warp = np.zeros_like(warped, np.uint8)
        else:
            color_warp = np.dstack((warped, warped, warped)) * 255

        # Recast the x and y points into usable format for cv2.fillPoly()
        pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
        pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))])
        pts = np.hstack((pts_left, pts_right))

        # Draw the lane onto the warped blank image
        cv2.fillPoly(color_warp, np.int_([pts]), (0, 255, 0))

        # Warp the blank back to original image space using inverse perspective matrix (Minv)
        # newwarp = cv2.warpPerspective(color_warp, Minv, (image.shape[1], image.shape[0]))
        newwarp = cv2.warpPerspective(color_warp, Minv, (undist.shape[1], undist.shape[0]))
        # Combine the result with the original image
        result = cv2.addWeighted(undist, 1, newwarp, 0.3, 0)
        # plt.imshow(result)
        # plt.show()
        return result

    # def pipeline(self, img):
    #     (imgSeries, Minv) = self.get_binary_warped_image(img)   # ([undist, bin, binWarped], Minv)
    #
    #     if (self.first_time):
    #         self.first_time = False
    #         outImg = self.poly_fit_first(imgSeries[-1])
    #         imgSeries[-1] = self.visualize_polyfit_first(imgSeries[-1], outImg)
    #     else:
    #         outImg = self.poly_fit(imgSeries[-1])  #blank binWarped
    #         imgSeries[-1] = self.visualize_polyfit(imgSeries[-1], outImg)  # binWarped with windows, lane lines
    #         (left_curverad, right_curverad) = self.measure_radius()
    #         img = self.draw_on_orig(imgSeries[0], imgSeries[2], Minv)  # undist image with polyfill overlay.
    #
    #         undist_bin = cv2.resize(imgSeries[1], (0, 0), fx=0.3, fy=0.3)  # bin
    #         warped_bin_debug = cv2.resize(imgSeries[-1], (0, 0), fx=0.3, fy=0.3)  # binWarped with windows, lane lines
    #         img[:250, :, :] = img[:250, :, :] * .4
    #         (h, w, _) = undist_bin.shape
    #         img[20:20 + h, 20:20 + w, :] = undist_bin
    #         img[20:20 + h, 20 + 20 + w:20 + 20 + w + w, :] = warped_bin_debug
    #         txt_x_loc = 20 + 20 + w + w + 20
    #         cv2.putText(img, 'Curvature: L {}m R {}m'.format(left_curverad, right_curverad),
    #                     (txt_x_loc, 80), cv2.FONT_HERSHEY_SIMPLEX, .8, (255, 255, 255), 2)
    #         # self.draw_text(img, 'Distance (left):       {:.1f} m'.format(10), text_x, 140)
    #         # self.draw_text(img, 'Distance (right):      {:.1f} m'.format(10), text_x, 200)
    #
    #
    #     return img
    #
    # def pipeline(self, img):
    #
    #     (imgSeries, Minv) = self.get_binary_warped_image(img)   # ([undist, bin, binWarped], Minv)
    #     outImg = self.poly_fit_first(imgSeries[-1])  #blank binWarped
    #     imgSeries[-1] = self.visualize_polyfit_first(imgSeries[-1], outImg)  # binWarped with windows, lane lines
    #     (left_curverad, right_curverad) = self.measure_radius()
    #     img = self.draw_on_orig(imgSeries[0], imgSeries[2], Minv)  # undist image with polyfill overlay.
    #
    #     undist_bin = cv2.resize(imgSeries[1], (0, 0), interpolation=cv2.INTER_NEAREST, fx=0.3, fy=0.3)  # bin
    #     warped_bin_debug = cv2.resize(imgSeries[-1], (0, 0), fx=0.3, fy=0.3)  # binWarped with windows, lane lines
    #     img[:250, :, :] = img[:250, :, :] * .4
    #     (h, w, _) = undist_bin.shape
    #     # img[20:20 + h, 20:20 + w, :] = undist_bin
    #     # img[20:20 + h, 20 + 20 + w:20 + 20 + w + w, :] = warped_bin_debug
    #     img[20:20 + h, 20:20 + w, :] = warped_bin_debug
    #
    #     txt_x_loc = 20 + 20 + w + w + 20
    #     cv2.putText(img, 'Curvature: L {}m, R {}m'.format(int(left_curverad), int(right_curverad)),
    #                     (txt_x_loc, 80), cv2.FONT_HERSHEY_SIMPLEX, .8, (255, 255, 255), 2)
    #     # self.draw_text(img, 'Distance (left):       {:.1f} m'.format(10), text_x, 140)
    #     # self.draw_text(img, 'Distance (right):      {:.1f} m'.format(10), text_x, 200)
    #
    #
    #     return img

    def pipeline(self, img):

        (imgSeries, Minv) = self.get_binary_warped_image(img)   # ([undist, bin, binWarped], Minv)
        outImg = self.poly_fit_line(imgSeries[-1])  #blank binWarped
        imgSeries[-1] = outImg  # binWarped with windows, lane lines
        left_curverad = self.line_left.measure_radius()
        right_curverad = self.line_right.measure_radius()

        img = self.draw_on_orig_new(imgSeries[0], imgSeries[2], Minv)  # undist image with polyfill overlay.

        warped_bin_debug = cv2.resize(imgSeries[-1], (0, 0), fx=0.3, fy=0.3)  # binWarped with windows, lane lines
        img[:250, :, :] = img[:250, :, :] * .4
        (h, w, _) = warped_bin_debug.shape

        img[20:20 + h, 20:20 + w, :] = warped_bin_debug

        txt_x_loc = 20 + 20 + w + w + 20
        cv2.putText(img, 'Curvature: L {}m, R {}m'.format(int(left_curverad), int(right_curverad)),
                        (txt_x_loc, 80), cv2.FONT_HERSHEY_SIMPLEX, .8, (255, 255, 255), 2)

        return img


