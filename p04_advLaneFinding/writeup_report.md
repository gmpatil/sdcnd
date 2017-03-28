## P4 - Advanced Lane Finding Writeup 

### Writeup for Advanced Lane Finding Project Report

---

**Advanced Lane Finding Project**

The goals / steps of this project are the following:

* Compute the camera calibration matrix and distortion coefficients given a set of chessboard images.
* Apply a distortion correction to raw images.
* Use color transforms, gradients, etc., to create a thresholded binary image.
* Apply a perspective transform to rectify binary image ("birds-eye view").
* Detect lane pixels and fit to find the lane boundary.
* Determine the curvature of the lane and vehicle position with respect to center.
* Warp the detected lane boundaries back onto the original image.
* Output visual display of the lane boundaries and numerical estimation of lane curvature and vehicle position.

[//]: # (Image References)

[image1]: ./output_images/undistorted_images.png "Undistorted"
[image2]: ./output_images/undistorted_test_images.png "Road Transformed"
[image3]: ./output_images/undistorted_binary_test1.png "Binary Example (test1,jpg)"
[image32]: ./output_images/undistorted_binary_test4.png "Binary Example (test4.jpg)"
[image33]: ./output_images/binary_test_images_1_4.png "Undistorted and Binary Images [1-4]"
[image34]: ./output_images/binary_test_images_1_4.png "Undistorted and Binary Images [5-8]"
[image4]: ./output_images/perspective_3x3.png "Warp Example"
[image41]: ./output_images/hist_3x2.png "Histogram"
[image42]: ./output_images/poly_line1.png "Ploynomial Line"
[image43]: ./output_images/poly_line3x3.png "Ploynomial Line For Test Images"
[image44]: ./output_images/polyFill_line3x3.png "Ploynomial Line Fill For Test Images"
[image5]: ./output_images/radius_vehicle_offset.jpg "Image showing radius of curvature and vehicle offset"
[image6]: ./output_images/advLaneOnOrig.png "Lane Lines Projected Back on Original Image"
[image61]: ./output_images/advLaneOnOrig2.png "Lane Lines Projected Back on Original Image"
[video1]: ./project_video.mp4 "Video"

## [Rubric](https://review.udacity.com/#!/rubrics/571/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  

This Markdown file is used to provide writeup.

Note all code related to this project are mainly in below Python files.
* AdvancedLaneFinding.py
* Camera.py

 
### Camera Calibration

#### 1. Briefly state how you computed the camera matrix and distortion coefficients. Provide an example of a distortion corrected calibration image.
The code for this step is contained in **`Camera.py`** file in methods **`calibrate_camera`** and **`undistort_calibration_images`**.

I start by preparing "object points", which will be the (x, y, z) coordinates of the chessboard corners in the 
world. Here I am assuming the chessboard is fixed on the (x, y) plane at z=0, such that the object points are the 
same for each calibration image.  Thus, `objp` is just a replicated array of coordinates, and `objpoints` will be
 appended with a copy of it every time I successfully detect all chessboard corners in a test image.  `imgpoints` 
 will be appended with the (x, y) pixel position of each of the corners in the image plane with each successful 
 chessboard detection.


I then used the output `objpoints` and `imgpoints` to compute the camera calibration and distortion coefficients using the `cv2.calibrateCamera()` function.  I applied this distortion correction to the test image using the `cv2.undistort()` function and obtained this result: 


I used `calibrate_camera` method to calibrate camera using the provided chessboard pattern images under `camera_cal` directory. 
Able to use 17 of 20 provided chessboard pattern image files to detect all the corners (9X6). Not able to detect all the corners in 3 of the images.
 
Steps involved are,
* Convert chessboard pattern images to Gray scale
* Identify the image points corresponding to interior corners using CV2 function findChessboardCorners.
* Once all the chessboard pattern images are processed, calculate camera calibration parameters like camera matrix, 
distribution co-efficient, rotation vectors and translation vectors using CV2 function calibrateCamera.

Once camera is calibrated, we can use calibration parameters to undistort the distorted images using CV2 function undistort.
See method ```undistort_calibration_images()```.

```python
        # Convert to gray scale image
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Find the chessboard corners
        ret, corners = cv2.findChessboardCorners(gray, (nx,ny), None)
        
        # Calibrate Camera
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, img_size, None, None)
        
        # Undistort distorted image
        udst1 = cv2.undistort(img1, mtx, dist, None, mtx)

```  

![alt text][image1]

### Pipeline (single images)

#### 1. Provide an example of a distortion-corrected image.
Using the above steps, below is sample of undistorted test images. See method `undistort_test_images()` in class `Camera` (Camera.py).
![alt text][image2]

#### 2. Describe how (and identify where in your code) you used color transforms, gradients or other methods to create a thresholded binary image.  Provide an example of a binary image result.
I used a combination of color and gradient thresholds to generate a binary image 
(thresholding steps at lines #39 through #77 in `VideoProcessor.py`).
```pythonstub
        # Convert to HLS color space and separate the L channel
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

        # Stack each channel
        color_binary = np.dstack((np.zeros_like(sxbinary), sxbinary, sh_binary))
        
```    

Here's an example of my output for this step. Below are samples of binary color image with green channel showing Sobel X gradients in the threshold and blue channel 
showing  

Binary color/RGB image of test1.png 
![alt text][image3]

Binary color/RGB image of test4.png  
![alt text][image32]

Below are the binary images for all the 8 test images along with undistorted images.
![alt text][image33]

![alt text][image34]

#### 3. Describe how (and identify where in your code) you performed a perspective transform and provide an example of a transformed image.

The code for my perspective transform includes a function called `warper()`, which appears in lines#77 to #100 
the file `VideoProcessor.py`.
The `warper()` function takes as inputs an image (`img`) and calculates based on image size `src` source and 
 `dst` destination points. I used the test image with straight/parallel lanes to manually measure source `src` points 
  and calculated the ration w.r.t image size so that points are not fixed and hard coded.
 
I chose caluclate the source and destination points in the following manner:

```
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

```
This resulted in the following source and destination points:

| Source           | Destination   | 
|:------------:    |:-------------:| 
| 0560, 477| 320, 0        | 
| 0295, 660| 320, 720      |
| 1011, 660| 960, 720      |
| 0727, 477| 960, 0        |

I verified that my perspective transform was working as expected by drawing the `src` and `dst` points onto a test image and its warped counterpart to verify that the lines appear parallel in the warped image.

![alt text][image4]

#### 4. Describe how (and identify where in your code) you identified lane-line pixels and fit their positions with a polynomial?

Then I did some other stuff and fit my lane lines with a 2nd order polynomial kinda like this:
* Using binary threshold image, perspective transform it to bird eye-view. See method `get_binary_warped_image`
in `VideoProcessor.py`.
* Calculate histogram of lane line pixels across Y-axis. Use x-position on on either side 
 of the mid point as potential lane line positions. See method `poly_fit_line` in `VideoProcessor.py`.
 
* Use Window tiling algorithm described in class to fit both left and right lane line.
 See `Line` class' `poly_fit_line` method.
 
 Below images show these steps used test images.
 
![alt text][image41]

![alt text][image42]

![alt text][image43]

![alt text][image44]


#### 5. Describe how (and identify where in your code) you calculated the radius of curvature of the lane and the position of the vehicle with respect to center.

I did this in lines #158 through #180 in my code in `Line.py` in the function `measure_radius()`.

Screen capture of the video showing radius of curvature and position of the vehicle with respect to the center.

![alt text][image5]

#### 6. Provide an example image of your result plotted back down onto the road such that the lane area is identified clearly.

I implemented this step in lines #515 through #553 in my code in `VideoProcessor.py` in the function `draw_on_orig_new()`.  
Here is an example of my result on a test image:

![alt text][image61]

![alt text][image6]

---

### Pipeline (video)

#### 1. Provide a link to your final video output.  Your pipeline should perform reasonably well on the entire project video (wobbly lines are ok but no catastrophic failures that would cause the car to drive off the road!).

Here's a [link to my video result](./p04_out.mp4)

---

### Discussion

#### 1. Briefly discuss any problems / issues you faced in your implementation of this project.  Where will your pipeline likely fail?  What could you do to make it more robust?

Here I'll talk about the approach I took, what techniques I used, what worked and why, where the pipeline might fail and how I might improve it if I were going to pursue this project further.

Using Window stacking/tiling algorithm for all the images worked well most of the video segment. But
 failed during segment of the road were contrast changed suddenly due to repair/patch work.
 Shade and other minor contrast introduced little wobblyness. 
 
 To smoothen the lane line identification throughout test video/road segment, I started keeping track of last 5 2nd degree 
 polynomial lane fit parameters and used the average of the latest five. 
 To further improve I filtered-out the "invalid lane line fit" if it deviated significantly from the average.
See method `valid_line` line #49 to #70 in `Line.py`.      

