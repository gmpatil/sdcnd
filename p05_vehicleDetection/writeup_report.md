## Writeup Report for Vehicle Detection Project

---

**Vehicle Detection Project**

The goals / steps of this project are the following:

* Perform a Histogram of Oriented Gradients (HOG) feature extraction on a labeled training set of images and train a classifier Linear SVM classifier
* Optionally, you can also apply a color transform and append binned color features, as well as histograms of color, to your HOG feature vector. 
* Note: for those first two steps don't forget to normalize your features and randomize a selection for training and testing.
* Implement a sliding-window technique and use your trained classifier to search for vehicles in images.
* Run your pipeline on a video stream (start with the test_video.mp4 and later implement on full project_video.mp4) and create a heat map of recurring detections frame by frame to reject outliers and follow detected vehicles.
* Estimate a bounding box for vehicles detected.

[//]: # (Image References)
[image1]: ./output_images/r1_sample_images.png
[image12]: ./output_images/r1_v_features.png
[image13]: ./output_images/r1_nv_features.png
[image3]: ./output_images/scaling.png
[image31]: ./output_images/sliding_windows_red.png
[image32]: ./output_images/sliding_windows_green.png
[image33]: ./output_images/sliding_windows_blue.png
[image41]: ./output_images/p1_1.png
[image42]: ./output_images/p1_2.png
[image43]: ./output_images/p1_4.png
[image44]: ./output_images/p1_5.png
[image45]: ./output_images/p1_6.png
[image45]: ./output_images/p1_6.png
[image51]: ./output_images/p2_1.png
[image52]: ./output_images/p2_2.png
[image52b]: ./output_images/p2_2b.png
[image53]: ./output_images/p2_4.png
[image54]: ./output_images/p2_5.png
[image55]: ./output_images/p2_6.png
[image61]: ./output_images/img39_001.jpg
[image62]: ./output_images/img39_002.jpg
[image63]: ./output_images/img39_003.jpg
[image64]: ./output_images/img39_004.jpg
[image65]: ./output_images/img39_005.jpg
[image66]: ./output_images/img39_006.jpg
[video1]: ./p05_out_full.mp4

## [Rubric](https://review.udacity.com/#!/rubrics/513/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  
You can submit your writeup as markdown or pdf.  
Used file from [here](https://github.com/udacity/CarND-Vehicle-Detection/blob/master/writeup_template.md) as a template and starting point writeup 
for this project.  


### Histogram of Oriented Gradients (HOG)

#### 1. Explain how (and identify where in your code) you extracted HOG features from the training images.

The code for this step is contained in the method `get_hog_features` of class `VideoProcessor.py`.


I started by reading in all the `vehicle` and `non-vehicle` images.  Here are examples of  
`vehicle` and `non-vehicle` class images. 
(`Method display_sample_training_images` in class `VehicleDetection.py`):

![alt text][image1]

I then explored different color spaces and different `skimage.hog()` parameters (`orientations`, 
`pixels_per_cell`, and `cells_per_block`).  I grabbed random images from each of the two classes and 
displayed them to get a feel for what the `skimage.hog()` output looks like.

Here is an example using the `YCrCb` color space and HOG parameters of `orientations=9`, `pixels_per_cell=(8, 8)` 
and `cells_per_block=(2, 2)`:


![alt text][image12]

![alt text][image13]

#### 2. Explain how you settled on your final choice of HOG parameters.

I tried various combinations of parameters. Mainly tried different color spaces to get HOG orientations, and found YCrCb 
and LUV faired better on test images as compared to HLS, HSV, YUV and RGB color spaces. And YCrCb slightly better than LUV 
with better or more true positive and less false positive detections. Adding all the HOG channels, gave better result 
than just using individual channels. Method `train()` in  `VehicleDetection.py` .

#### 3. Describe how (and identify where in your code) you trained a classifier using your selected HOG features (and color features if you used them).

I started training linear SVM initially using 100 each images  each of vehicle and non-vehicle classes. Gradually as some
 of the parameters started showing promising results, increases the training size and finally training and tested using all 
the vehicle and non-vehicle classes images provided. See method `train()` in  `VehicleDetection.py` for details.

    # Number of vehicle samples: 8792
    # Number of non-vehicle samples 8968
    # Using: 9 orientations 8 pixels per cell and 2 cells per block
    # Feature vector length: 6108
    # 13.21 Seconds to train SVC
    # Test Accuracy of SVC =  0.993

```python

    car_features = extract_features(cars, color_space=color_space,
                                    spatial_size=spatial_size, hist_bins=hist_bins,
                                    orient=orient, pix_per_cell=pix_per_cell,
                                    cell_per_block=cell_per_block,
                                    hog_channel=hog_channel, spatial_feat=spatial_feat,
                                    hist_feat=hist_feat, hog_feat=hog_feat)
    notcar_features = extract_features(notcars, color_space=color_space,
                                       spatial_size=spatial_size, hist_bins=hist_bins,
                                       orient=orient, pix_per_cell=pix_per_cell,
                                       cell_per_block=cell_per_block,
                                       hog_channel=hog_channel, spatial_feat=spatial_feat,
                                       hist_feat=hist_feat, hog_feat=hog_feat)

    X = np.vstack((car_features, notcar_features)).astype(np.float64)
    X_scaler = StandardScaler().fit(X)
    scaled_X = X_scaler.transform(X)

    # Define the labels vector
    y = np.hstack((np.ones(len(car_features)), np.zeros(len(notcar_features))))

    # Split up data into randomized training and test sets
    rand_state = np.random.randint(0, 100)
    X_train, X_test, y_train, y_test = train_test_split(scaled_X, y, test_size=0.2, random_state=rand_state)
    
    # Use a linear SVC
    svc = LinearSVC()
    svc.fit(X_train, y_train)
```

Parameters values finally used to train Linear SVC classifier are as below.  

``` 
    color_space = 'YCrCb'  #  YCrCb =~ LUV > HLS > HSV > > YUV > RGB
    orient = 9  # HOG orientations
    pix_per_cell = 8  # HOG pixels per cell
    cell_per_block = 2  # HOG cells per block
    hog_channel = "ALL"  # 
    spatial_size = (16, 16)  # Spatial binning dimensions
    hist_bins = 16  # Number of histogram bins
    spatial_feat = True  # Spatial features on or off
    hist_feat = True  # Histogram features on or off
    hog_feat = True  # HOG features on or off
```    
    

### Sliding Window Search

#### 1. Describe how (and identify where in your code) you implemented a sliding window search.  How did you decide what scales to search and how much to overlap windows?
Started with scaling factor of 1.5 (64 * 1.5 = 96x96px) as shown by Rayn Keenan in one of the session, which gave good 
performance compared to other factor like 1.0 and 0.75 alone. But gradually added other factor Wondow sliding to improve/increase
 true positives. User 8 pixel per cell and 2 cells per step/stride.  

In order to improve performance and avoid false positive did sliding window scanning only from bottom of the screen to 
around horizon. Scanning is done across X co-ordinates and for Y co-ordinates from 400px to 656px. 
Scanning is further restricted to band near horizon as below figure shows for smaller window sizes as they appear near horizon. 

![alt text][image3]

Sliding Window of scale factor of 1.5 (96x96 px)
![alt text][image31]

Sliding Window of scale factor of 1 (64x64 px)
![alt text][image32]

Sliding Window of scale factor of 0.75 (48x48 px)
![alt text][image33]


See method `find_cars_at_scale()` in class `VideoProcessor.py` sliding window search implementation logic.
See method `find_cars_with_heatmap()` in class `VideoProcessor.py` different scaling used search vehicles using 
`find_cars_at_scale()` method.

#### 2. Show some examples of test images to demonstrate how your pipeline is working.  What did you do to optimize the performance of your classifier?

I searched on two scales using YCrCb 3-channel HOG features plus spatially binned color and histograms of color 
in the feature vector, which provided a nice result.  Here are some example images:

##### Pipeline Images Without Heatmap and Labels
Test Image-1
![alt text][image41]

Test Image-2
![alt text][image42]

Test Image-4
![alt text][image43]
Test Image-5
![alt text][image44]
Test Image-6
![alt text][image45]

##### Pipeline Images Using Heatmap and Labels
Test Image-1
![alt text][image51]
Test Image-2 with False Positive on Left Side
![alt text][image52]
Test Image-2 with False Positive Filtered-out Using Hieght Width Ratio
![alt text][image52b]
Test Image-4 
![alt text][image53]
Test Image-5
![alt text][image54]
Test Image-6
![alt text][image55]

---

### Video Implementation

#### 1. Provide a link to your final video output.  Your pipeline should perform reasonably well on the entire project video (somewhat wobbly or unstable bounding boxes are ok as long as you are identifying the vehicles most of the time with minimal false positives.)
Here's a [link to my video result](./p05_out_full.mp4) 


#### 2. Describe how (and identify where in your code) you implemented some kind of filter for false positives and some method for combining overlapping bounding boxes.

I recorded the positions of positive detections in each frame of the video and also kept count of running heat count
from previous frames.  From the positive detections I created a heatmap and then thresholded that map to identify vehicle positions.  
I then used `scipy.ndimage.measurements.label()` to identify individual blobs in the heatmap.  
I then assumed each blob corresponded to a vehicle.  I constructed bounding boxes to cover the area of each blob detected.  

See method `find_cars_with_heatmap` in class `VideoProcessor.py`.
```python

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

        # draw_img = self.draw_labeled_bboxes(np.copy(draw_img), labels)
        draw_img = self.draw_labeled_bboxes(draw_img, labels)

```
Here's an example result showing the heatmap from a series of frames of video, the 
result of `scipy.ndimage.measurements.label()` and the bounding boxes then overlaid on the last frame of video:

### Here are six frames and their corresponding heatmaps:
These 6 frames are extracted from 39th second onwards from the video results.
These frames show on left overlay sliding windows classified as vehicles both true and false positives.
The second overlay in each frame shows heatmap and main frame image shows bounding box overlay calculated using 
the output of `scipy.ndimage.measurements.label()` on the integrated heatmap

![alt text][image61]

![alt text][image62]

![alt text][image63]

![alt text][image64]

![alt text][image65]

![alt text][image66]


---

### Discussion

#### 1. Briefly discuss any problems / issues you faced in your implementation of this project.  Where will your pipeline likely fail?  What could you do to make it more robust?

Here I'll talk about the approach I took, what techniques I used, what worked and why, where the pipeline 
might fail and how I might improve it if I were going to pursue this project further.

Main challenges in vehicle detection was increasing the positive identification count of near and far vehicles and at the same
time filtering out many false postives. These challenges are over come by tuning or selecting appropriate features and SVC 
parameters.  Next challenge was to improve the performance of pipeline processing. Selective scanning technique performance was 
 improved from processing 1 frame in 1.5 seconds or 30 min to process video of 1260 frames to 0.75 seconds per frame or 16 min for 
 whole video. Though performance is improved, still further improvement is needed for real-time use. 
 Further improvements are identifying vehicles which are further away and robustly identify vehicles near by. 
 Identifying nearby vehicles should not be sentive to heatmap threshold and density of the traffic.
     
     
 
 
 


