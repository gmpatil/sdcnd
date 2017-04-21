import numpy as np
import cv2
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from VideoProcessor import VideoProcessor
from moviepy.editor import VideoFileClip
import glob
import time
import pickle

from skimage.feature import hog
from sklearn.svm import LinearSVC
from sklearn.preprocessing import StandardScaler
from sklearn.model_selection import train_test_split

import cProfile


TEST_IMG_DIR = "./test_images/"
TRAIN_IMG_DIR = "./train_images/"


def get_rand_img(files_path):
    images = glob.glob(files_path, recursive=True)
    num = len(images)
    randIndex = np.random.random_integers(0, num)
    img = mpimg.imread(images[randIndex])
    return  img

def display_training_images():
    # non-vehicle: ${TRAIN_IMG_DIR}/non-vehicles/Extras/extra1.png to extra5766.png,
    #              ${TRAIN_IMG_DIR}/non-vehicles/GTI/image1.png to image3900.png
    # vehicle: ${TRAIN_IMG_DIR}/vehicles/GTI_Far/image0000.png to image0974.png ,
    #          ${TRAIN_IMG_DIR}/vehicles/GTI_Left/image0009.png to image0974.png,
    #          ${TRAIN_IMG_DIR}/vehicles/GTI_MiddleClose/image0000.png to image0494.png,
    #          ${TRAIN_IMG_DIR}/vehicles/GTI_Right/image0000.png to image0974.png,
    #          ${TRAIN_IMG_DIR}/vehicles/KITTI_extracted/1.png to 5969.png
    img_nv = []
    img_nv.append(get_rand_img(str(TRAIN_IMG_DIR + "/non-vehicles/GTI/image*.png") ) )
    img_nv.append(get_rand_img(str(TRAIN_IMG_DIR + "/non-vehicles/GTI/image*.png") ) )
    img_nv.append(get_rand_img(str(TRAIN_IMG_DIR + "/non-vehicles/GTI/image*.png") ) )
    img_nv.append(get_rand_img(str(TRAIN_IMG_DIR + "/non-vehicles/GTI/image*.png") ) )

    img_v = []
    img_v.append(get_rand_img(str(TRAIN_IMG_DIR + "/vehicles/GTI_Far/image*.png") ) )
    img_v.append(get_rand_img(str(TRAIN_IMG_DIR + "/vehicles/GTI_Left/image*.png") ) )
    img_v.append(get_rand_img(str(TRAIN_IMG_DIR + "/vehicles/GTI_MiddleClose/image*.png") ) )
    img_v.append(get_rand_img(str(TRAIN_IMG_DIR + "/vehicles/GTI_Right/image*.png") ) )


    fig, (axs) = plt.subplots(2, 4, figsize=(4 * 4, 4 * 2))
    axs[0][0].imshow(img_nv[0])
    axs[0][1].imshow(img_nv[1])
    axs[0][2].imshow(img_nv[2])
    axs[0][3].imshow(img_nv[3])

    axs[0][0].set_title("Non-vehicle 1")
    axs[0][1].set_title("Non-vehicle 2")
    axs[0][2].set_title("Non-vehicle 3")
    axs[0][3].set_title("Non-vehicle 4")

    axs[1][0].imshow(img_v[0])
    axs[1][1].imshow(img_v[1])
    axs[1][2].imshow(img_v[2])
    axs[1][3].imshow(img_v[3])

    axs[1][0].set_title("Vehicle 1 Far")
    axs[1][1].set_title("Vehicle 2 Left")
    axs[1][2].set_title("Vehicle 3 Middle")
    axs[1][3].set_title("Vehicle 4 Right")

    plt.show()

def display_training_image_features():
    vp = VideoProcessor()

    # vehicle
    img = get_rand_img(str(TRAIN_IMG_DIR + "/vehicles/GTI_Far/image*.png"))
    img_cs = vp.convert_color(img)
    ch1 = img_cs[:, :, 0]
    ch2 = img_cs[:, :, 1]
    ch3 = img_cs[:, :, 2]
    # hist1 = np.histogram(ch1, bins=32)
    # hist2 = np.histogram(ch2, bins=32)
    # hist3 = np.histogram(ch3, bins=32)

    features, hog1 = hog(ch1, orientations=9,
                              pixels_per_cell=(8, 8),
                              cells_per_block=(2, 2),
                              transform_sqrt=False,
                              visualise=True, feature_vector=True)

    features, hog2 = hog(ch2, orientations=9,
                              pixels_per_cell=(8, 8),
                              cells_per_block=(2, 2),
                              transform_sqrt=False,
                              visualise=True, feature_vector=True)

    features, hog3 = hog(ch3, orientations=9,
                              pixels_per_cell=(8, 8),
                              cells_per_block=(2, 2),
                              transform_sqrt=False,
                              visualise=True, feature_vector=True)

    fig, (axs) = plt.subplots(3, 4, figsize=(4 * 4, 4 * 3))
    axs[0][0].imshow(img)
    axs[0][1].imshow(ch1)
    axs[0][2].imshow(hog1)
    data = np.hstack(ch1)
    binwidth =  (max(data) - min(data)) / 32
    axs[0][3].hist(data, np.arange(min(data), max(data) + binwidth, binwidth) )

    axs[1][0].imshow(img)
    axs[1][1].imshow(ch2)
    axs[1][2].imshow(hog2)
    data = np.hstack(ch2)
    binwidth = (max(data) - min(data)) / 32
    axs[1][3].hist(data, np.arange(min(data), max(data) + binwidth, binwidth) )

    axs[2][0].imshow(img)
    axs[2][1].imshow(ch3)
    axs[2][2].imshow(hog3)
    data = np.hstack(ch3)
    binwidth = (max(data) - min(data)) / 32
    axs[2][3].hist(data, np.arange(min(data), max(data) + binwidth, binwidth) )

    axs[0][0].set_title("Vehicle Image")
    axs[1][0].set_title("Vehicle Image")
    axs[2][0].set_title("Vehicle Image")

    axs[0][1].set_title("Ch 1")
    axs[1][1].set_title("Ch 2")
    axs[2][1].set_title("Ch 3")

    axs[0][2].set_title("Ch 1 HOG")
    axs[1][2].set_title("Ch 2 HOG")
    axs[2][2].set_title("Ch 3 HOG")

    axs[0][3].set_title("Ch 1 Histogram")
    axs[1][3].set_title("Ch 2 Histogram")
    axs[2][3].set_title("Ch 3 Histogram")

    plt.show()

    # non-vehicle
    img = get_rand_img(str(TRAIN_IMG_DIR + "/non-vehicles/GTI/image*.png"))
    img_cs = vp.convert_color(img)
    ch1 = img_cs[:, :, 0]
    ch2 = img_cs[:, :, 1]
    ch3 = img_cs[:, :, 2]

    features, hog1 = hog(ch1, orientations=9,
                              pixels_per_cell=(8, 8),
                              cells_per_block=(2, 2),
                              transform_sqrt=False,
                              visualise=True, feature_vector=True)

    features, hog2 = hog(ch2, orientations=9,
                              pixels_per_cell=(8, 8),
                              cells_per_block=(2, 2),
                              transform_sqrt=False,
                              visualise=True, feature_vector=True)

    features, hog3 = hog(ch3, orientations=9,
                              pixels_per_cell=(8, 8),
                              cells_per_block=(2, 2),
                              transform_sqrt=False,
                              visualise=True, feature_vector=True)

    fig, (axs) = plt.subplots(3, 4, figsize=(4 * 4, 4 * 3))
    axs[0][0].imshow(img)
    axs[0][1].imshow(ch1)
    axs[0][2].imshow(hog1)
    data = np.hstack(ch1)
    binwidth =  (max(data) - min(data)) / 32
    axs[0][3].hist(data, np.arange(min(data), max(data) + binwidth, binwidth) )

    axs[1][0].imshow(img)
    axs[1][1].imshow(ch2)
    axs[1][2].imshow(hog2)
    data = np.hstack(ch2)
    binwidth = (max(data) - min(data)) / 32
    axs[1][3].hist(data, np.arange(min(data), max(data) + binwidth, binwidth) )

    axs[2][0].imshow(img)
    axs[2][1].imshow(ch3)
    axs[2][2].imshow(hog3)
    data = np.hstack(ch3)
    binwidth = (max(data) - min(data)) / 32
    axs[2][3].hist(data, np.arange(min(data), max(data) + binwidth, binwidth) )

    axs[0][0].set_title("Non-vehicle Image")
    axs[1][0].set_title("Non-vehicle Image")
    axs[2][0].set_title("Non-vehicle Image")

    axs[0][1].set_title("Ch 1")
    axs[1][1].set_title("Ch 2")
    axs[2][1].set_title("Ch 3")

    axs[0][2].set_title("Ch 1 HOG")
    axs[1][2].set_title("Ch 2 HOG")
    axs[2][2].set_title("Ch 3 HOG")

    axs[0][3].set_title("Ch 1 Histogram")
    axs[1][3].set_title("Ch 2 Histogram")
    axs[2][3].set_title("Ch 3 Histogram")

    plt.show()


# Define a function to return HOG features and visualization
def get_hog_features(img, orient, pix_per_cell, cell_per_block,
                        vis=False, feature_vec=True):
    # Call with two outputs if vis==True
    if vis == True:
        features, hog_image = hog(img, orientations=orient, pixels_per_cell=(pix_per_cell, pix_per_cell),
                                  cells_per_block=(cell_per_block, cell_per_block), transform_sqrt=True,
                                  visualise=vis, feature_vector=feature_vec)
        return features, hog_image
    # Otherwise call with one output
    else:
        features = hog(img, orientations=orient, pixels_per_cell=(pix_per_cell, pix_per_cell),
                       cells_per_block=(cell_per_block, cell_per_block), transform_sqrt=True,
                       visualise=vis, feature_vector=feature_vec)
        return features

# Define a function to extract features from a list of images
# Have this function call bin_spatial() and color_hist()
def extract_features(imgs, color_space='RGB', spatial_size=(32, 32),
                     hist_bins=32, orient=9,
                     pix_per_cell=8, cell_per_block=2, hog_channel=0,
                     spatial_feat=True, hist_feat=True, hog_feat=True):

    vp = VideoProcessor()

    # Create a list to append feature vectors to
    features = []
    # Iterate through the list of images
    for file in imgs:
        file_features = []
        # Read in each one by one
        image = mpimg.imread(file)
        # apply color conversion if other than 'RGB'
        if color_space != 'RGB':
            if color_space == 'HSV':
                feature_image = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
            elif color_space == 'LUV':
                feature_image = cv2.cvtColor(image, cv2.COLOR_RGB2LUV)
            elif color_space == 'HLS':
                feature_image = cv2.cvtColor(image, cv2.COLOR_RGB2HLS)
            elif color_space == 'YUV':
                feature_image = cv2.cvtColor(image, cv2.COLOR_RGB2YUV)
            elif color_space == 'YCrCb':
                feature_image = cv2.cvtColor(image, cv2.COLOR_RGB2YCrCb)
        else:
            feature_image = np.copy(image)

        if spatial_feat == True:
            spatial_features = vp.bin_spatial(feature_image, size=spatial_size)
            file_features.append(spatial_features)
        if hist_feat == True:
            # Apply color_hist()
            hist_features = vp.color_hist(feature_image, nbins=hist_bins)
            file_features.append(hist_features)
        if hog_feat == True:
            # Call get_hog_features() with vis=False, feature_vec=True
            if hog_channel == 'ALL':
                hog_features = []
                for channel in range(feature_image.shape[2]):
                    hog_features.append(get_hog_features(feature_image[:, :, channel],
                                                         orient, pix_per_cell, cell_per_block,
                                                         vis=False, feature_vec=True))
                hog_features = np.ravel(hog_features)
            else:
                hog_features = get_hog_features(feature_image[:, :, hog_channel], orient,
                                                pix_per_cell, cell_per_block, vis=False, feature_vec=True)
            # Append the new feature vector to the features list
            file_features.append(hog_features)
        features.append(np.concatenate(file_features))
    # Return list of feature vectors
    return features

def train():
    # images = glob.glob(TRAIN_IMG_DIR + 'vehicles/**/GTI*/*.png', recursive=True)
    images = glob.glob(TRAIN_IMG_DIR + 'vehicles/**/*.png', recursive=True)
    cars = []
    notcars = []
    for image in images:
        cars.append(image)

    # images = glob.glob(TRAIN_IMG_DIR + 'non-vehicles/**/GTI*/*.png', recursive=True)
    images = glob.glob(TRAIN_IMG_DIR + 'non-vehicles/**/*.png', recursive=True)
    for image in images:
        notcars.append(image)


    print("Number of vehicle samples: {}\nNumber of non-vehicle samples {}".format(len(cars), len(notcars)))
    # sample_size = 500
    # cars = cars[0:sample_size]
    # notcars = notcars[0:sample_size]


    color_space = 'YCrCb'  # ''RGB'  # Can be RGB, HSV, LUV, HLS, YUV, YCrCb  ( YCrCb = LUV? > HLS > HSV > > YUV > RGB)
    orient = 9  # HOG orientations
    pix_per_cell = 8  # HOG pixels per cell
    cell_per_block = 2  # HOG cells per block
    hog_channel = "ALL"  # 2  # Can be 0, 1, 2, or "ALL" ( "ALL" > 0 > 1> 2)
    spatial_size = (16, 16)  # Spatial binning dimensions
    hist_bins = 16  # Number of histogram bins
    spatial_feat = True  # Spatial features on or off
    hist_feat = True  # Histogram features on or off
    hog_feat = True  # HOG features on or off

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
    # Fit a per-column scaler
    X_scaler = StandardScaler().fit(X)
    # Apply the scaler to X
    scaled_X = X_scaler.transform(X)

    # Define the labels vector
    y = np.hstack((np.ones(len(car_features)), np.zeros(len(notcar_features))))

    # Split up data into randomized training and test sets
    rand_state = np.random.randint(0, 100)
    X_train, X_test, y_train, y_test = train_test_split(
        scaled_X, y, test_size=0.2, random_state=rand_state)

    print('Using:', orient, 'orientations', pix_per_cell,
          'pixels per cell and', cell_per_block, 'cells per block')
    print('Feature vector length:', len(X_train[0]))
    # Use a linear SVC
    svc = LinearSVC()
    # Check the training time for the SVC
    t = time.time()
    svc.fit(X_train, y_train)
    t2 = time.time()
    print(round(t2 - t, 2), 'Seconds to train SVC...')

    # Check the score of the SVC
    print('Test Accuracy of SVC = ', round(svc.score(X_test, y_test), 4))


    # Save the training parameters.
    dist_pickle = {'svc': svc, 'scaler':X_scaler, "orient":orient,  "pix_per_cell":pix_per_cell ,
                   "cell_per_block": cell_per_block, "spatial_size":spatial_size, "hist_bins": hist_bins}

    pickle.dump(dist_pickle, open("svc_pickle.p", "wb"))

def predict():
    vp = VideoProcessor()
    # y_start_stop = [400, 670]  # Min and max in y to search in slide_window()
    y_start_stop = [400, 656]  # Min and max in y to search in slide_window()

    test_images = glob.glob(TEST_IMG_DIR + '**/*.jpg', recursive=True)

    for t_image in test_images:
        image = mpimg.imread(t_image)
        t = time.time()
        draw_image = np.copy(image)

        # Uncomment the following line if you extracted training
        # data from .png images (scaled 0 to 1 by mpimg) and the
        # image you are searching is a .jpg (scaled 0 to 255)
        image = image.astype(np.float32)/255

        windows = vp.slide_window(image, x_start_stop=[None, None], y_start_stop=y_start_stop,
                               xy_window=(96, 96), xy_overlap=(0.5, 0.5))

        hot_windows = vp.search_windows(image, windows)

        print ("### # of Hot windows {}".format(len(hot_windows)))

        window_img = vp.draw_boxes(draw_image, hot_windows, color=(0, 0, 255), thick=6)

        t2 = time.time()
        print(round(t2 - t, 2), 'Seconds to predict a image SVC...')
        plt.imshow(window_img)

        plt.show()

def predict2():
    # find_cars_with_heatmap
    # vp = VideoProcessor()
    # y_start_stop = [400, 656]  # Min and max in y to search in slide_window()

    test_images = glob.glob(TEST_IMG_DIR + '**/test1.jpg', recursive=True)
    # test_images = glob.glob("gtest/imagefile001.jpg", recursive=True)

    for t_image in test_images:
        vp = VideoProcessor()
        image = mpimg.imread(t_image)
        vp.find_cars_with_heatmap(image, jpg_image=True)

def sliding_windows():
    test_images = glob.glob(TEST_IMG_DIR + '**/test1.jpg', recursive=True)
    # test_images = glob.glob("gtest/imagefile001.jpg", recursive=True)

    for t_image in test_images:
        vp = VideoProcessor()
        image = mpimg.imread(t_image)
        vp.draw_windows_all_scales(image, jpg_image=True)

def find_vehicles_in_video():
    video = VideoProcessor()

    p05_output = 'p05_out.mp4'
    clip1 = VideoFileClip("../p04_advLaneFinding/project_video.mp4")
    # clip1 = VideoFileClip("./project_video.mp4")
    # clip1 = VideoFileClip("./project_video_clp1.mp4")
    p05_video_clip = clip1.fl_image(video.pipeline)
    p05_video_clip.write_videofile(p05_output, audio=False)

    # 235/236 [01:12<00:00,  3.17it/s]

# [MoviePy] >>>> Building video p05_out.mp4
# [MoviePy] Writing video p05_out.mp4
# 100%|█████████▉| 1260/1261 [15:42<00:00,  1.34it/s]
# [MoviePy] Done.
# [MoviePy] >>>> Video ready: p05_out.mp4

def extract_clip_pipeline(img):
    return img

def extract_clip():
    video = VideoProcessor()

    clip1_out = 'project_video_clp3.mp4'
    # clip1 = VideoFileClip("../p04_advLaneFinding/project_video.mp4")
    # clip1 = clip1.cutout(0, 25)
    clip1 = VideoFileClip("project_video_clp1.mp4")
    clip1 = clip1.cutout(10, 26)
    video_clip = clip1.fl_image(extract_clip_pipeline)
    video_clip.write_videofile(clip1_out, audio=False)

def main():
    # extract_clip()
    # Rubric #2
    # display_training_images()
    # display_training_image_features()

    # Rubric #3
    # Number of vehicle samples: 8792
    # Number of non-vehicle samples 8968
    # Using: 9 orientations 8 pixels per cell and 2 cells per block
    # Feature vector length: 6108
    # 13.21 Seconds to train SVC...
    # Test Accuracy of SVC =  0.993
    # train()

    # Rubric 4.1, 4.2
    # predict()
    # cProfile.run('predict2()')
    # predict2()
    sliding_windows()

    # Rubric 5
    # find_vehicles_in_video()

if __name__ == '__main__':
    main()
