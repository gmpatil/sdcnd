import numpy as np
import cv2
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from VideoProcessor import VideoProcessor
from moviepy.editor import VideoFileClip
import glob
from skimage.feature import hog

TEST_IMG_DIR = "./test_images/"
TRAIN_IMG_DIR = "./train_images/"


def get_rand_img(files_path):
    images = glob.glob(files_path, recursive=True)
    num = len(images)
    randIndex = np.random.random_integers(0, num)
    img = mpimg.imread(images[randIndex])
    return  img

def display_sample_training_images():
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


def main():
    # Rubric #2
    # display_sample_training_images()
    display_training_image_features()

    # Rubric #3

if __name__ == '__main__':
    main()
