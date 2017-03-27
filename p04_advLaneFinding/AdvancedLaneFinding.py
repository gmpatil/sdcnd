import numpy as np
import cv2
import matplotlib.pyplot as plt
from Camera import Camera
from VideoProcessor import VideoProcessor
from moviepy.editor import VideoFileClip

TEST_IMG_DIR = "./test_images/"

def getOrigBinWarpedImages(imageFileNames, vidp):
    '''
    Takes list of image file names, creates image, binary image and warped image for each and returns
    all the images in array of array.

    :param imageFileNames:
    :param vidp:
    :return:
    '''
    images = []
    for imgFileName in imageFileNames:
        img = cv2.imread(TEST_IMG_DIR + imgFileName)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        imgSeries = vidp.get_binary_warped_image(img)
        images.append(imgSeries)

    return images


def getOrigBinWarpedImages2(imageFileNames, vidp):
    '''
    Takes list of image file names, creates image, binary image and warped image for each and returns
    all the images in array of array.

    :param imageFileNames:
    :param vidp:
    :return:
    '''

    first = True

    images = []
    for imgFileName in imageFileNames:
        img = cv2.imread(TEST_IMG_DIR + imgFileName)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        (imgSeries, Minv) = vidp.get_binary_warped_image(img)
        if (first):
            first = False
            outImg = vidp.poly_fit_first(imgSeries[-1])
            imgSeries[-1] = vidp.visualize_polyfit_first(imgSeries[-1], outImg)
            # vidp.measure_radius()
            # vidp.draw_on_orig(imgSeries[0], imgSeries[2], Minv)

            vidp.measure_radius()
            imgSeries[-1] = vidp.draw_on_orig(imgSeries[0], imgSeries[2], Minv)
        else:
            outImg = vidp.poly_fit(imgSeries[-1])
            imgSeries[-1] = vidp.visualize_polyfit(imgSeries[-1], outImg)
            vidp.measure_radius()
            imgSeries[-1] = vidp.draw_on_orig(imgSeries[0], imgSeries[2], Minv)

        images.append(imgSeries)

    return images



def displayOrigBinWarpedImages(images, histo=False):
    '''
    Display images in row x 3 images.

    :param images:
    :param histo: display histogram image
    :return:
    '''
    if (histo):
        f, axarr = plt.subplots(len(images), 2, figsize=(4 * 2, len(images) * 4))
    else:
        f, axarr = plt.subplots(len(images), 3, figsize=(4 * 3, len(images) * 4))

    cnt = 0
    for imgs in images:

        if (histo):
            img = imgs[2]
            histogram = np.sum(img[int(img.shape[0] / 2):, :], axis=0) # okay, may be more reliable during sharp curve
            # histogram = np.sum(img[int(img.shape[0]* 3 / 4):, :], axis=0)  # bad
            # histogram = np.sum(img[int(img.shape[0]/ 4):, :], axis=0) # good
            # histogram = np.sum(img[:, :], axis=0)
            axarr[cnt][0].imshow(imgs[2])
            axarr[cnt][0].set_title('Warped ' + str(cnt + 1))

            axarr[cnt][1].plot(histogram)
            axarr[cnt][1].set_title('Hist ' + str(cnt + 1))
        else:
            axarr[cnt][0].imshow(imgs[0])
            axarr[cnt][0].set_title('Img ' + str(cnt + 1))
            axarr[cnt][1].imshow(imgs[1])
            axarr[cnt][1].set_title('Bin ' + str(cnt + 1))
            axarr[cnt][2].imshow(imgs[2])
            axarr[cnt][2].set_title('Warped ' + str(cnt + 1))

        cnt += 1

    # plt.tight_layout()
    plt.show()

def find_lanes_in_video():
    cam = Camera(load=True)
    video = VideoProcessor(cam)

    p04_output = 'p04_out.mp4'
    clip1 = VideoFileClip("./project_video.mp4")
    p04_video_clip = clip1.fl_image(video.pipeline)  #this function expects color images!!
    p04_video_clip.write_videofile(p04_output, audio=False)


def main():

    find_lanes_in_video()

    # cam = Camera(load=False)
    # ret, mtx, dist, rvecs, tvecs = cam.calibrate_camera(save=True)
    # cam.undistort_calibration_images()
    # cam.undistort_test_images()
    #

    # cam = Camera(load=True)
    # video = VideoProcessor(cam)

    # load test images, undistort, convert binary and warp to bird view perspective.
    # images = getOrigBinWarpedImages(["straight_lines1.jpg",
    #                                 # "straight_lines2.jpg",
    #                                 # "test1.jpg",
    #                                  "test2.jpg",
    #                                 # "test3.jpg",
    #                                 # "test4.jpg",
    #                                 # "test5.jpg",
    #                                  "test6.jpg",
    #                                  ], video)
    # # displayOrigBinWarpedImages(images, histo=False)
    # displayOrigBinWarpedImages(images, histo=True)

    # images = getOrigBinWarpedImages2(["straight_lines1.jpg",
    #                                 # "straight_lines2.jpg",
    #                                 # "test1.jpg",
    #                                 # "test2.jpg",
    #                                 # "test3.jpg",
    #                                 # "test4.jpg",
    #                                 # "test5.jpg"
    #                                 # "test6.jpg"
    #                                   "mytest5.png",
    #                                   "mytest6.png"
    #                                  ], video)
    # displayOrigBinWarpedImages(images, histo=False)


if __name__ == '__main__':
    main()
