import numpy as np
import cv2
import matplotlib.pyplot as plt
from Camera import Camera
from VideoProcessor import VideoProcessor


CALIBRATION_IMG_DIR = "./camera_cal/"
TEST_IMG_DIR = "./test_images/"





def main():
    cam = Camera(load=True)
    # ret, mtx, dist, rvecs, tvecs = cam.calibrate_camera(save=True)
    # cam.undistort_calibration_images()
    # cam.undistort_test_images()
    #
    img = cv2.imread(TEST_IMG_DIR + "test4.jpg")
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img = cam.undistort(img)

    video = VideoProcessor(cam)
    bin = video.thresholded_binary(img)

    plt.imshow(bin)
    plt.show()


    # # Test code to identify points for perspective transformation
    # img = cv2.imread(TEST_IMG_DIR + "straight_lines1.jpg")
    # img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    # img = cam.undistort(img)
    # #
    # video = VideoProcessor(cam)
    # bin = video.thresholded_binary(img)
    #
    # plt.imshow(bin)
    # plt.show()


    img_size = (img.shape[1], img.shape[0])

    print (img_size)

    print (bin.shape)


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

    bin = video.warper(bin, src, dst)
    plt.imshow(bin)
    plt.show()




if __name__ == '__main__':
    main()
