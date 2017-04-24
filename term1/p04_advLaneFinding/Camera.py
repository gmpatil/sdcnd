import numpy as np
import cv2
import glob
import matplotlib.pyplot as plt
import pickle

CALIBRATION_IMG_DIR = "./camera_cal/"
TEST_IMG_DIR = "./test_images/"


class Camera(object):
    '''
    Camera class to calibrate the camera, save the parameter in Pickle. Can load during initialization once
    and re-use to repeated undistortion of images.

    '''

    def __init__(self, load=True):
        '''

        :param load: True to load from the Pickle file.
        '''
        if (load):
            dist_pickle = pickle.load(open(CALIBRATION_IMG_DIR + "camera_mtx_dist_pickle.p", "rb"))
            self.mtx = dist_pickle["mtx"]
            self.dist = dist_pickle["dist"]
        else:
            self.mtx = None
            self.dist = None


    def calibrate_camera(self, save=True):
        '''
        Do Camera calibration using images under ./camera_cal/
        Code is forked from https://github.com/udacity/CarND-Camera-Calibration/blob/master/camera_calibration.ipynb

        :return: ret, mtx, dist, rvecs, tvecs
        '''

        nx = 9
        ny = 6

        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        objp = np.zeros((ny * nx, 3), np.float32)
        objp[:, :2] = np.mgrid[0:nx, 0:ny].T.reshape(-1, 2)

        # Arrays to store object points and image points from all the images.
        objpoints = []  # 3d points in real world space
        imgpoints = []  # 2d points in image plane.

        # Make a list of calibration images
        images = glob.glob(CALIBRATION_IMG_DIR + "calibration*.jpg")

        num_files = 0
        num_files_not_used = 0

        img = None

        # Step through the list and search for chessboard corners
        for idx, fname in enumerate(images):
            print("Calibrating using file: {}".format(fname))
            num_files += 1

            img = cv2.imread(fname)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            # Find the chessboard corners
            ret, corners = cv2.findChessboardCorners(gray, (nx, ny), None)

            # print ("Corners: \n {}".format(corners))

            # If found, add object points, image points
            if ret == True:
                objpoints.append(objp)
                imgpoints.append(corners)

                # Draw and display the corners
                cv2.drawChessboardCorners(img, (nx, ny), corners, ret)
                # cv2.imwrite(fname.replace("calibration", "upd_calibration"), img)
                # cv2.imshow(fname, img)
                # cv2.waitKey(500)
            else:
                print("Corners not found in file {}".format(fname))
                num_files_not_used += 1

        # cv2.destroyAllWindows()

        print("Number of images read = {}. Num of images in which all corners not found = {}".format(num_files,
                                                                                                     num_files_not_used))

        img_size = (img.shape[1], img.shape[0])
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, img_size, None, None)

        # Save the camera calibration result for later use (we won't worry about rvecs / tvecs)
        if (save):
            dist_pickle = {}
            dist_pickle["mtx"] = mtx
            dist_pickle["dist"] = dist
            pickle.dump(dist_pickle, open(CALIBRATION_IMG_DIR + "camera_mtx_dist_pickle.p", "wb"))

        return ret, mtx, dist, rvecs, tvecs

    def undistort_calibration_images(self):

        mtx = self.mtx
        dist = self.dist

        img1 = cv2.imread(CALIBRATION_IMG_DIR + "calibration1.jpg")
        img2 = cv2.imread(CALIBRATION_IMG_DIR + "calibration4.jpg")
        img3 = cv2.imread(CALIBRATION_IMG_DIR + "calibration5.jpg")
        udst1 = cv2.undistort(img1, mtx, dist, None, mtx)
        udst2 = cv2.undistort(img2, mtx, dist, None, mtx)
        udst3 = cv2.undistort(img3, mtx, dist, None, mtx)
        # Visualize undistortion
        f, ((ax11, ax12, ax13), (ax21, ax22, ax23)) = plt.subplots(2, 3, figsize=(30, 20))
        ax11.imshow(img1)
        ax11.set_title('Original Image 1')
        ax12.imshow(img2)
        ax12.set_title('Original Image 2')
        ax13.imshow(img3)
        ax13.set_title('Original Image 3')
        ax21.imshow(udst1)
        ax21.set_title('Undistorted Image 1')
        ax22.imshow(udst2)
        ax22.set_title('Undistorted Image 2')
        ax23.imshow(udst3)
        ax23.set_title('Undistorted Image 3')
        plt.show()

    def undistort_test_images(self):

        mtx = self.mtx
        dist = self.dist

        img1 = cv2.cvtColor(cv2.imread("./test_images/test1.jpg"), cv2.COLOR_BGR2RGB)
        img2 = cv2.cvtColor(cv2.imread("./test_images/test4.jpg"), cv2.COLOR_BGR2RGB)
        img3 = cv2.cvtColor(cv2.imread("./test_images/test6.jpg"), cv2.COLOR_BGR2RGB)
        udst1 = cv2.undistort(img1, mtx, dist, None, mtx)
        udst2 = cv2.undistort(img2, mtx, dist, None, mtx)
        udst3 = cv2.undistort(img3, mtx, dist, None, mtx)

        # Visualize undistortion
        f, ((ax11, ax12, ax13), (ax21, ax22, ax23)) = plt.subplots(2, 3, figsize=(30, 20))
        ax11.imshow(img1)
        ax11.set_title('Original Image 1')
        ax12.imshow(img2)
        ax12.set_title('Original Image 2')
        ax13.imshow(img3)
        ax13.set_title('Original Image 3')
        ax21.imshow(udst1)
        ax21.set_title('Undistorted Image 1')
        ax22.imshow(udst2)
        ax22.set_title('Undistorted Image 2')
        ax23.imshow(udst3)
        ax23.set_title('Undistorted Image 3')
        plt.show()

    def undistort(self, img):

        mtx = self.mtx
        dist = self.dist

        udst = cv2.undistort(img, mtx, dist, None, mtx)

        return udst