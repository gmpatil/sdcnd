import  cv2
import pandas as pd
import csv
import numpy as np
import matplotlib.pyplot as plt


#  https://chatbotslife.com/using-augmentation-to-mimic-human-driving-496b569760a9#.8mcdtb70w
def reduceBrightness(image):
    image1 = cv2.cvtColor(image,cv2.COLOR_RGB2HSV)
    image1 = np.array(image1, dtype = np.float64)
    # random_bright = .5+np.random.uniform()
    image1[:,:,2] = image1[:,:,2]* np.random.uniform(.20, .80)  #  < 1 darken. (> 1 and  < 2) increase brightness
    # image1[:, :, 2] = image1[:, :, 2] * 0.5
    image1[:,:,2][image1[:,:,2]>255]  = 255
    image1 = np.array(image1, dtype = np.uint8)
    image1 = cv2.cvtColor(image1,cv2.COLOR_HSV2RGB)
    return image1



def addShade(image):
    image1 = cv2.cvtColor(image,cv2.COLOR_RGB2HSV)
    image1 = np.array(image1, dtype = np.float64)

    # Add random shadow as a vertical slice of image
    ym, xm = image.shape[0], image.shape[1]
    [x0, x1] = np.random.choice(xm, 2, replace=False)

    m = ( (ym * 1.0) / (x1 - x0))
    c = (-1.0 * m * x0)
    shadeSide = np.random.choice([0, 1])

    for y in range (ym):
        if (shadeSide == 0):
            image1[y, : int( (y - c) / m), 1] = image1[y, : int((y - c) / m), 1] * 0.5
            image1[y, : int( (y - c) / m), 2] = image1[y, : int((y - c) / m), 2] * 0.25
        else:
            image1[y, int( (y - c) / m):, 1] = image1[y, int((y - c) / m):, 1] * 0.5
            image1[y, int( (y - c) / m):, 2] = image1[y, int((y - c) / m):, 2] * 0.25

    image1 = np.array(image1, dtype = np.uint8)
    image1 = cv2.cvtColor(image1,cv2.COLOR_HSV2RGB)
    return image1



training_data_folder = '/home/girish/dev/sdcnd/CarND-Behavioral-Cloning-P3/train/'
csv_file_name = 'driving_log.csv'
csv_file_path = training_data_folder + csv_file_name

df = pd.read_csv(csv_file_path, header=None, usecols=[3, 4, 5, 6])
print("Number of samples {}".format(df[3].count()))
print("Steering Angle Max : {}, Min:{}, Avg:{}".format(df[3].max(), df[3].min(), df[3].mean()))
print("Throttle Max : {}, Min:{}, Avg:{}".format(df[4].max(), df[4].min(), df[4].mean()))
print("Break Max : {}, Min:{}, Avg:{}".format(df[5].max(), df[5].min(), df[5].mean()))
print("Speed Max : {}, Min:{}, Avg:{}".format(df[6].max(), df[6].min(), df[6].mean()))
zero_steering_angles = ((df[3][:] > -0.03) & (df[3][:] < 0.03)).sum()
print(" # of samples with steering Angle > -0.03 and < 0.03 = {}".format(zero_steering_angles))

samples = []
with open(csv_file_path) as csvfile:
    reader = csv.reader(csvfile)
    for line in reader:
        samples.append(line)

samplesArr = np.asarray(samples)
print ("shape " + str(samplesArr.shape) + samplesArr[0, 0])
# samplesa = samplesa.reshape(len(samplesa), len(samplesa[0]))
# print ("after re-shape " + str(samplesa.shape))

# sample = centerImage, leftImage, rightImage,steeringAngle, Throttle, break, Speed

# sa = samplesArr[:, 3]
samplesArr[:, 3] = samplesArr[:, 3].astype(float)
sa = samplesArr[:, 3].astype(float)

import matplotlib.pyplot as plt
import random

plt.hist(sa, bins=100)

print ("Steering angles samples {}".format(samplesArr[:, 3]))
print ( "# of samples with appx zero steering angles (> -0.03 and +0.03 <) : {} ".format(len(sa[(sa > -0.03) & (sa < 0.03)])))
print ( "# of samples with right steering angles (< -0.03) : {} ".format(len(sa[(sa < -0.03)])))
print ( "# of samples with left steering angles (> 0.03) : {} ".format(len(sa[(sa > 0.03)])))

# samplesArr = samplesArr[ (samplesArr[:, 0] == '/home/girish/dev/sdcnd/CarND-Behavioral-Cloning-P3/train1/IMG/center_2017_03_05_19_33_46_729.jpg') |
#             (samplesArr[:, 0] == '/home/girish/dev/sdcnd/CarND-Behavioral-Cloning-P3/train1/IMG/center_2017_03_05_19_33_46_799.jpg')]
# print ("{}  {}".format(len(samplesArr), samplesArr.shape))
# print ("sa[0,0], [1, 0] {}  {}".format(samplesArr[0,0], samplesArr[1,0]))
# print (samplesArr)

centerSamples = samplesArr[ (samplesArr[:, 3].astype(float) > -0.03) & (samplesArr[:, 3].astype(float) < +0.03)]
leftSamples = samplesArr[ (samplesArr[:, 3].astype(float) >= +0.03) ]
rightSamples = samplesArr[ (samplesArr[:, 3].astype(float) <= -0.03) ]

print ("Center {}  Left {}  Right {}".format(len(centerSamples), len(leftSamples), len(rightSamples)))

centerSamples = centerSamples[np.random.choice(centerSamples.shape[0],
                                            int ( ((len(leftSamples) + len(rightSamples))/ 2)), replace=False),
                :]


print ("Center {}  {}".format(len(centerSamples), centerSamples.shape))


# za = samplesa[:,3][( float(samplesa[:, 3]) > -0.03 & float(samplesa[:, 3]) < 0.03)]
# print ("za shape : " + str(za.shape))
# print (za)


print ("left " + samplesArr[0, 1])
leftImage = cv2.cvtColor(cv2.imread(samplesArr[0, 1]), cv2.COLOR_BGR2RGB)

fr = 2
fc = 3
fig = plt.figure(figsize=(15, 3))
fig.suptitle("3 camera images")
sp = fig.add_subplot(fr,fc,1)
sp.set_title("Left")
sp.imshow(leftImage)

print ("center " + samplesArr[0, 0])
center = cv2.cvtColor(cv2.imread(samplesArr[0, 0]), cv2.COLOR_BGR2RGB)

sp = fig.add_subplot(fr,fc,2)
sp.set_title("Center")
sp.imshow(center)

print ("right " + samplesArr[0, 2])
right = plt.imread(samplesArr[0, 2])

sp = fig.add_subplot(fr,fc,3)
sp.set_title("right")
sp.imshow(right)


sp = fig.add_subplot(fr,fc,4)
sp.set_title("center flipped")
sp.imshow(np.fliplr(center))

sp = fig.add_subplot(fr,fc,5)
sp.set_title("change brightness")
sp.imshow(reduceBrightness(center))

sp = fig.add_subplot(fr,fc,6)
sp.set_title("Shade")
sp.imshow(addShade(center))

plt.show()

