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