import cv2
import numpy as np
import skimage.feature          # pip3 install scikit-image

# manage local binary patterns represenation

class LBP():
    def __init__(self, image, radius=3):
        # compute the Local Binary Pattern representation of the image
        print("Computing LBP")
        self.lbp_map = None
        self.numPoints = radius * 8
        print(image.shape)
        if len(image.shape) == 3:
            print("LBP needs a gray scale image, cannot continue")
        else:
            self.lbp_map = skimage.feature.local_binary_pattern(image, self.numPoints, radius, method="uniform")

    # use the LBP representation and build a histogram of values around the specified point
    def gen_classifier(self, row, col, span=10):
        r1 = row - span
        if r1 < 0: r1 = 0
        r2 = row + span
        c1 = col - span
        if c1 < 0: c1 = 0
        c2 = col + span
        region = self.lbp_map[r1:r2,c1:c2]
        if region.size < 1:
            print(r1, r2, c1, c2, region.size)
        (hist, _) = np.histogram(region.ravel(), bins=np.arange(0, self.numPoints + 3), range=(0, self.numPoints + 2))
        hist = hist.astype('float') / region.size # normalize
        if False:
            # dist histogram
            plt.figure()
            y_pos = np.arange(len(hist))
            plt.bar(y_pos, hist, align='center', alpha=0.5)
            plt.xticks(y_pos, range(len(hist)))
            plt.ylabel('count')
            plt.title('classifier')
            plt.show()
        return hist
