# one issue of a least squares fit of a system is if it spends a high % of time
# in a narrow band of conditions, and there is sensor noise, you end up with a
# linear fit through a big ball of noise.
#
# This is a quick experiment to test the idea of binning the input data,
# computing an average per bin, and then doing a linear interpolation between
# bins.  Oh, and building the fint on the fly in real time, because that seems
# like fun.
#
# just handles a single term per bin, but later could be expanded to multiple
# term fits if this approach seems productive.

import numpy as np

class AverageBin():
    def __init__(self, x_val, tf=10):
        self.x_val = x_val
        self.tf = tf
        self.count = 0
        # self.x_sum = 0
        self.y_sum = 0

    def AddData(self, y, dt):
        if self.count == 0:
            self.y_val = y
        self.count += 1
        # self.y_sum += y
        # self.avg = self.y_sum / self.count
        tf = self.count * dt
        if tf > self.tf:
            tf = self.tf
        w = dt / self.tf
        self.y_val = (1-w)*self.y_val + w*y

class BinnedFit():
    def __init__(self, minx, maxx, num_bins):
        self.minx = minx
        self.maxx = maxx
        self.num_bins = num_bins

        self.range = self.maxx - self.minx
        bin_size = self.range / self.num_bins
        self.bins = []
        for i in range(num_bins):
            x = self.minx + 0.5*bin_size + i*bin_size
            print("make bin x:", x)
            bin = AverageBin(x)
            self.bins.append(bin)
        # print("bins:", self.bins)

    def AddData(self, x, y, dt):
        # compute bin
        # print("%.3f" % (((x - self.minx) / self.range) * self.num_bins))
        bin_num = int(((x - self.minx) / self.range) * self.num_bins)
        if bin_num >= self.num_bins:
            bin_num = self.num_bins - 1
        print("x:", x, "bin_num:", bin_num)
        self.bins[bin_num].AddData(y, dt)

    def Interp(self, x):
        self.xp = []
        self.fp = []
        for bin in self.bins:
            if bin.count > 0:
                self.xp.append(bin.x_val)
                self.fp.append(bin.y_val)
        if len(self.xp):
            f = open("flap0.gnuplot", "w")
            for i in range(len(self.xp)):
                f.write("%.3f %.3f\n" % (self.xp[i], self.fp[i]))
            f.close()
            return np.interp(x, self.xp, self.fp)
        else:
            return 0

# a = BinnedFit(-1, 1, 4)

# a.fp = [ -0.75, 12, 0.25, 0.75]

# for i in np.linspace(-1, 1, 37):
#     # print(i, np.interp(i, a.xp, a.fp))
#     print(i, a.AddData(i, 0))