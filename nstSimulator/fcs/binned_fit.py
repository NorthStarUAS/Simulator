# one issue of a least squares fit of a system is if it spends a high % of time
# in a narrow band of conditions, and there is sensor noise, you end up with a
# linear fit through a big ball of noise.  There can be an ill defined localized
# slope that dominates the slope of the fit.
#
# This is a quick experiment to test the idea of binning the input data,
# computing an average per bin, and then doing a linear interpolation between
# bins.  Oh, and building the fint on the fly in real time, because that seems
# like fun.
#
# The average fit is weighted by number of data points in each bin.  This gives
# most authority to the are of the envelope that has the most data, but avoids
# localized slope bias due to noise (or a flight controller pushing back on
# disturbances.)
#
# This is just a 1d set of bins, but later this could be expanded to higher
# dimensions.
#
# The final fit is currently linear, but this could also be extended to higher
# order polygons (but the extrapolation beyond the are where we have data
# becomes a concern.)

from math import ceil, floor, log10
import numpy as np

class AverageBin():
    def __init__(self, x_val, tf=10):
        self.x_val = x_val
        self.tf = tf
        self.count = 0
        self.y_sum = 0
        self.y_avg = 0
        self.y_filt = 0
        self.xp = []
        self.fp = []

    def AddData(self, y, dt):
        if self.count == 0:
            self.y_filt = y
        self.count += 1
        self.y_sum += y
        self.y_avg = self.y_sum / self.count
        tf = self.count * dt
        if tf > self.tf:
            tf = self.tf
        w = dt / tf
        self.y_filt = (1-w)*self.y_filt + w*y

class IntervalBinnedFit():
    def __init__(self, bin_width):
        self.bin_width = bin_width
        self.half_bin_width = bin_width * 0.5
        br = ceil(-log10(self.bin_width)) + 1  # compute number of sig digits of cell size (+ 1) for bin naming
        if br < 0: br = 0
        self.bin_round = br
        self.total_count = 0
        self.bins = {}
        self.fit_model = [0, 0]

    def AddData(self, x, y, dt):
        # compute bin
        # print("%.3f" % (((x - self.minx) / self.range) * self.num_bins))
        key = round( floor(x / self.bin_width) * self.bin_width + self.half_bin_width, self.bin_round )
        print("%.6f" % key)
        self.total_count += 1
        if not key in self.bins:
            self.bins[key] = AverageBin(key)
        self.bins[key].AddData(y, dt)
        print("x:", x, "key:", key)

    def FitModel(self):
        self.xp = []
        self.fp = []
        self.w = []
        for bin in self.bins.values():
            self.xp.append(bin.x_val)
            self.fp.append(bin.y_avg)
            self.w.append(bin.count / self.total_count)
        if len(self.xp):
            self.fit_model = np.polyfit(self.xp, self.fp, deg=1, w=self.w)
            print("polyfit:", self.fit_model)

    def Interp(self, x):
        return self.fit_model[0]*x + self.fit_model[1]

class old_RangeBinnedFit():
    def __init__(self, minx, maxx, num_bins):
        self.minx = minx
        self.maxx = maxx
        self.num_bins = num_bins
        self.total_count = 0
        self.fit_model = [0, 0]

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
        self.total_count += 1
        bin_num = int(((x - self.minx) / self.range) * self.num_bins)
        if bin_num >= self.num_bins:
            bin_num = self.num_bins - 1
        print("x:", x, "bin_num:", bin_num)
        self.bins[bin_num].AddData(y, dt)

    def FitModel(self):
        self.xp = []
        self.fp = []
        self.w = []
        for bin in self.bins:
            if bin.count > 0:
                self.xp.append(bin.x_val)
                self.fp.append(bin.y_avg)
                self.w.append(bin.count / self.total_count)
        if len(self.xp):
            f = open("flap0.gnuplot", "w")
            for i in range(len(self.xp)):
                f.write("%.3f %.3f\n" % (self.xp[i], self.fp[i]))
            f.close()
            self.fit_model = np.polyfit(self.xp, self.fp, deg=1, w=self.w)
            print("polyfit:", self.fit_model)

    def Interp(self, x):
        return self.fit_model[0]*x + self.fit_model[1]

# a = BinnedFit(-1, 1, 4)

# a.fp = [ -0.75, 12, 0.25, 0.75]

# for i in np.linspace(-1, 1, 37):
#     # print(i, np.interp(i, a.xp, a.fp))
#     print(i, a.AddData(i, 0))