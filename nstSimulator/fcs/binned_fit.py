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
    def __init__(self, x_vals=[], tf=10):
        if type(x_vals) is not list:
            x_vals = [x_vals]

        # bin
        self.x_vals = x_vals
        self.tf = tf
        self.y_filt = 0
        self.xp = []
        self.fp = []

        # welford (Knuth) mean/var
        self.k = 0
        self.M = 0
        self.S = 0
        self.var = 0

    # Numerically robust incremental mean and variance (Knuth)
    # From: https://www.embeddedrelated.com/showarticle/785.php
    def welford(self, x):
        self.k += 1
        Mnext = self.M + (x - self.M) / self.k
        self.S = self.S + (x - self.M)*(x - Mnext)
        self.M = Mnext
        if self.k > 1:
            self.var = self.S/(self.k-1)
        else:
            self.var = 0

    def AddData(self, y, dt):
        self.welford(y)

        if self.k == 1:
            self.y_filt = y
        # self.y_sum += y
        # self.y_avg = self.y_sum / self.count
        tf = self.k * dt
        if tf > self.tf:
            tf = self.tf
        w = dt / tf
        self.y_filt = (1-w)*self.y_filt + w*y

class IntervalBinnedFit1d():
    def __init__(self, bin_width):
        self.bin_width = bin_width
        self.half_bin_width = bin_width * 0.5
        br = ceil(-log10(self.bin_width)) + 1  # compute number of sig digits of cell size (+ 1) for bin naming
        if br < 0: br = 0
        self.bin_round = br
        self.total_count = 0
        self.bins = {}
        self.fit_model = [0, 0]
        self.xp = []
        self.fp = []

    def AddData(self, x, y, dt):
        # compute bin
        # print("%.3f" % (((x - self.minx) / self.range) * self.num_bins))
        key = round( floor(x / self.bin_width) * self.bin_width + self.half_bin_width, self.bin_round )
        # print("%.6f" % key)
        self.total_count += 1
        if not key in self.bins:
            self.bins[key] = AverageBin([key])
        self.bins[key].AddData(y, dt)
        # print("x:", x, "key:", key)

    def FitModel(self):
        self.xp = []
        self.fp = []
        self.w = []
        for bin in self.bins.values():
            self.xp.append(bin.x_vals[0])
            self.fp.append(bin.M)
            self.w.append(bin.k / self.total_count)
        if len(self.xp):
            self.fit_model = np.polyfit(self.xp, self.fp, deg=1, w=self.w)
            # print("polyfit:", self.fit_model)

    def Interp(self, x):
        return self.fit_model[0]*x + self.fit_model[1]

class IntervalBinnedFit():
    def __init__(self, bin_widths=[]):
        self.dim = len(bin_widths)
        self.bin_widths = []
        self.half_bin_widths = []
        self.bin_rounds = []
        self.max_vals = [float('-inf')] * self.dim
        self.min_vals = [float('inf')] * self.dim
        for bin_width in bin_widths:
            self.bin_widths.append(bin_width)
            self.half_bin_widths.append(bin_width * 0.5)
            br = ceil(-log10(bin_width)) + 1  # compute number of sig digits of cell size (+ 1) for bin naming
            if br < 0: br = 0
            self.bin_rounds.append(br)
        self.total_count = 0
        self.bins = {}
        self.fit_model = None
        self.A = []
        self.B = []

    def AddData(self, xs=[], y=0, dt=0.01):
        # compute bin
        # print("%.3f" % (((x - self.minx) / self.range) * self.num_bins))
        key=""
        ks = []
        for i in range(self.dim):
            if xs[i] < self.min_vals[i]: self.min_vals[i] = xs[i]
            if xs[i] > self.max_vals[i]: self.max_vals[i] = xs[i]
            if i > 0:
                key += ","
            k = ( round( floor(xs[i] / self.bin_widths[i]) * self.bin_widths[i] + self.half_bin_widths[i], self.bin_rounds[i] ) )
            ks.append(k)
            key += str(k)
            # print("kk:", str(ks))
        # print("key:", key)
        self.total_count += 1
        if not key in self.bins:
            self.bins[key] = AverageBin(ks)
        self.bins[key].AddData(y, dt)
        # print("x:", x, "key:", key)

    def FitModel(self):
        # solve AX = B with weights
        # https://theoryl1.wordpress.com/2016/08/03/solve-weighted-least-squares-with-numpy/

        if len(self.bins) <= self.dim:
            print("not enough bins to fit model")
            return
        self.A = []
        self.B = []
        self.W = []
        # print("bins:", self.bins)
        for bin in self.bins.values():
            self.A.append(bin.x_vals + [1])
            self.B.append(bin.M)
            self.W.append(bin.k / self.total_count)
        # print("xp:", self.xp)
        Aw = self.A * np.sqrt(np.array(self.W)[:, np.newaxis])
        Bw = np.array(self.B) * np.sqrt(np.array(self.W))
        result = np.linalg.lstsq(Aw, Bw, rcond=None)
        self.fit_model = result[0]
        # print("fit_model:", self.fit_model)

    def Interp(self, xs):
        result = 0
        if self.fit_model is None:
            print("Interp: no fit model")
        elif len(xs) != self.dim:
            print("Interp: wrong dimension:", len(xs), "expected:", self.dim)
        else:
            # xs = np.clip(xs, self.min_vals, self.max_vals)
            for i in range(self.dim):
                result += self.fit_model[i]*xs[i]
            result += self.fit_model[-1]
        return result

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
        # print("x:", x, "bin_num:", bin_num)
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
            self.fit_model = np.polyfit(self.xp, self.fp, deg=1, w=self.w)[0]
            print("polyfit:", self.fit_model)

    def Interp(self, x):
        return self.fit_model[0]*x + self.fit_model[1]

# a = BinnedFit(-1, 1, 4)

# a.fp = [ -0.75, 12, 0.25, 0.75]

# for i in np.linspace(-1, 1, 37):
#     # print(i, np.interp(i, a.xp, a.fp))
#     print(i, a.AddData(i, 0))

if __name__ == "__main__":
    from math import sqrt
    import random
    import matplotlib.pyplot as plt

    if True:
        a = IntervalBinnedFit1d(0.5)

        for i in range(1000):
            x = random.uniform(-10, 10)
            y = 2.0*x + 3.0 + random.gauss(0, 5.0)
            a.AddData(x, y, 0.1)

        a.FitModel()

        xs = []
        ys = []
        ys_fit = []
        err = []
        for x_vals, bin in sorted(a.bins.items()):
            print("bin xs:", x_vals, "M:", bin.M, "var:", bin.var, "k:", bin.k)
            xs.append(x_vals)
            ys.append(bin.M)
            err.append(sqrt(bin.var))
            ys_fit.append(a.Interp(x_vals))

        # plt.plot(xs, ys, label="true")
        plt.plot(xs, ys_fit, label="binned fit")
        plt.errorbar(xs, ys, err, color='red', fmt='o', label="binned averages")
        plt.grid()
        plt.legend()

        plt.show()

    if True:
        b = IntervalBinnedFit([0.01, 0.5, 10])
        for i in range(1000):
            x1 = random.uniform(-0.2, 0.2)
            x2 = random.uniform(-10, 10)
            x3 = random.uniform(-100, 100)

            y = 2.0*x1 + 3.0*x2 + 4.0*x3 + random.gauss(0, 1.0)
            b.AddData([x1, x2, x3], y, 0.1)
            b.FitModel()
            print("y: %.1f y_fit %.1f  diff: %.2f" % (y, b.Interp([x1, x2, x3]), y - b.Interp([x1, x2, x3])))

        # xs = []
        # ys = []
        # ys_fit = []
        # err = []
        # for x_vals, bin in sorted(b.bins.items()):
        #     print("bin xs:", x_vals, "M:", bin.M, "var:", bin.var, "k:", bin.k)
        #     xs.append(x_vals)
        #     ys.append(bin.M)
        #     err.append(sqrt(bin.var))
        #     ys_fit.append(a.Interp(x_vals))

        # # plt.plot(xs, ys, label="true")
        # plt.plot(xs, ys_fit, label="binned fit")
        # plt.errorbar(xs, ys, err, color='red', fmt='o', label="binned averages")
        # plt.grid()
        # plt.legend()

        # plt.show()
