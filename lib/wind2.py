# Estimate wind vector given indicated airspeed, aircraft heading
# (true), and gps ground velocity vector.  This function is designed
# to be called repeatedly to update the wind estimate in real time.

import math
from matplotlib import pyplot as plt
import numpy as np
from scipy import interpolate
from scipy import signal
from scipy.optimize import least_squares
from scipy.stats import pearsonr
from tqdm import tqdm

from lib import lowpass

# useful constants
d2r = math.pi / 180.0
r2d = 180.0 / math.pi
mps2kt = 1.94384
kt2mps = 1 / mps2kt

class Wind2():
    def __init__(self):
        self.pitot_time_factor = 240
        self.psi_error_time_factor = 60
        self.ps = 1
        self.last_time = 0

    def opt_err(self, xk):
        ps = xk[0]              # pitot scale
        yb = xk[1]              # yaw angle bias
        yb = 0
        un = np.sin(self.data[:,1] + yb) * self.data[:,2] * ps
        ue = np.cos(self.data[:,1] + yb) * self.data[:,2] * ps
        wn = self.data[:,3] - un  # vn - un
        we = self.data[:,4] - ue  # ve - ue
          
        # minimize the error between filtered wind and raw wind
        # wn_filt = signal.filtfilt(self.b, self.a, wn)
        # we_filt = signal.filtfilt(self.b, self.a, we)
        # return np.concatenate( [wn_filt - wn, we_filt - we] )

        # minimize the correlation between body velocity vector and
        # wind velocity vector
        corr_n, _ = pearsonr(un, wn)
        corr_e, _ = pearsonr(ue, we)
        print('Pearsons correlation: %.3f %.3f' % (corr_n, corr_e))

        return [ corr_n, corr_e ]
    
    # run a quick wind estimate and pitot calibration based on nav
    # estimate + air data
    def estimate(self, iter, imu_dt):
        print("Estimating winds aloft:")
        winds = []
        asi_mps = 0
        yaw_rad = 0
        vn = 0
        ve = 0
        wind_deg = 0
        wind_kt = 0
        ps = 1.0
        data = []
        for i in tqdm(range(iter.size())):
            record = iter.next()
            if len(record):
                t = record['imu']['time']
                if 'air' in record:
                    asi_mps = record['air']['airspeed'] * kt2mps
                if asi_mps < 5.0:
                    continue
                if "filter" in record:
                    navpt = record["filter"]
                else:
                    continue
                yaw_rad = navpt['psi']
                psi = 0.5*math.pi - yaw_rad
                vn = navpt['vn']
                ve = navpt['ve']
                ue = math.cos(psi) * asi_mps
                un = math.sin(psi) * asi_mps
                # print("yaw_deg: %0f psi_deg: %.2f" % (yaw_rad*r2d, psi*r2d),
                #       "ue: %.1f un: %.1f" % (ue, un),
                #       "ve: %.1f vn: %.1f" % (ve, vn))
                # instantaneous wind velocity
                we = ve - ue
                wn = vn - un
                data.append( [t, psi, asi_mps, vn, ve, un, ue] )
                
        self.data = np.array(data)
        vn = self.data[:,3]
        ve = self.data[:,4]
        
        fs = len(data) / (data[-1][0] - data[0][0])
        #fs = (1 / imu_dt)
        print("fs:", fs)
        cutoff_freq = 1.0 / 100.0  # 1/n hz
        self.b, self.a = signal.butter(2, cutoff_freq, 'lowpass', fs=fs)
        
        res = least_squares(self.opt_err, [1.0, 0.0], verbose=2)
        pitot_scale = res["x"][0]
        psi_bias = res["x"][1]
        print("pitot scale:", pitot_scale)
        print("psi bias (deg):", psi_bias*r2d)
        psi_bias = 0
        un = np.sin(self.data[:,1] + psi_bias) * self.data[:,2] * pitot_scale
        ue = np.cos(self.data[:,1] + psi_bias) * self.data[:,2] * pitot_scale

        un = self.data[:,5] * pitot_scale
        ue = self.data[:,6] * pitot_scale

        wn = vn - un
        we = ve - ue
        wn_filt = signal.filtfilt(self.b, self.a, wn)
        we_filt = signal.filtfilt(self.b, self.a, we)
        wn_mean = np.mean(wn)
        we_mean = np.mean(we)
        print("wind north:", wn_mean, "wind_east:", we_mean)

        # ground velocity
        plt.figure()
        plt.plot(self.data[:,0], vn, label="vn (mps)")
        plt.plot(self.data[:,0], ve, label="ve (mps)")
        plt.xlabel("time (sec)")
        plt.legend()
            
        # body velocity
        plt.figure()
        # indicated
        plt.plot(self.data[:,0], un, label="un indicated (mps)")
        plt.plot(self.data[:,0], ue, label="ue indicated (mps)")
        # true
        true_n = vn - wn_filt
        true_e = ve - we_filt
        plt.plot(self.data[:,0], true_n, label="un true (mps)")
        plt.plot(self.data[:,0], true_e, label="ue true (mps)")
        plt.xlabel("time (sec)")
        plt.legend()

        # wind heading/velocity
        wind_fig, (ax0, ax1) = plt.subplots(2, 1, sharex=True)
        ax0.set_title("Winds Aloft")
        ax0.set_ylabel("Heading (degrees)", weight='bold')
        ax0.plot(self.data[:,0], np.mod(270 - np.arctan2(wn_filt, we_filt)*r2d,360))
        ax0.grid()
        ax1.set_xlabel("Time (secs)", weight='bold')
        ax1.set_ylabel("Speed (kts)", weight='bold')
        speed = np.linalg.norm(np.vstack( [wn_filt, we_filt] ), axis=0)
        ax1.plot(self.data[:,0], speed*mps2kt, label="Wind Speed")
        ax1.grid()
        ax1.legend()

        # wind adjusted for (estimated) pitot scaling
        plt.figure()
        plt.plot(self.data[:,0], wn, label="wn raw (mps)")
        plt.plot(self.data[:,0], we, label="we raw (mps)")
        plt.plot(self.data[:,0], wn_filt, label="wn filt (mps)")
        plt.plot(self.data[:,0], we_filt, label="we filt (mps)")
        plt.xlabel("time (sec)")
        plt.legend()

        # north velocies
        plt.figure()
        plt.plot(self.data[:,0], wn, label="wn raw (mps)")
        plt.plot(self.data[:,0], un, label="un indicated (mps)")
        plt.plot(self.data[:,0], vn, label="vn (mps)")
        plt.xlabel("time (sec)")
        plt.legend()
        
        # east velocies
        plt.figure()
        plt.plot(self.data[:,0], we, label="we raw (mps)")
        plt.plot(self.data[:,0], ue, label="ue indicated (mps)")
        plt.plot(self.data[:,0], ve, label="ve (mps)")
        plt.xlabel("time (sec)")
        plt.legend()
        
        # heading correction
        true_psi = np.arctan2(true_n, true_e)
        logged_psi = np.arctan2(un, ue)
        plt.figure()
        plt.plot(self.data[:,0], logged_psi*r2d, label="original psi")
        plt.plot(self.data[:,0], true_psi*r2d, label="true psi")
        plt.xlabel("time (sec)")
        plt.legend()

        # airspeed
        #print(np.vstack( [true_n, true_e] ))
        true_v = np.linalg.norm(np.vstack( [true_n, true_e] ), axis=0)
        plt.figure()
        plt.plot(self.data[:,0], true_v, label="est true airspeed (mps)")
        plt.plot(self.data[:,0], self.data[:,2]*pitot_scale, label="true airspeed (mps)")
        plt.legend()
        
        plt.show()

        wn_interp = interpolate.interp1d(self.data[:,0], wn_filt,
                                         bounds_error=False,
                                         fill_value='extrapolate')
        we_interp = interpolate.interp1d(self.data[:,0], we_filt,
                                         bounds_error=False,
                                         fill_value='extrapolate')
        return pitot_scale, psi_bias, wn_interp, we_interp
