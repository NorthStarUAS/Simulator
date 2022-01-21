# Estimate wind vector given indicated airspeed, aircraft heading
# (true), and gps ground velocity vector.  This function is designed
# to be called repeatedly to update the wind estimate in real time.

import math
from matplotlib import pyplot as plt
import numpy as np
from scipy.optimize import least_squares
from scipy import signal
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
        self.wind_tf_long = 60
        self.wind_tf_short = 30
        self.filt_ps = lowpass.LowPassFilter(self.pitot_time_factor, 1.0)
        self.filt_psi_error = lowpass.LowPassFilter(self.psi_error_time_factor, 1.0)
        self.filt_long_wn = lowpass.LowPassFilter(self.wind_tf_long, 0.0)
        self.filt_long_we = lowpass.LowPassFilter(self.wind_tf_long, 0.0)
        self.filt_short_wn = lowpass.LowPassFilter(self.wind_tf_short, 0.0)
        self.filt_short_we = lowpass.LowPassFilter(self.wind_tf_short, 0.0)
        self.ps = 1
        self.last_time = 0

    def update(self, time, airspeed_kt, yaw_rad, vn, ve):
        dt = 0.0
        if self.last_time > 0:
            dt = time - self.last_time
        self.last_time = time

        if dt > 0.0 and airspeed_kt >= 10.0:
            # update values if 'flying' and time has elapsed
            psi = 0.5*math.pi - yaw_rad
            psi += self.filt_psi_error.value

            # estimate body velocity
            ue = math.cos(psi) * (airspeed_kt * self.filt_ps.value * kt2mps)
            un = math.sin(psi) * (airspeed_kt * self.filt_ps.value * kt2mps)
            # print("yaw_deg: %0f psi_deg: %.2f" % (yaw_rad*r2d, psi*r2d),
            #       "ue: %.1f un: %.1f" % (ue, un),
            #       "ve: %.1f vn: %.1f" % (ve, vn))
            # instantaneous wind velocity
            we = ve - ue
            wn = vn - un

            # filtered wind velocity
            self.filt_long_wn.update(wn, dt)
            self.filt_long_we.update(we, dt)
            self.filt_short_wn.update(wn, dt)
            self.filt_short_we.update(we, dt)

            # estimate true airspeed
            true_e = ve - self.filt_long_we.value
            true_n = vn - self.filt_long_wn.value

            # estimate aircraft 'true' heading
            true_psi = math.atan2(true_n, true_e)
            psi_error = true_psi - psi
            if psi_error < -math.pi: psi_error += 2*math.pi
            if psi_error > math.pi: psi_error -= 2*math.pi
            self.filt_psi_error.update(psi_error, dt)
            #print(self.filt_psi_error.value*r2d, psi_error*r2d)

            # estimate pitot tube bias
            true_speed_kt = math.sqrt( true_e*true_e + true_n*true_n ) * mps2kt
            self.ps = true_speed_kt / airspeed_kt
            #print("asi: %.1f  true(est): %.1f true(act): %.1f scale: %.2f" % (airspeed_kt, airspeed_kt * self.filt_ps.value, true_speed_kt, self.filt_ps.value))
            # don't let the scale factor exceed some reasonable limits
            if self.ps < 0.75: self.ps = 0.75
            if self.ps > 1.25: self.ps = 1.25
            self.filt_ps.update(self.ps, dt)

    # run a quick wind estimate and pitot calibration based on nav
    # estimate + air data
    def estimate(self, data, wind_tf_long):
        print("Estimating winds aloft:")
        if wind_tf_long:
            self.wind_tf_long = wind_tf_long
        self.last_time = 0.0
        winds = []
        airspeed = 0
        psi = 0
        vn = 0
        ve = 0
        wind_deg = 0
        wind_kt = 0
        ps = 1.0
        interp = flight_interp.InterpolationGroup(data)
        iter = flight_interp.IterateGroup(data)
        for i in tqdm(range(iter.size())):
            record = iter.next()
            if len(record):
                t = record['imu']['time']
                if 'air' in record:
                    airspeed = record['air']['airspeed']
                filt = interp.query(t, 'filter')
                if not len(filt):
                    continue
                phi = filt['phi']
                psi = filt['psi']
                vn = filt['vn']
                ve = filt['ve']
                #print("Phi:", phi)
                if airspeed > 15.0: # and abs(phi) > 0.2 and abs(phi) < 0.3:
                    self.update(t, airspeed, psi, vn, ve)
                    wn = self.filt_long_wn.value
                    we = self.filt_long_we.value
                    psi_error = self.filt_psi_error.value
                    #print wn, we, math.atan2(wn, we), math.atan2(wn, we)*r2d
                    wind_deg = 90 - math.atan2(wn, we) * r2d
                    if wind_deg < 0: wind_deg += 360.0
                    wind_kt = math.sqrt( we*we + wn*wn ) * mps2kt
                    #print wn, we, ps, wind_deg, wind_kt
                    # make sure we log one record per each imu record
                    winds.append( { 'time': t,
                                    'wind_deg': wind_deg,
                                    'wind_kt': wind_kt,
                                    'pitot_scale': self.filt_ps.value,
                                    'long_wn': self.filt_long_wn.value,
                                    'long_we': self.filt_long_we.value,
                                    'short_wn': self.filt_short_wn.value,
                                    'short_we': self.filt_short_we.value,
                                    'psi_error': self.filt_psi_error.value,
                                    'phi': phi,
                                    'ps': self.ps
                                   } )
        return winds

    def opt_err(self, xk):
        un = np.sin(self.data[:,1] + xk[1]) * self.data[:,2] * xk[0]
        ue = np.cos(self.data[:,1] + xk[1]) * self.data[:,2] * xk[0]
        #wn = self.data[:,3] - self.data[:,5]*xk[0] # vn - un * scale
        #we = self.data[:,4] - self.data[:,6]*xk[0] # ve - ue * scale
        wn = self.data[:,3] - un # vn - un * scale
        we = self.data[:,4] - ue # ve - ue * scale

        wn_filt = signal.filtfilt(self.b, self.a, wn)
        we_filt = signal.filtfilt(self.b, self.a, we)
        
        return np.concatenate( [wn_filt - wn, we_filt - we] )
    
    # run a quick wind estimate and pitot calibration based on nav
    # estimate + air data
    def estimate2(self, iter, imu_dt):
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
