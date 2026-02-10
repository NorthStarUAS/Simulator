# blends a Panda3d window, camera, and lens for convenience (enabling multiple
# display systems)

from math import atan2, sin

from panda3d.core import *

from nstSimulator.utils.constants import d2r, r2d

min_cam_agl_m = 1.0

class Display():
    def __init__(self, window, camera, config):
        self.fov_deg = config["fov_deg"]
        self.offset_deg = config["offset_deg"]

        self.window = window
        self.camera = camera
        self.lens = self.camera.node().getLens()
        self.lens.setFov(self.fov_deg)

        ccd_offset_x = 0
        ccd_offset_y = 0
        # print("  focal len:", config["focal_len"])
        # print("  lens center offset:", ccd_offset_x, ccd_offset_y)
        # print("  ccd dimension:", config["ccd_height"], config["ccd_width"])
        # self.lens.setFocalLength(config["focal_len"])
        # self.lens.setFilmSize(config["ccd_width"], config["ccd_height"])

        self.cam_pos = LVector3f(0.0, 0.0, 0.0)
        self.cam_hpr = LVector3f(0.0, 0.0, 0.0)
        self.timer = 0
        self.north_filt = 0.0
        self.east_filt = 0.0
        self.roll_filt = 0.0
        self.pitch_filt = 0.0
        self.smooth_factor = 0.01

    def update(self, nedpos, nedvel, hpr, ground_elev_m, ht_hpr=None):
        self.north_filt = (1 - self.smooth_factor) * self.north_filt + self.smooth_factor * nedvel[0]
        self.east_filt = (1 - self.smooth_factor) * self.east_filt + self.smooth_factor * nedvel[1]
        course_filt = 90 - atan2(self.north_filt, self.east_filt)*r2d
        self.roll_filt = (1 - self.smooth_factor) * self.roll_filt + self.smooth_factor * hpr[2]
        self.pitch_filt = (1 - self.smooth_factor) * self.pitch_filt + self.smooth_factor * hpr[1]
        self.timer += 0.004

        cam_elev_m = -nedpos[2]
        if cam_elev_m < ground_elev_m + min_cam_agl_m:
            cam_elev_m = ground_elev_m + min_cam_agl_m

        if True:
            # view track's ownship (augment with head tracker offsets if given)
            camq = Quat()
            camq.setHpr((-hpr[0], hpr[1], hpr[2]))
            if ht_hpr:
                q = Quat()
                # yaw relative to base camera (aircraft) attitude
                q.setHpr((-ht_hpr[0], 0, 0))
                camq = q * camq
                # then pitch
                q.setHpr((0, ht_hpr[1], 0))
                camq = q * camq
                # then roll
                q.setHpr((0, 0, ht_hpr[2]))
                camq = q * camq
            self.cam_pos = LVector3f(nedpos[1], nedpos[0], cam_elev_m)
            self.cam_hpr = camq.getHpr()
            # self.cam_hpr = LVector3f(-hpr[0]-self.offset_deg, hpr[1]-self.offset_deg*sin(hpr[2]*d2r), hpr[2])
            #self.cam_hpr = LVector3f(-course_filt, 0, 0)
        elif False:
            # lazy track aircraft
            self.cam_pos = LVector3f(nedpos[1], nedpos[0], -nedpos[2])
            self.cam_hpr = LVector3f(-course_filt, self.pitch_filt, self.roll_filt)
            #self.cam_hpr = LVector3f(-course_filt, 0, 0)
        elif False:
            self.cam_pos = LVector3f(nedpos[1], nedpos[0], -nedpos[2])
            self.cam_hpr = LVector3f(-course_deg, -90, 0.0)
        elif False:
            factor = (1+sin(self.timer))*0.5  # 0-1
            self.cam_pos = LVector3f(nedpos[1], nedpos[0], -nedpos[2] + factor*7500)
            self.cam_hpr = LVector3f(-course_deg, -factor**0.6 * 70.0, 0.0)
        elif False:
            factor1 = sin(self.timer*2)  # [-1,1]
            factor2 = sin(self.timer*3)  # [-1,1]
            factor3 = sin(self.timer*4)  # [-1,1]
            factor4 = sin(self.timer*5)  # [-1,1]
            self.cam_pos = LVector3f(nedpos[1] + factor2*250, nedpos[0] + factor3*250, -nedpos[2] + factor1*30)
            self.cam_hpr = LVector3f(-course_deg + factor2*0, factor3*0, factor4*0)
        #print(lla, nedpos)
        self.camera.setPos(self.cam_pos)
        self.camera.setHpr(self.cam_hpr)

        # update aspect ratio from current window dimensions
        x = self.window.getXSize()
        y = self.window.getYSize()
        # print("ar:", i, x/y)
        self.lens.setAspectRatio(x/y)

        # print("fov:", self.lens.getFov(), "ar:", self.lens.getAspectRatio())
        # print("cam set hpr:", camera.getHpr(), self.cam_hpr)

        # h = LRotation((0, 0, 1), -hpr[0])
        # p = LRotation((0, 1, 0), hpr[1])
        # r = LRotation((1, 0, 0), hpr[2])
        # o = LRotation((0, 0, 1), 0)
        # print("o:", o)

        # q = r*h
        # print("q:", q, "oq:", o*q)

        # # offsetq = LRotation((0, 0, 1), -self.offset_deg)
        # self.camera.setQuat(q)

        # print("up:", render.getRelativeVector(self.camera, Vec3(0,0,1)))
        up = render.getRelativeVector(self.camera, Vec3(0,0,1))

        # up = self.camera.getUpVector()
        q = self.camera.getQuat()
        o = LRotation(up, -self.offset_deg)
        # o = Quat()
        # o.setHpr((-self.offset_deg, 0, 0))
        self.camera.setQuat(q*o)