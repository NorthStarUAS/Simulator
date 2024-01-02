# opengl camera

from math import atan2, sin

from panda3d.core import *

from world.constants import r2d

min_cam_agl_m = 2.5

class Camera():
    def __init__(self, config):
        ccd_offset_x = 0
        ccd_offset_y = 0
        print("  focal len:", config["focal_len"])
        print("  lens center offset:", ccd_offset_x, ccd_offset_y)
        print("  ccd dimension:", config["ccd_height"], config["ccd_width"])

        base.camLens.setFocalLength(config["focal_len"])
        base.camLens.setFilmSize(config["ccd_width"], config["ccd_height"])
        self.cam_pos = LVector3f(0.0, 0.0, 0.0)
        self.cam_hpr = LVector3f(0.0, 0.0, 0.0)
        self.timer = 0
        self.north_filt = 0.0
        self.east_filt = 0.0
        self.roll_filt = 0.0
        self.pitch_filt = 0.0
        self.smooth_factor = 0.01

    def update(self, nedpos, nedvel, hpr, ground_elev_m):
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
            # view track's ownship
            self.cam_pos = LVector3f(nedpos[1], nedpos[0], cam_elev_m)
            self.cam_hpr = LVector3f(-hpr[0], hpr[1], hpr[2])
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
        camera.setPos(self.cam_pos)
        camera.setHpr(self.cam_hpr)
        # print("cam set hpr:", camera.getHpr(), self.cam_hpr)

