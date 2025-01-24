# compute intial positions from airport and runway.

from math import atan2, cos, pi, sin
import numpy as np
import os
from pathlib import Path
import pickle

import navpy

from ..utils.constants import ft2m, m2ft, r2d, d2r

class PositionInit:
    def __init__(self):
        self.apt_rwy_db = {}
        basepath = Path(__file__).parent
        filename = basepath / "../data/airports/apt_rwy_db.pkl"
        if os.path.exists(filename):
            print("Loading the airport, runway db:", filename)
            with open(filename, "rb") as f:
                self.apt_rwy_db = pickle.load(f)
        else:
            print("No airport/runway database:", filename)

    def get_airport(self, apt_id):
        if not apt_id in self.apt_rwy_db:
            print("PosInit not found in database:", apt_id)
            quit()
        return self.apt_rwy_db[apt_id]

    def find_runway(self, apt_id, rwy_id):
        if not apt_id in self.apt_rwy_db:
            print("PosInit not found in database:", apt_id)
            quit()
        apt = self.apt_rwy_db[apt_id]
        found = False
        for rwy in apt["rwys"]:
            if rwy["rwy1"] == rwy_id:
                reverse = False
                found = True
            if rwy["rwy2"] == rwy_id:
                reverse = True
                found = True
            if found:
                return apt["alt_ft"]*ft2m, rwy["lat1"], rwy["lon1"], rwy["lat2"], rwy["lon2"], reverse
        # we didn't find the requested airport/rwy so die!
        print("Request not found in database.  apt_id:", apt_id, "rwy_id:", rwy_id)
        print(apt)
        quit()

    def runway_stats(self, lat1, lon1, lat2, lon2, alt_m):
        nedref = [ (lat1 + lat2) * 0.5, (lon1 + lon2) * 0.5, alt_m ]
        print("nedref:", nedref)
        ned1 = navpy.lla2ned(lat1, lon1, alt_m, nedref[0], nedref[1], nedref[2])
        ned2 = navpy.lla2ned(lat2, lon2, alt_m, nedref[0], nedref[1], nedref[2])
        angle = atan2(ned2[0]-ned1[0], ned2[1]-ned1[1])
        length = np.linalg.norm(ned1-ned2)
        print("end1: %.2f %.2f %.2f" % (ned1[0], ned1[1], ned1[2]))
        print("end2: %.2f %.2f %.2f" % (ned2[0], ned2[1], ned2[2]))
        print("length (m):", length)
        print("angle:", angle*r2d)
        print("heading (true):", (0.5*pi - angle) * r2d)
        return nedref, ned1, ned2, angle

    def takeoff(self, id, rwy):
        alt_m, lat1, lon1, lat2, lon2, reverse = self.find_runway(id, rwy)
        nedref, ned1, ned2, angle = self.runway_stats(lat1, lon1, lat2, lon2, alt_m)

        # takeoff start 100' from end of runway
        dist = 100 * ft2m
        if not reverse:
            takeoff_ned = [ ned1[0] + sin(angle)*dist, ned1[1] + cos(angle)*dist, 0 ]
        elif reverse:
            takeoff_ned = [ ned2[0] - sin(angle)*dist, ned2[1] - cos(angle)*dist, 0 ]
        print("takeoff ned: %.2f %.2f" % (takeoff_ned[0], takeoff_ned[1]))
        takeoff_lla = list(navpy.ned2lla(takeoff_ned, nedref[0], nedref[1], nedref[2]))
        takeoff_lla[2] = alt_m
        print("takeoff lla: %.8f %.8f %.1f" % (takeoff_lla[0], takeoff_lla[1], takeoff_lla[2]))
        angle_deg = angle * r2d
        if reversed: angle_deg += 180
        return takeoff_lla, angle_deg

    def touchdown(self, id, rwy):
        alt_m, lat1, lon1, lat2, lon2, reverse = self.find_runway(id, rwy)
        nedref, ned1, ned2, angle = self.runway_stats(lat1, lon1, lat2, lon2, alt_m)

        # assume TD is 1000' from end of runway
        dist = 1000 * ft2m
        if not reverse:
            td_ned = [ ned1[0] + sin(angle)*dist, ned1[1] + cos(angle)*dist, 0 ]
        elif reverse:
            td_ned = [ ned2[0] - sin(angle)*dist, ned2[1] - cos(angle)*dist, 0 ]
        print("td ned: %.2f %.2f" % (td_ned[0], td_ned[1]))
        td_lla = list(navpy.ned2lla(td_ned, nedref[0], nedref[1], nedref[2]))
        td_lla[2] = alt_m
        print("td lla: %.8f %.8f %.1f" % (td_lla[0], td_lla[1], td_lla[2]))
        return td_lla, td_ned, angle*r2d

    def final_approach(self, id, rwy, dist_nm, gs_deg=3 ):
        dist_m = dist_nm * 1852

        alt_m, lat1, lon1, lat2, lon2, reverse = self.find_runway(id, rwy)
        nedref, ned1, ned2, angle = self.runway_stats(lat1, lon1, lat2, lon2, alt_m)

        td_lla, td_ned, angle_deg = self.touchdown(id, rwy)
        angle = angle_deg*d2r

        if not reverse:
            pt_ned = [ td_ned[0] - sin(angle)*dist_m, td_ned[1] - cos(angle)*dist_m, 0 ]
        elif reverse:
            pt_ned = [ td_ned[0] + sin(angle)*dist_m, td_ned[1] + cos(angle)*dist_m, 0 ]
        print("%.1f nm final ned: %.2f %.2f" % (dist_nm, pt_ned[0], pt_ned[1]))
        pt_lla = list(navpy.ned2lla(pt_ned, nedref[0], nedref[1], nedref[2]))
        pt_lla[2] = alt_m + sin(gs_deg*d2r)*dist_m
        print("%.1f nm final lla: %.8f %.8f %.1f" % (dist_nm, pt_lla[0], pt_lla[1], pt_lla[2]), "AGL: %.1f" % ((pt_lla[2]-alt_m)*m2ft))
        angle_deg = angle * r2d
        if reversed: angle_deg += 180
        return pt_lla, angle_deg

    def pattern_entry(self, id, rwy):
        alt_m, lat1, lon1, lat2, lon2, reverse = self.find_runway(id, rwy)
        nedref, ned1, ned2, angle = self.runway_stats(lat1, lon1, lat2, lon2, alt_m)

        # 45 degree left downwind entry @ pattern altitude
        dist = 2 * 5280 * ft2m
        a = angle + pi*1.25
        # midpoint of the runway is nedref is 0, 0, 0 (ned)
        if not reverse:
            pt_ned = [ 0 - sin(a)*dist, 0 - cos(a)*dist, 0 ]
        elif reverse:
            pt_ned = [ 0 + sin(a)*dist, 0 + cos(a)*dist, 0 ]
        print("45 (left) downwind entry ned: %.2f %.2f" % (pt_ned[0], pt_ned[1]))
        pt_lla = list(navpy.ned2lla(pt_ned, nedref[0], nedref[1], nedref[2]))
        pt_lla[2] = alt_m + 1000*ft2m
        print("45 (left) downwind entry lla: %.8f %.8f %.1f" % (pt_lla[0], pt_lla[1], pt_lla[2]))
        return pt_lla, a*r2d