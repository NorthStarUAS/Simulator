# simple computation of arc coordinates, but can be reused by several higher
# level object generators.

from math import cos, sin
import numpy as np

from nstSimulator.utils.constants import d2r

def gen_arc(radius=1, start_deg=0, end_deg=360, divs=10):
    divs_deg = np.linspace(start_deg, end_deg, divs+1)
    print("divs:", divs_deg)
    result = []
    for a_deg in divs_deg:
        a_rad = (90 - a_deg) * d2r
        x = cos(a_rad) * radius
        y = sin(a_rad) * radius
        print("gen_arc:", x, y)
        result.append( (x, y) )
    return result

def gen_arc_list(radius=1, angle_list=[]):
    result = []
    for a_deg in angle_list:
        print("a_deg:", a_deg)
        a_rad = (90 - a_deg) * d2r
        x = cos(a_rad) * radius
        y = sin(a_rad) * radius
        print("gen_arc:", x, y)
        result.append( (x, y) )
    return result