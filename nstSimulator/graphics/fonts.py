import os
from panda3d.core import *

from nstSimulator import get_data_path

# fixme, is there a better way to do this?  We just want to load the font once,
# but we need panda3d initialized first, so we can't do it outside of a later
# function call.

B612_font = None

def get_B612_font():
    global B612_font
    if not B612_font:
        file_path = get_data_path()
        base_path = os.path.join(file_path, "fonts", "B612-Regular.ttf")
        print("loading: ", base_path)
        B612_font = loader.loadFont(str(Filename.fromOsSpecific(base_path)))
        B612_font.setPageSize(512,512)
        B612_font.setPixelsPerUnit(80)
    return B612_font
