# Todo

* remove personal maptiler key, put in a config file: ~/.nsWorld/maptiler.txt
  (currently hard coded into tile_builder.py)
* internally cache maptiler images under maptiler/ not satellite/
* onscreen tile size from 4 corners instead of center-radius? (probalby no)  Can
  we support a tight bounding box?
* blend patches with surrounding srtm data better
* consolodate dot_root locations
* Coordinate system(s):
  * define/document coordinate system
  * Make world/tile_mgr interface accept lla only (it figures out nedref and
    stuffs)
* Demo script that updates it's position (drone mode?) so we can do devel
  testing, quick demos, no joystick needed, etc.
* Separate out textures from bam files (so we could manipulate the texture or
  select from a different set at runtime).
* Manage a cache of textures and replace content instead of just
  loading/unloading textures.  Suspect card/driver default memory management is
  leading to big pauses in rendering.
* 4x consolodated sub textures instead of 4x sub tiles, but recursively done.
