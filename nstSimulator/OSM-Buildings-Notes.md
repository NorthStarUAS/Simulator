# OSM buildings

maptiler.com has a full export of OSM buildings, but pyosmium cannot parse maptiler.com pbf files.

It's not a .osm.pbf ... but a vector tile file using same extension

ubuntu: pip install --break-system-packages mapbox-vector-tile

    import sys
    import mapbox_vector_tile
    with open(sys.argv[1], "rb") as f:
        data = f.read()
    print(mapbox_vector_tile.decode(data))

Coordinates system:

<https://docs.mapbox.com/data/tilesets/guides/vector-tiles-standards/>

    wget https://api.maptiler.com/tiles/v3/15/7985/11562.pbf?key=1234maptiler.key5678

=============> following doesn't apply to vector tile stuff ....

.pbf google protocol buffer format

pyosmium can parse pbf -> but apparently not maptiler.com pbf files?

Schema for maptiler tiles:

<https://docs.maptiler.com/schema/omt-planet/>

?Open Map Tiles

<https://wiki.openstreetmap.org/wiki/PBF_Format>