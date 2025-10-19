import sys, os
from ament_index_python.packages import get_package_share_directory, get_package_prefix

import yaml
import imagesize
from pprint import pprint

from environment_common.convertors.templating.kml import KmlTemplates, KmlDraw
from environment_common.convertors.tools.gps import get_gps_from_datumrelative_metric


def run(args=None):
    # Load the datum.yaml file
    datum_path = os.path.join(args['src'], 'config', 'location', 'datum.yaml')
    if not os.path.isfile(datum_path):
        datum_path = os.path.join(args['src'], 'config', 'location', 'datum_autogen.yaml')
    with open(datum_path) as f:
        data = f.read()
        datum_raw = yaml.safe_load(data)

    # Load the map.yaml file
    mapyaml_path = os.path.join(args['src'], 'config', 'metric', 'map', 'map.yaml')
    if not os.path.isfile(mapyaml_path):
        mapyaml_path = os.path.join(args['src'], 'config', 'metric', 'map', 'map_autogen.yaml')
    with open(mapyaml_path) as f:
        data = f.read()
        mapyaml = yaml.safe_load(data)

    # Extract the useful information
    name = args['location_name']
    resolution = mapyaml['resolution']
    image = mapyaml['image']
    tpimage = mapyaml['image'].replace('.','-tp.')

    # Load the map file to get width and height
    map_path = os.path.join(args['src'], 'config', 'metric', 'map', image)
    width, height = imagesize.get(map_path)
    size = {'width':width, 'height':height}

    # Get the map origin as gps
    datum = {'latitude': datum_raw['datum_latitude'], 'longitude': datum_raw['datum_longitude'], 'elevation':0}
    distance_from_datum_to_sw = {'x':mapyaml['origin'][0], 'y':mapyaml['origin'][1], 'z':0}
    print('calculate back offset:')
    print(' datum gps', datum)
    print('datum 2 sw', distance_from_datum_to_sw)
    sw_gps = get_gps_from_datumrelative_metric(datum, distance_from_datum_to_sw)
    print('    new sw', sw_gps)
    print('\n\nComparison:')
    print('new sw', sw_gps)
    print('\n\n')

    # Get far corner of map
    w, h = width*resolution, height*resolution
    distance_from_sw_to_ne = {'x':w, 'y':h, 'z':0}
    ne_gps = get_gps_from_datumrelative_metric(sw_gps, distance_from_sw_to_ne)

    # Get bounds
    bounds = {'north':ne_gps['latitude'], 'east':ne_gps['longitude'], 'south':sw_gps['latitude'], 'west':sw_gps['longitude']}
    north = bounds['north']
    east = bounds['east']
    south = bounds['south']
    west = bounds['west']

    # Get the href
    href = f"https://raw.githubusercontent.com/LCAS/environment_template/{name}/config/metric/map/{image}"
    tphref = f"https://raw.githubusercontent.com/LCAS/environment_template/{name}/config/metric/map/{tpimage}"
    print(f"Image must be visible as a raw github link. Attempting to load file from the following:\n{href}\n")
    #inp = input("Is this correct (Y/n): ")
    #if inp == 'n':
    #    user = input("Enter user account (default=LCAS): ") or "LCAS"
    #    repo = input("Enter repository name (default=environment_template): ") or "environment_template"
   #     branch = input(f"Enter branch name (default={name}): ") or name
   #     href = f"https://raw.githubusercontent.com/{user}/{repo}/{branch}/config/metric/map/{image}"
   #     tphref = f"https://raw.githubusercontent.com/{user}/{repo}/{branch}/config/metric/map/{tpimage}"
   #     print(f"Attempting to load file from the following:\n{href}\n")
   #     inp2 = input("Is this correct (Y/n): ")
   #     if inp2 == 'n':
   #         href = input("Enter url directly here: ")

    # Create image kml from templates
    kml = KmlTemplates.opening % f"{args['location_name']}_auto_metric"
    #kml += KmlTemplates.point % ('A', 'NE', east, north)
    #kml += KmlTemplates.point % ('B', 'SW', west, south)
    #kml += KmlTemplates.point % ('C', 'Datum', datum_raw['datum_longitude'], datum_raw['datum_latitude'])
    #kml += KmlTemplates.point % ('Aa', 'SW_ori', "-0.5318159296480074", "53.26480228820538")
    #kml += KmlTemplates.point % ('Bb', 'NE_ori', "-0.5248664430937977", "53.26753171791476")
    kml += KmlTemplates.image % ('0', 'auto_raw', 1, href, north, south, west, east, 0.0)
    #kml += KmlTemplates.image % ('0', 'auto_tp', 1, href, north, south, west, east, 0.0)
    kml += KmlTemplates.closing

    # Save kml file
    kml_path = os.path.join(args['src'], 'config', 'metric', 'map', 'metric_autogen.kml')
    with open(kml_path, 'w') as f:
        f.write(kml)

def main(args=None):
    e = 'environment_template'
    src = '/'.join(get_package_prefix(e).split('/')[:-2]) + f'/src/{e}'
    location_name = 'r_gep'
    args = {'src': src, 'location_name':location_name}
    run(args)

if __name__ == '__main__':
    main()
