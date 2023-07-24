import sys, os
from ament_index_python.packages import get_package_share_directory, get_package_prefix

import yaml
import imagesize
from pprint import pprint

from environment_common.convertors.templating.kml import KmlTemplates, KmlDraw


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
    with open(mapyaml_path) as f:
        data = f.read()
        mapyaml = yaml.safe_load(data)

    # Extract the useful information
    name = args['location_name']
    datum = {'latitude': datum_raw['datum_latitude'], 'longitude': datum_raw['datum_longitude'], 'elevation':0}
    resolution = mapyaml['resolution']
    origin = {'x':mapyaml['origin'][0], 'y':mapyaml['origin'][1]} #The 2-D pose of the lower-left pixel in the map, as (x, y).
    image = mapyaml['image']
    tpimage = mapyaml['image'].replace('.','-tp.')

    # Load the map file to get width and height
    map_path = os.path.join(args['src'], 'config', 'metric', 'map', image)
    width, height = imagesize.get(map_path)
    size = {'width':width, 'height':height}

    # Get the href
    href = f"https://raw.githubusercontent.com/LCAS/environment_template/{name}/config/metric/map/{image}"
    tphref = f"https://raw.githubusercontent.com/LCAS/environment_template/{name}/config/metric/map/{tpimage}"
    print(f"Image must be visible as a raw github link. Attempting to load file from the following:\n{href}\n")
    inp = input("Is this correct (Y/n): ")
    if inp == 'n':
        user = input("Enter user account (default=LCAS): ") or "LCAS"
        repo = input("Enter repository name (default=environment_template): ") or "environment_template"
        branch = input(f"Enter branch name (default={name}): ") or name
        href = f"https://raw.githubusercontent.com/{user}/{repo}/{branch}/config/metric/map/{image}"
        tphref = f"https://raw.githubusercontent.com/{user}/{repo}/{branch}/config/metric/map/{tpimage}"
        print(f"Attempting to load file from the following:\n{href}\n")
        inp2 = input("Is this correct (Y/n): ")
        if inp2 == 'n':
            href = input("Enter url directly here: ")

    # Create image kml from templates
    kml = KmlTemplates.opening % f"{args['location_name']}_auto_metric"

    # autogen
    kml += KmlDraw.draw_image('0', 'auto_raw', datum, origin, resolution, size, href=href, rotation=0.0, visibility=0)
    kml += KmlDraw.draw_image('1', 'auto_tp', datum, origin, resolution, size, href=tphref, rotation=0.0, visibility=1)
    kml += KmlTemplates.closing

    kml_path = os.path.join(args['src'], 'config', 'metric', 'map', 'metric_autogen.kml')
    with open(kml_path, 'w') as f:
        f.write(kml)

    gdrive_path = os.path.join(os.getenv('GDRIVE_PATH'), 'Google Earth', 'kml', 'metric_autogen.kml')
    with open(gdrive_path, 'w') as f:
        f.write(kml)

def main(args=None):
    e = 'environment_template'
    src = '/'.join(get_package_prefix(e).split('/')[:-2]) + f'/src/{e}'
    location_name = 'riseholme_general_east_pathway'
    args = {'src': src, 'location_name':location_name}
    run(args)

if __name__ == '__main__':
    main()
