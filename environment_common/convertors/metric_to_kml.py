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
    origin = {'x':mapyaml['origin'][0], 'y':mapyaml['origin'][1]}
    image = mapyaml['image']

    # Load the map file ot get width and height
    map_path = os.path.join(args['src'], 'config', 'metric', 'map', image)
    width, height = imagesize.get(map_path)
    size = {'width':width, 'height':height}
    href = f"https://raw.githubusercontent.com/LCAS/environment_template/{name}/config/metric/map/map-tp.png"

    # Create image kml from templates
    kml = KmlTemplates.opening % f"{args['location_name']}_auto_metric"
    kml += KmlDraw.draw_image('0', name, datum, origin, resolution, size, href=href, rotation=0.0)
    kml += KmlTemplates.closing

    print(kml)

    kml_path = os.path.join(args['src'], 'config', 'metric', 'map', 'metric_autogen.kml')
    with open(kml_path, 'w') as f:
        f.write(kml)

    gdrive_path = os.path.join(os.getenv('GDRIVE_PATH'), 'Google Earth', 'kml', 'metric_autogen.kml')
    with open(gdrive_path, 'w') as f:
        f.write(kml)

def main(args=None):
    e = 'environment_template'
    src = '/'.join(get_package_prefix(e).split('/')[:-2]) + f'/src/{e}'
    location_name = 'riseholme_polytunnel'
    args = {'src': src, 'location_name':location_name}
    run(args)

if __name__ == '__main__':
    main()
