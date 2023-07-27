import sys, os
from ament_index_python.packages import get_package_share_directory, get_package_prefix

import yaml
from PIL import Image
from pprint import pprint

from environment_common.convertors.templating.metric import MetricTemplates
from environment_common.convertors.tools.gps import get_bounds, get_range, get_datumrelative_metric_from_gps


def run(args=None):
    # Load the datum.yaml file
    datum_path = os.path.join(args['src'], 'config', 'location', 'datum.yaml')
    if not os.path.isfile(datum_path):
        datum_path = os.path.join(args['src'], 'config', 'location', 'datum_autogen.yaml')
    with open(datum_path) as f:
        data = f.read()
        datum_raw = yaml.safe_load(data)

    # Load the image
    bounds = get_bounds(datum_raw['gnss_fence']['gnss_fence_coords'])
    x, y = get_range(bounds)

    # Specify details
    image = "map_autogen.png"
    resolution = "0.25000"
    print(w, h)

    # Calculate offset from datum gps to south west corner of map
    sw_gps = {'latitude':bounds['south'], 'longitude':bounds['west'], 'elevation':0}
    datum_gps = {'latitude':datum_raw['datum_latitude'], 'longitude':datum_raw['datum_longitude'], 'elevation':0}
    origin_from_datum = get_datumrelative_metric_from_gps(sw_gps, datum_gps)

    # Populate the map.yaml
    mapyaml_raw = MetricTemplates.mapyaml % (image, resolution, origin_from_datum['x'], origin_from_datum['y'])
    mapyaml_path = os.path.join(args['src'], 'config', 'metric', 'map', 'map_autogen.yaml')
    with open(mapyaml_path, 'w') as f:
        f.write(mapyaml_raw)

    # Generate the map image
    img = Image.new("L", (w, h), (255))
    img_path = os.path.join(args['src'], 'config', 'metric', 'map', image)
    img.save(img_path, "PNG")


def main(args=None):
    e = 'environment_template'
    src = '/'.join(get_package_prefix(e).split('/')[:-2]) + f'/src/{e}'
    location_name = 'riseholme_polytunnel'
    run({'src': src, 'location_name':location_name})

if __name__ == '__main__':
    main()
