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
    #x, y = get_range(bounds)
    #print('xy',[x,y])

    # Specify details
    image = "map_autogen.png"
    resolution = "0.25000"

    # Calculate offset from datum gps to south west corner of map
    datum_gps = {'latitude':datum_raw['datum_latitude'], 'longitude':datum_raw['datum_longitude'], 'elevation':0}
    sw_gps = {'latitude':bounds['south'], 'longitude':bounds['west'], 'elevation':0}
    distance_from_datum_to_sw = get_datumrelative_metric_from_gps(datum_gps, sw_gps)
    print('\n\ngenerate offset:')
    print('ori sw gps', sw_gps)
    print(' datum gps', datum_gps)
    print('datum 2 sw', distance_from_datum_to_sw)
    #print('rough est.', {'x':-9, 'y':-52, 'z':0})
    #exit()
    # Populate the map.yaml
    mapyaml_raw = MetricTemplates.mapyaml % (image, resolution, distance_from_datum_to_sw['x'], distance_from_datum_to_sw['y'])
    mapyaml_path = os.path.join(args['src'], 'config', 'metric', 'map', 'map_autogen.yaml')
    with open(mapyaml_path, 'w') as f:
        f.write(mapyaml_raw)

    # Calculate width and height of region (sw to ne)
    sw_gps = {'latitude':bounds['south'], 'longitude':bounds['west'], 'elevation':0}
    ne_gps = {'latitude':bounds['north'], 'longitude':bounds['east'], 'elevation':0}
    region_dimensions = get_datumrelative_metric_from_gps(sw_gps, ne_gps)
    h = int(abs(region_dimensions['x'])/float(resolution))
    w = int(abs(region_dimensions['y'])/float(resolution))

    # Generate the map image
    img = Image.new("L", (h, w), (255))
    img_path = os.path.join(args['src'], 'config', 'metric', 'map', image)
    img.save(img_path, "PNG")


def main(args=None):
    e = 'environment_template'
    src = '/'.join(get_package_prefix(e).split('/')[:-2]) + f'/src/{e}'
    location_name = 'riseholme_polytunnel'
    run({'src': src, 'location_name':location_name})

if __name__ == '__main__':
    main()
