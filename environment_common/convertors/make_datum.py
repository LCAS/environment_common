import sys, os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
import xml.etree.ElementTree as ET
from pprint import pprint


template = """# Define site datum - best to be in the center of the field of interest
datum_latitude: &datum_latitude %s
datum_longitude: &datum_longitude %s


# Define GNSS Fence coordinates [lat, lon]
gnss_fence:
  gnss_fence_coords: %s

# Define map size (x east, y north) in meters from the datum
gmapping:
  xmin: %s
  xmax: %s
  ymin: %s
  ymax: %s

mapviz_initialize_origin:
  local_xy_origins:
    - {
    name: dont_change_this,
    latitude: *datum_latitude,
    longitude: *datum_longitude,
    altitude: 100.0,
    heading: 0.0
    }

navsat_transform_node:
  magnetic_declination_radians: 0.0
  yaw_offset: 0.0
  zero_altitude: true
  broadcast_utm_transform: false
  broadcast_utm_transform_as_parent_frame: false
  publish_filtered_gps: true
  use_odometry_yaw: false
  wait_for_datum: true
  datum: [ *datum_latitude, *datum_longitude, 0.0]

"""

def main(args=None):
    ENV = get_package_share_directory('environment_template')
    klm_path = os.path.join(args['src'], 'config', 'location', 'region.klm')
    locations = dict()

    tree = ET.parse(klm_path)
    root = tree.getroot()
    for i, base in enumerate(root[0]):
        if not base: continue
        if base[0].text == '\n\t\t\t': continue
        locations[base[0].text] = base
    pprint(locations)

    environment = locations[args['location_name']]
    latitude = environment[1][0].text
    longitude = environment[1][1].text

    fency = environment[3][0][0][0].text.replace('\n','').replace('\t','').split(' ')
    gnss_fence = [lle.split(',')[:2] for lle in fency][:-1]

    xmin = -100
    xmax = 100
    ymin = -100
    ymax = 100

    datum_path = os.path.join(args['src'], 'config', 'location', 'datum.yaml')
    print(datum_path)
    with open(datum_path, 'w') as f:
        txt = template%(latitude, longitude, str(gnss_fence).replace("'",""), xmin, xmax, ymin, ymax)
        f.write(txt)


if __name__ == '__main__':
    e = 'environment_template'
    src = '/'.join(get_package_prefix(e).split('/')[:-2]) + f'/src/{e}'
    location_name = 'riseholme_polytunnel'
    main({'src': src, 'location_name':location_name})
