import sys, os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
import yaml
from pprint import pprint

from environment_common.convertors.templating.kml import KmlTemplates


def run(args=None):
    datum_path = os.path.join(args['src'], 'config', 'location', 'datum.yaml')
    if not os.path.isfile(datum_path):
        datum_path = os.path.join(args['src'], 'config', 'location', 'datum_autogen.yaml')
    with open(datum_path) as f:
        data = f.read()
        datum = yaml.safe_load(data)
    colour_args = {'line_col':'ff2f2fd3', 'line_width':'4', 'fill_col':'c02f2fd3'}

    place_id = args['location_name']
    lat = datum['datum_latitude']
    lon = datum['datum_longitude']
    alt = 0

    fency = datum['gnss_fence']['gnss_fence_coords']
    fency += [datum['gnss_fence']['gnss_fence_coords'][0]]
    fence = ' '.join([f'{g[1]},{g[0]},{alt}' for g in fency])

    line_col = colour_args['line_col']
    line_width = colour_args['line_width']
    fill_col = colour_args['fill_col']

    kml = KmlTemplates.opening % f"{place_id}_kml_autogen"
    kml += KmlTemplates.styler("a",  line_col, line_width, fill_col)
    kml += KmlTemplates.placemark % ("datum_fence_placemark", place_id, place_id, lon, lat, alt, "a", fence)
    kml += KmlTemplates.closing

    kml_path = os.path.join(args['src'], 'config', 'location', 'fence_autogen.kml')
    with open(kml_path, 'w') as f:
        f.write(kml)

def main(args=None):
    e = 'environment_template'
    src = '/'.join(get_package_prefix(e).split('/')[:-2]) + f'/src/{e}'
    location_name = 'riseholme_polytunnel'
    run({'src': src, 'location_name':location_name})

if __name__ == '__main__':
    main()
