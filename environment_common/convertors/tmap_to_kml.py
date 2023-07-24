import sys, os
from ament_index_python.packages import get_package_share_directory, get_package_prefix

import yaml
from pprint import pprint

from environment_common.convertors.templating.kml import KmlTemplates, KmlDraw
from environment_common.convertors.tools.gps import calculate_displacement, calculate_coordinates


def run(args=None):
    line_col = args['line_col']
    line_width = args['line_width']
    fill_col = args['fill_col']

    datum_path = os.path.join(args['src'], 'config', 'location', 'datum.yaml')
    if not os.path.isfile(datum_path):
        datum_path = os.path.join(args['src'], 'config', 'location', 'datum_autogen.yaml')
    with open(datum_path) as f:
        data = f.read()
        datum = yaml.safe_load(data)

    place_id = args['location_name']
    lat = datum['datum_latitude']
    lon = datum['datum_longitude']

    tmap_path = os.path.join(args['src'], 'config', 'topological', 'network.tmap2')
    if not os.path.isfile(tmap_path):
        tmap_path = os.path.join(args['src'], 'config', 'topological', 'network_autogen.tmap2')
    with open(tmap_path) as f:
        data = f.read()
        tmap = yaml.safe_load(data)
    print(tmap_path)

    kml = KmlTemplates.opening % f"{args['location_name']}_auto_tmap"
    kml += KmlTemplates.styler('a', args['line_col'], args['line_width'], args['fill_col'])

    points = [{'name': n['node']['name'],
               'x': n['node']['pose']['position']['x'],
               'y': n['node']['pose']['position']['y'],
               'z': n['node']['pose']['position']['z'],
               'connections': [e['node'] for e in n['node']['edges']]} for n in tmap['nodes']]
    for p in points:
        p['latitude'], p['longitude'] = calculate_coordinates(lat, lon, p['y'], p['x'])
        p['elevation'] = p['z']

    kml += KmlDraw.draw_nodes(gnss_dict_list=points, shape='diamond', style='a', size=args['shape_size'])
    kml += KmlDraw.draw_edges(gnss_dict_list=points, style='a')
    kml += KmlTemplates.closing

    kml_path = os.path.join(args['src'], 'config', 'topological', 'tmap2_autogen.kml')
    with open(kml_path, 'w') as f:
        f.write(kml)

    #gdrive_path = os.path.join(os.getenv('GDRIVE_PATH'), 'Google Earth', 'kml', 'tmap2_autogen.kml')
    #with open(gdrive_path, 'w') as f:
    #    f.write(kml)

def main(args=None):
    e = 'environment_template'
    src = '/'.join(get_package_prefix(e).split('/')[:-2]) + f'/src/{e}'
    location_name = 'riseholme_polytunnel'
    args = {'src': src, 'location_name':location_name, 'line_col':'ff2f2fd3', 'line_width':'4', 'fill_col':'c02f2fd3', 'shape_size':0.000005}
    run(args)

if __name__ == '__main__':
    main()
