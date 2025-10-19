import sys, os
from ament_index_python.packages import get_package_share_directory, get_package_prefix

import yaml
from pprint import pprint

from environment_common.convertors.templating.kml import KmlTemplates, KmlDraw
from environment_common.convertors.tools.xml import getroot
from environment_common.convertors.tools.kml import *
from environment_common.convertors.tools.osm import *


def run(args=None):

    # Load the osm.xml file
    osm_path = os.path.join(args['src'], 'config', 'topological', 'osm.xml')
    if not os.path.isfile(osm_path):
        osm_path = os.path.join(args['src'], 'config', 'topological', 'osm_autogen.xml')
    root = getroot(osm_path)
    tree = gettree(root)


    # Create image kml from templates
    kml = KmlTemplates.opening % f"{args['location_name']}_auto_metric"
    kml += KmlTemplates.styler('a', line_col='ffff0000', line_width='8')
    kml += KmlTemplates.styler('b', line_col='ff0000ff', fill_col='aa0000aa')
    kml += KmlTemplates.styler('c', line_col='ffff00ff', fill_col='aaaa00aa')
    kml += KmlTemplates.styler('d', line_col='ff00ffff', fill_col='aa00aaaa')
    kml += KmlTemplates.styler('e', line_col='ffff00ff', fill_col='aaaa00aa')

    #for node in tree['node'].values():
    #    kml += KmlTemplates.point % (node['id'], node['id'], node['lon'], node['lat'])
    for way in tree['way'].values():
        coords = [[c['lon'],c['lat'],0] for c in way['coords']]
        id, wtd = way['id'], way['tag_dict']
        if 'highway' in wtd.keys():
            name = (wtd['name'] + '_' + id) if 'name' in wtd else 'highway_' + id
            kml += KmlDraw.draw_open_line(id, name, coords, style='a')

        elif 'building' in wtd.keys():
            name = 'building_' + id
            kml += KmlDraw.draw_polygon(id, name, coords, style='b')

        elif 'man_made' in wtd.keys():
            name = wtd['man_made'] +'_' + id
            kml += KmlDraw.draw_open_line(id, name, coords, style='c')

        elif 'leisure' in wtd.keys():
            name = wtd['leisure'] + '_' + id
            kml += KmlDraw.draw_polygon(id, name, coords, style='d')

        elif 'natural' in wtd.keys():
            name = (wtd['natural'] + '_' + id) if 'natural' in wtd else 'natural_' + id
            kml += KmlDraw.draw_open_line(id, name, coords, style='e')

        else:
            print(way['id'], way['tag_dict'])

    kml += KmlTemplates.closing


    # Save kml file
    kml_path = os.path.join(args['src'], 'config', 'topological', 'osm.kml')
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
