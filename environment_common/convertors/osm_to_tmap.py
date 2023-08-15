import sys, os
from ament_index_python.packages import get_package_share_directory, get_package_prefix

import yaml
from pprint import pprint

from environment_common.convertors.templating.tmap import TMapTemplates
from environment_common.convertors.tools.gps import calculate_distance_changes
from environment_common.convertors.tools.xml import getroot
from environment_common.convertors.tools.osm import *


def run(args=None):

    # Load the datum.yaml file
    datum_path = os.path.join(args['src'], 'config', 'location', 'datum.yaml')
    if not os.path.isfile(datum_path):
        datum_path = os.path.join(args['src'], 'config', 'location', 'datum_autogen.yaml')
    with open(datum_path) as f:
        data = f.read()
        datum = yaml.safe_load(data)

    place_id = args['location_name']
    lat = datum['datum_latitude']
    lon = datum['datum_longitude']


    # Load the osm.xml file
    osm_path = os.path.join(args['src'], 'config', 'topological', 'osm.xml')
    if not os.path.isfile(osm_path):
        osm_path = os.path.join(args['src'], 'config', 'topological', 'osm_autogen.xml')
    root = getroot(osm_path)
    tree = gettree(root)

    # Specify default properties
    node = {'location':place_id, 'vert': 'vert1', 'restrictions':'robot', 'connections':None}
    edge = {'action':'move_base', 'action_type':'move_base_msgs/MoveBaseGoal', 'restrictions':'robot'}

    # Create tmap from templates
    tmap = TMapTemplates.vert_sample
    tmap += TMapTemplates.opening.format(**{'gen_time':0, 'location':place_id})

    # Loop through each way object
    for way in tree['way'].values():
        wtd = way['tag_dict']

        # Only render highways into the tmap, skip otherwise
        if 'highway' not in wtd.keys(): continue

        # Specify the highway name
        name = (wtd['name'] + '_' + way['id']) if 'name' in wtd else 'highway_' + way['id']

        # Formulate node list
        nodes = dict()
        for c in way['coords']:
            y, x = calculate_distance_changes(lat, lon, float(c['lat']), float(c['lon']))
            nodes[name+"_"+c['ref']] = {'x':x, 'y':y, 'lat':float(c['lat']), 'lon':float(c['lon'])}

        # Plot each coord as a node for rendering
        for id, n in nodes.items():
            node.update({'name':id, 'x':n['x'], 'y':n['y']})
            tmap += TMapTemplates.node.format(**node)
            if node['connections']:
                tmap += TMapTemplates.edges_start
                for c in node['connections']:
                    edge.update({'name':l['name'], 'name2':c})
                    tmap += TMapTemplates.edges.format(**edge)


    # Save tmap file
    tmap_path = os.path.join(args['src'], 'config', 'topological', 'network_autogen.tmap2.yaml')
    with open(tmap_path, 'w') as f:
        f.write(tmap)

    # Save tmap file to google drive
    if os.getenv('GDRIVE_PATH', ""):
        gdrive_path = os.path.join(os.getenv('GDRIVE_PATH'), 'Google Earth', 'kml', 'network_autogen.tmap2.yaml')
        with open(gdrive_path, 'w') as f:
            f.write(kml)


def main(args=None):
    e = 'environment_template'
    src = '/'.join(get_package_prefix(e).split('/')[:-2]) + f'/src/{e}'
    location_name = 'r_gep'
    args = {'src': src, 'location_name':location_name}
    run(args)

if __name__ == '__main__':
    main()
