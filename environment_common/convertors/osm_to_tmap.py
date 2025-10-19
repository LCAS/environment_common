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
    nodes = dict()
    for way in tree['way'].values():
        wtd = way['tag_dict']

        # Only render highways into the tmap, skip otherwise
        if 'highway' not in wtd.keys(): continue

        # Specify the highway name
        name = (wtd['name'] + '_' + way['id']) if 'name' in wtd else 'highway_' + way['id']

        # Determine if way is a polyline or polygon
        polytype = 'polygon' if way['coords'][0] == way['coords'][-1] else 'polyline'

        # Formulate node list
        for i,c in enumerate(way['coords']):
            # Determine pose
            y, x = calculate_distance_changes(lat, lon, float(c['lat']), float(c['lon']))

            # Determine connections to other nodes in list/loop
            connections, wc = [], way['coords']
            if i == 0:
                prev = wc[-1] if polytype == 'polygon' else None
                next = wc[i+1]
            elif i == len(way['coords'])-1:
                prev = wc[i-1]
                next = wc[0] if polytype == 'polygon' else None
            else:
                prev = wc[i-1]
                next = wc[i+1]
            if next: connections += [next]
            if prev: connections += [prev]

            # Construct node details dictionary
            nodes[name+"_"+c['ref']] = {'x':x, 'y':y,
                                        'lat':float(c['lat']), 'lon':float(c['lon']),
                                        'raw_connections':connections,
                                        'raw_name':c['ref'],
                                        'keep':False,
                                        'clear':False}


    # Plot each coord as a node for rendering
    for id, n in nodes.items():

        # Skip if node is already to be cleared
        if n['clear']: continue

        # Mark node to keep (first time seeing node)
        n['keep'] = True
        n['cleared_by'] = n['raw_name']
        x, y = n['x'], n['y']

        for id2, n2 in nodes.items():

            # Skip if node has been viewed already
            if n2['keep'] or n2['clear']: continue

            # If node 1 and node 2 are within close proximity, clear node 2
            x2, y2 = n2['x'], n2['y']
            if abs(x-x2) < 0.2 and abs(y-y2) < 0.2:

                # Mark node for clearance so it is not viewed anymore
                n2['clear'] = True
                n2['cleared_by'] = n['raw_name']

                # Copy the cleared nodes connections into the kept node's details
                n['raw_connections'] += n2['raw_connections']


    # Remove nodes marked for clearance
    kept = [n for n in nodes.values() if not n['clear']]

    # Assign WayPoint names for each node
    for i in range(len(kept)):
        kept[i]['name'] = f"WayPoint{i+1}"

    # Convert names of connections to the new WayPoint ids
    final_convertor = {n['raw_name']: n['name'] for n in kept}
    convertor = {n['raw_name']: final_convertor[n['cleared_by']] for n in nodes.values()}
    for k in kept:
        k['connections'] = set([convertor[c['ref']] if c['ref'] in convertor else c for c in k['raw_connections']])
        del k['raw_connections'], k['raw_name']


    # Plot each coord as a node for rendering
    for n in kept:
        node.update({'name':n['name'], 'x':n['x'], 'y':n['y']})
        tmap += TMapTemplates.node.format(**node)
        if not n['connections']:
            tmap += TMapTemplates.edges_empty
        else:
            tmap += TMapTemplates.edges_start
            for c in n['connections']:
                if c == n['name']: continue
                connection_name = n['name'] #+"_"+c['ref']
                edge.update({'name':n['name'], 'name2':c})
                tmap += TMapTemplates.edges.format(**edge)

    # Save tmap file
    tmap_path = os.path.join(args['src'], 'config', 'topological', 'network_autogen.tmap2.yaml')
    with open(tmap_path, 'w') as f:
        f.write(tmap)


def main(args=None):
    e = 'environment_template'
    src = '/'.join(get_package_prefix(e).split('/')[:-2]) + f'/src/{e}'
    location_name = 'r_gep'
    args = {'src': src, 'location_name':location_name}
    run(args)

if __name__ == '__main__':
    main()
