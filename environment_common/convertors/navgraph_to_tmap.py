import sys, os
from ament_index_python.packages import get_package_share_directory, get_package_prefix

import yaml
from pprint import pprint

from environment_common.convertors.templating.tmap import TMapTemplates
from environment_common.convertors.tools.gps import calculate_distance_changes
from environment_common.convertors.tools.navgraph import getroot


def run(args=None):
    place_id = args['location_name']

    # Load the osm.xml file
    navgraph_path = os.path.join(args['src'], 'config', 'topological', 'navgraph.yaml')
    if not os.path.isfile(navgraph_path):
        navgraph_path = os.path.join(args['src'], 'config', 'topological', 'navgraph_autogen.yaml')
    root = getroot(navgraph_path)

    # Specify default properties
    node = {'location':place_id, 'vert': 'vert1', 'restrictions':'robot', 'connections':None}
    edge = {'action':'move_base', 'action_type':'move_base_msgs/MoveBaseGoal', 'restrictions':'robot'}

    # Create tmap from templates
    tmap = TMapTemplates.vert_sample
    tmap += TMapTemplates.opening.format(**{'gen_time':0, 'location':place_id})

    # Add edge connections to each node
    for n in root['nodes']:
        n['connections'] = []
        for c in root['connections']:

            # If no included constructor, treat as bidirectional
            if len(c) == 2:
                if n['name'] == c[0]: n['connections'] += [c[1]]
                if n['name'] == c[1]: n['connections'] += [c[0]]
                continue

            # If the edge is directed, add both connections
            if c[0] == '!dir':
                if n['name'] == c[1]: n['connections'] += [c[2]]
                continue

            # I dont have the slightest idea on how to integrate this
            elif c[0] == '!split-intersection':
                pass

    # Add each node to the tmap object
    for n in root['nodes']:
        node.update({'name':n['name'], 'x':n['pos'][0], 'y':n['pos'][1]})
        tmap += TMapTemplates.node.format(**node)
        if not n['connections']:
            tmap += TMapTemplates.edges_empty
        else:
            tmap += TMapTemplates.edges_start
            for c in n['connections']:
                print(c)
                #if c == n['name']: continue
                connection_name = n['name']
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
