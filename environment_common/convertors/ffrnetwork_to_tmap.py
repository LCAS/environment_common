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
    ffrnetwork_path = os.path.join(args['src'], 'config', 'topological', 'network.ffr.yaml')
    if not os.path.isfile(ffrnetwork_path):
        ffrnetwork_path = os.path.join(args['src'], 'config', 'topological', 'network_autogen.ffr.yaml')
    root = getroot(ffrnetwork_path)

    # Specify default properties
    node = {'location':place_id, 'vert': 'vert1', 'restrictions':'robot', 'connections':None}
    edge = {'action':'move_base', 'action_type':'nav2_msgs/MoveBaseGoal', 'restrictions':'robot'}

    # Create tmap from templates
    tmap = TMapTemplates.vert_sample
    tmap += TMapTemplates.opening.format(**{'gen_time':0, 'location':place_id})

    # Create dictionary for nodes
    nodes = dict()
    for n in root['internal_points']:
        n_pos = f"x{n['x']}y{n['y']}"
        nodes[n_pos] = n

    # Add each node to the tmap object
    edges = dict()
    for e in root['network_lines']:
        # Create uuids
        e_start = f"x{e['start']['x']}y{e['start']['y']}"
        e_end = f"x{e['end']['x']}y{e['end']['y']}"

        # Add starts to edge list
        if e_start not in edges: edges[e_start] = []
        edges[e_start].append(e)

        if e_end not in edges: edges[e_end] = []
        edges[e_end].append(e)


    # Add each node to the tmap object
    for n in root['internal_points']:
        node.update({'name':n['name'], 'x':n['x'], 'y':n['y']})
        tmap += TMapTemplates.node.format(**node)

        n_pos = f"x{n['x']}y{n['y']}"
        if not edges[n_pos]:
            tmap += TMapTemplates.edges_empty
        else:
            tmap += TMapTemplates.edges_start
            for e in edges[n_pos]:
                e_end = f"x{e['end']['x']}y{e['end']['y']}"
                connection = nodes[e_end]
                edge.update({'name':n['name'], 'name2':connection['name']})
                tmap += TMapTemplates.edges.format(**edge)

    # Save tmap file
    tmap_path = os.path.join(args['src'], 'config', 'topological', 'network_autogen.tmap2.yaml')
    with open(tmap_path, 'w') as f:
        f.write(tmap)

if __name__ == '__main__':
    main()
