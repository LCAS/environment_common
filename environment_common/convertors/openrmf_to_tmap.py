import sys, os
from ament_index_python.packages import get_package_share_directory, get_package_prefix

import yaml
from pprint import pprint

from environment_common.convertors.templating.tmap import TMapTemplates

def run(args=None):

    place_id = args['location_name']

    rmf_path = os.path.join(args['src'], 'config', 'topological', 'rmf.building.yaml')
    with open(rmf_path) as f:
        data = f.read()
        rmf = yaml.safe_load(data)


    # Construct TMap
    tmap = TMapTemplates.vert_start
    tmap += TMapTemplates.vert_ring('vert1m', 1)

    tmap += TMapTemplates.opening.format(**{'gen_time':0, 'location':place_id})

    # Include Nodes and edges
    node = {'location':place_id, 'vert': 'vert1m', 'restrictions':'robot', 'connections':None}
    edge = {'action':'move_base', 'action_type':'move_base_msgs/MoveBaseGoal', 'restrictions':'robot'}

    for k, level in rmf['levels'].items():
        for i, n in enumerate(level['vertices']):
            x, y = n[0], n[1]
            name = f"{k}_{i}"
            node.update({'name':name, 'x':x, 'y':y})
            tmap += TMapTemplates.node.format(**node)
            tmap += TMapTemplates.edges_empty

    # Save TMap File
    tmap_path = os.path.join(args['src'], 'config', 'topological', 'network_autogen.tmap2.yaml')
    with open(tmap_path, 'w') as f:
        f.write(tmap)



    return rmf

def main(args=None):
    e = 'environment_template'
    src = '/'.join(get_package_prefix(e).split('/')[:-2]) + f'/src/{e}'
    location_name = 'riseholme_polytunnel'
    args = {'src': src, 'location_name':location_name}
    return run(args)

if __name__ == '__main__':
    rmf = main()
