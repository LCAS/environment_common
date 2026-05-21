import sys, os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from pprint import pprint

from environment_common.convertors.templating.datum import DatumTemplates
from environment_common.convertors.tools.kml import polyline_to_list, get_coords

from environment_common.convertors.tools.files import load_kml_root, load_yaml_root
from environment_common.convertors.kml_to_tmap import group_similar_coords, calculate_distance_changes, format_tmap


def run(args=None):
    place_id = args['location_name']
    ENV = get_package_share_directory('environment_template')

    # 1) load datum file so we can convert to metric space
    print('\n\n1)')
    datum_path = os.path.join(args['src'], 'config', 'location', 'datum.yaml')
    datum = load_yaml_root(datum_path)
    lat = datum['datum_latitude']
    lon = datum['datum_longitude']

    # 2) load kml file containing network points
    print('\n\n2)')
    kml_path = os.path.join(args['src'], 'config', 'topological', f'network.kml')
    root1 = load_kml_root(kml_path)
    coords1 = get_coords(root1)

    # 3) load kml file containing network additions
    print('\n\n3)')
    kml_path = os.path.join(args['src'], 'config', 'topological', f'network_additions.kml')
    root2 = load_kml_root(kml_path)
    coords2 = get_coords(root2)


    # 4) flatten down the points into a single list and merge points which are in a similar position
    print('\n\n4)')
    allpoints = sum(coords1.values(),[]) + sum(coords2.values(),[])
    lesspoints = group_similar_coords(allpoints)
    print(lesspoints)
    for l in lesspoints:
        l['y'], l['x'] = calculate_distance_changes(lat, lon, l['latitude'], l['longitude'])
    [print(f"{l['name']} ({l['longitude']}:{l['latitude']})  -  {l['connections']}") for l in lesspoints]
    print(f"Reduced {len(allpoints)} raw points down to {len(lesspoints)}")

    # 5) begin formatting the tmap dictionary
    print('\n\n5)')
    tmap = format_tmap(lesspoints, place_id)

    # 6) write the tmap file
    print('\n\n6)')
    tmap_path = os.path.join(args['src'], 'config', 'topological', 'network_autogen.tmap2.yaml')
    with open(tmap_path, 'w') as f:
        f.write(tmap)

    import yaml
    print(len(yaml.safe_load(tmap)['nodes']))

def main(args=None):
    e = 'environment_template'
    src = '/'.join(get_package_prefix(e).split('/')[:-2]) + f'/src/{e}'
    args = {'src': src, 'location_name':'bob'}
    run(args)

if __name__ == '__main__':
    main()


"""
(network.tmap2) --> python3 tmap_to_kml.py --> (network_ag.kml)

(network_ag.kml, network_additions.kml) --> python3 kml_merge.py --> (network_ag.tmap2)

(network_ag.tmap2) --> python3 tmap_to_kml.py --> (network_ag.kml)
"""
