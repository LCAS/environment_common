import sys, os
import yaml
from ament_index_python.packages import get_package_share_directory, get_package_prefix
import xml.etree.ElementTree as ET
from pprint import pprint

from shapely.geometry import Point
from shapely.geometry.polygon import Polygon

from environment_common.convertors.templating.tmap import TMapTemplates
from environment_common.convertors.tools.gps import calculate_displacement, calculate_distance_changes
#from environment_common.convertors.tools.kml import KlmRead


class KlmRead:

    @classmethod
    def polyline_to_dictlist(cls, polyline_str, name, tagtype, circuit=False):
        pls = polyline_str.replace('\n','').replace('\t','').split(' ')[:-1]
        coords = [g.split(',') for g in pls]
        dictlist = [{'longitude':round(float(gnss[0]),6),
                     'latitude': round(float(gnss[1]),6),
                     'elevation':round(float(gnss[2]),6),
                     'name': f"{name}"} for i,gnss in enumerate(coords)]
        return dictlist

    @classmethod
    def get_coords(cls, root):
        details = dict()
        for i, base in enumerate(root[0]):
            if 'Placemark' in base.tag:
                name, coords = '', ''
                tags = {field.tag.split('}')[-1]:field for field in base}
                name = tags['name'].text
                if 'LineString' in tags: continue
                elif 'Point' in tags: continue
                elif 'Polygon' in tags:
                    coords = tags['Polygon'][0][0][0].text
                    tagtype = 'Polygon'
                details[name] = cls.polyline_to_dictlist(coords, name, tagtype)
        return details


def run(args=None):

    # Load datum
    datum_path = os.path.join(args['src'], 'config', 'location', 'datum.yaml')
    if not os.path.isfile(datum_path):
        datum_path = os.path.join(args['src'], 'config', 'location', 'datum_autogen.yaml')
    with open(datum_path) as f:
        data = f.read()
        datum = yaml.safe_load(data)

    # Load tmap
    tmap_path = os.path.join(args['src'], 'config', 'topological', 'network.tmap2.yaml')
    if not os.path.isfile(tmap_path):
        tmap_path = os.path.join(args['src'], 'config', 'topological', 'network_autogen.tmap2.yaml')
    with open(tmap_path) as f:
        data = f.read()
        tmap = yaml.safe_load(data)

    place_id = args['location_name']
    lat = datum['datum_latitude']
    lon = datum['datum_longitude']

    # Select the kml file containing the regions
    ENV = get_package_share_directory('environment_template')
    kml_path = os.path.join(args['src'], 'config', 'topological', 'actions.kml')
    kml_path = os.path.join(args['src'], 'config', 'topological', 'r_nr_man_tmap_regions.kml')
    while True:
        print(f'Applying actions specified in `{kml_path}`. \nTo use a different KML, place the `.kml` file in: `environment_template/config/topological/` \n\n\nEnter the name of the file below, or press [ENTER] to continue:')
        inp = input('>> environment_template/config/topological/')
        print('\n')
        print(inp)
        if inp != '':
            if not inp.endswith('.kml'):
                print('Ensure you have included the correct file extension of: `.kml`\n\n')
            else:
                kml_path = os.path.join(args['src'], 'config', 'topological', inp)
                break
        else:
            break
    locations = dict()
    tree = ET.parse(kml_path)
    root = tree.getroot()

    # Extract the polygon regions
    placemarks = KlmRead.get_coords(root)
    action_polygons = []
    restriction_polygons = []
    boundary_polygons = []

    # Convert polygons to datum
    for name, place in placemarks.items():

        # Convert gps points of polygon to netric relative to datum
        for c in place:
           c['y'], c['x'] = calculate_distance_changes(lat, lon, c['latitude'], c['longitude'])
        polygon = [name, Polygon([(c['x'],c['y']) for c in place])]

        # If placemark marks actions
        ## Action polygons should be labelled such like 'action/move_base/move_base_msgs/MoveBaseGoal'
        if name.startswith('action/'):
            print(f'Polygon of type action:      {name}')
            action_polygons += [polygon]

        # If placemark marks restrictions
        ## Restriction polygons should be labelled such like 'restriction/robot_short & robot_hunter'
        elif name.startswith('restrictions/'):
            print(f'Polygon of type restriction: {name}')
            restriction_polygons += [polygon]

        # If placemark mark map boundaries
        ## Boundary polygons should be labelled such like 'exits/12'
        elif name.startswith('exits/'):
            print(f'Polygon of type boundary:    {name}')
            boundary_polygons += [polygon]

        # If placemark does not match known format
        else:
            print(f'Polygon does not match any pattern: {name}')


    # Compile edge list and make nodes accessible by name
    node_dict = dict()
    edge_list = []
    for n in tmap['nodes']:
        node_dict[n['node']['name']] = n['node']
        for e in n['node']['edges']:
            edge_list += [[n['node']['name'], e]]

    # Apply actions to edges
    for e in edge_list:
        n_from, n_to = node_dict[e[0]], node_dict[e[1]['node']]
        P1 = Point(n_from['pose']['position']['x'], n_to['pose']['position']['y'])
        P2 = Point(n_to['pose']['position']['x'], n_to['pose']['position']['y'])

        # Actions are applied to an edge if both nodes of an edge are contained within a Polygon.
        for polygon in action_polygons:
            act, poly = polygon[0], polygon[1]
            if poly.contains(P1) and poly.contains(P2):
                print(f"node actioned as {act.split('/')[1]}: {act.split('/')[2]+'/'+act.split('/')[3]}")
                e[1]['action'] = act.split('/')[1]
                e[1]['action_type'] = act.split('/')[2]+"/"+act.split('/')[3]

    # Apply restrictions to nodes
    for n in tmap['nodes']:
        P1 = Point(n['node']['pose']['position']['x'], n['node']['pose']['position']['y'])

        # Restrictions are applied to nodes first
        for polygon in restriction_polygons:
            rest, poly = polygon[0], polygon[1]
            if poly.contains(P1):
                print(f"node restricted with {rest[12:]}: {n['node']['name']}")
                n['restrictions'] = rest[12:]

    # Apply boundary marks to nodes
    for n in tmap['nodes']:
        P1 = Point(n['node']['pose']['position']['x'], n['node']['pose']['position']['y'])

        # Restrictions are applied to nodes first
        for polygon in boundary_polygons:
            bound, poly = polygon[0], polygon[1]
            if poly.contains(P1):
                print(f"node marked as boundary by {bound}: {n['node']['name']}")
                n['node']['properties']['boundarys'] = True

    # Apply restrictions to edges
    #for e in tmap['nodes']['']:
    #    #if both edges have the same restriction, we should add it to the edge too?
    #    #but what if the restrictions are different?


    tmap_path = os.path.join(args['src'], 'config', 'topological', 'network_autogen.tmap2.yaml')
    with open(tmap_path, 'w') as f:
        f.write(yaml.dump(tmap))



def main(args=None):
    print('This script is used to apply restrictions and actions to edges and nodes across a topological map.')
    print('This is a roudementary system and should be used with a grain of salt.')
    print('Navigation actions are applied to edges without problem.')
    print('Restrictions are applied only to nodes, edges require manual intervention, while robots may be able to traverse across 2 nodes, it does not mean they can traverse between them.')
    print('ohoh ohohoohoh we could apply a filter which uses a condition of if both nodes have same restrictions AND are in the same polygon region????')
    e = 'environment_template'
    src = '/'.join(get_package_prefix(e).split('/')[:-2]) + f'/src/{e}'
    location_name = os.getenv('FIELD_NAME')
    if not location_name:
        print('missing ENVVAR FIELD_NAME, not continuing')
        return
    args = {'src': src, 'location_name':location_name}
    run(args)

if __name__ == '__main__':
    main()
