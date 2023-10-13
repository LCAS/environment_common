import sys, os
import yaml
from ament_index_python.packages import get_package_share_directory, get_package_prefix
import xml.etree.ElementTree as ET
from pprint import pprint

from shapely.geometry import Point
from shapely.geometry.polygon import Polygon

from environment_common.convertors.templating.tmap import TMapTemplates
from environment_common.convertors.tools.gps import calculate_displacement, calculate_distance_changes
from environment_common.convertors.tools.kml import KlmRead


class KlmRead:

    @classmethod
    def polyline_to_dictlist(cls, polyline_str, name, tagtype, circuit=False):
        pls = polyline_str.replace('\n','').replace('\t','').split(' ')[:-1]
        coords = [g.split(',') for g in pls]
        dictlist = [{'longitude':round(float(gnss[0]),6),
                     'latitude': round(float(gnss[1]),6),
                     'elevation':round(float(gnss[2]),6),
                     'raw_name': f"{name} {i}",
                     'raw_connections': []} for i,gnss in enumerate(coords)]

        if tagtype in ['LineString','Polygon']:
            for i in range(0,len(dictlist)-1):
                dictlist[i]['raw_connections'] += [dictlist[i+1]['raw_name']]
            for i in range(1,len(dictlist)):
                dictlist[i]['raw_connections'] += [dictlist[i-1]['raw_name']]

        if tagtype == 'Polygon':
            dictlist[0]['raw_connections'] += [dictlist[-1]['raw_name']]
            dictlist[-1]['raw_connections'] += [dictlist[0]['raw_name']]

        return dictlist


    @classmethod
    def get_coords(cls, root):
        details = dict()
        for i, base in enumerate(root[0]):
            if 'Placemark' in base.tag:
                name, coords = '', ''
                tags = {field.tag.split('}')[-1]:field for field in base}
                name = tags['name'].text
                if 'LineString' in tags:
                    coords = tags['LineString'][0].text
                    tagtype = 'LineString'
                elif 'Polygon' in tags:
                    coords = tags['Polygon'][0][0][0].text
                    tagtype = 'Polygon'
                elif 'Point' in tags:
                    coords = tags['Point'][0].text
                    tagtype = 'Point'
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
    tmap_path = os.path.join(args['src'], 'config', 'topological', 'network.tmap.yaml')
    if not os.path.isfile(datum_path):
        tmap_path = os.path.join(args['src'], 'config', 'topological', 'network_autogen.tmap.yaml')
    with open(tmap_path) as f:
        data = f.read()
        tmap = yaml.safe_load(data)

    place_id = args['location_name']
    lat = datum['datum_latitude']
    lon = datum['datum_longitude']

    # Select the kml file containing the regions
    ENV = get_package_share_directory('environment_template')
    kml_path = os.path.join(args['src'], 'config', 'topological', 'actions.kml')
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
    locations = dict()
    tree = ET.parse(kml_path)
    root = tree.getroot()

    # Extract the polygon regions
    coords = KlmRead.get_coords(root)
    action_polygons = dict()
    restriction_polygons = dict()

    # Convert polygons to datum
    for place in placemark.values():

        # Convert gps points of polygon to netric relative to datum
        place['x'], place['y'] = [], []
        for c in place['coords']:
           c['y'], c['x'] = calculate_distance_changes(lat, lon, c['latitude'], c['longitude'])

        # If placemark is marked as action
        ## Action polygons should be labelled as 'action/move_base/move_base_msgs/MoveBaseGoal'
        if p['name'].startswith('action/'):
            action_polygons[place['name']] = Polygon([(c['x'],c['y']) for c in place['coords']])

        # If placemark is marked as restriction
        ## Action polygons should be labelled as 'restriction/robot_short & robot_hunter'
        if p['name'].startswith('restriction/'):
            restriction_polygons[place['name']] = Polygon([(c['x'],c['y') for c in place['coords']])


    # Compile edge list and make nodes accessible by name
    node_dict = dict()
    edge_list = []
    for n in tmap['nodes']:
        node_dict[n['node']['name']] = n
        for e in n['node']['edges']:
            edge_list += [n['node']['name'], e]

    # Apply actions to edges
    for e in edge_list:
        n_from, n_to = node_dict[e[0]], node_dict[e[1]['node']]
        P1 = Point(n_from['pose']['position']['x'], n_to['pose']['position']['y'])
        P2 = Point(n_to['pose']['position']['x'], n_to['pose']['position']['y'])

        # Actions are applied to an edge if both nodes of an edge are contained within a Polygon.
        for act, poly in action_polygon.items():
            if poly.contains(P1) and poly.contains(P2):
                n_from['action'] = act.split('/')[1]
                n_from['action_type'] = act.split('/')[2]+"/"+act.split('/')[3]

    # Apply restrictions to nodes
    for n in tmap['nodes']:
        P1 = Point(n['pose']['position']['x'], n['pose']['position']['y'])

        # Restrictions are applied to nodes first
        for rest, poly in restriction_polygon.items():
            if poly.contains(P1):
                n['restrictions'] = rest[12:]

    # Apply restrictions to edges
    #for e in tmap['nodes']['']:
    #    #if both edges have the same restriction, we should add it to the edge too?
    #    #but what if the restrictions are different?




def main(args=None):
    e = 'environment_template'
    src = '/'.join(get_package_prefix(e).split('/')[:-2]) + f'/src/{e}'
    location_name = 'riseholme_polytunnel'
    args = {'src': src, 'location_name':location_name}
    run(args)

if __name__ == '__main__':
    main()
