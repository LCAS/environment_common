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

    print('If any properties such as actions, restrictions, exits of tollerance enforcements are to be defined, apply these first.')

    # Select the kml file containing the regions
    ENV = get_package_share_directory('environment_template')
    kml_path = os.path.join(args['src'], 'config', 'topological', 'copypastes.kml')
    while True:
        print(f'Applying copypastes specified in `{kml_path}`. \nTo use a different KML, place the `.kml` file in: `environment_template/config/topological/` \n\n\nEnter the name of the file below, or press [ENTER] to continue:')
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
    copy_polygons = []

    # Convert polygons to datum
    for name, place in placemarks.items():

        # Convert gps points of polygon to netric relative to datum
        for c in place:
           c['y'], c['x'] = calculate_distance_changes(lat, lon, c['latitude'], c['longitude'])
        polygon = [name, Polygon([(c['x'],c['y']) for c in place])]

        # If placemark marks copy action
        ## Copy polygons should be labelled such like 'copy/uuid'
        if name.startswith('copy/'):
            print(f'Polygon of type copy:      {name}')
            paste_polygons += [polygon]

        # If placemark marks paste action
        ## Copy polygons should be labelled such like 'paste/copy_uuid'
        elif name.startswith('paste/'):
            print(f'Polygon of type paste:      {name}')
            paste_polygons += [polygon]

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



    total_nodes = len(tmap['nodes'])
    extra_nodes = []

    # Actions are applied to an edge if both nodes of an edge are contained within a Polygon.
    for polygon in paste_polygons:
        paste, poly = polygon[0], polygon[1]

        # Identify priority level
        copy_uuid = paste.split('/')[1]
        copy_poly = copy_polygons[copy_uuid]
        copy_offset = copy_poly[1].pose[0] - paste_poly[1].pose[0]

        # Apply copypaste to nodes
        for n in tmap['nodes']:
            P1 = Point(n['node']['pose']['position']['x'], n['node']['pose']['position']['y'])

            if copy_poly.contains(P1):
                p1 = copy.deepcopy(P1)

                # Specify new Node ID and save reference
                node_id = total_nodes + len(extra_nodes) + 1
                new_nodes[n['node']['name']] = node_id
                extra_nodes = node(p1.pose+copy_offset)

        # Rename edges in extra_nodes
        for n in extra_nodes:
            old_name = n['node']['name']
            new_name = new_nodes[n['node']['name']]

            # Rename edges containing old_name, with new_name
            for e in p1['node']['edges']:

                # Update edge_id to use the new source
                old_target = e['node']
                e['edge_id'] = new_name + '_' + old_target

                # Update edge_id to use the new target
                if old_target in new_nodes:
                    new_target = new_nodes[old_target]
                    e['node'] = new_target
                    e['edge_id'] = new_name + '_' + new_target

                # Handle if the target is outside the copy region
                else:
                    print('Execute handling for if target is beyond the copy region')
                    print('perhaps we should do this by searching for if any other nodes')
                    print('are within the offset proximity of the taregt node')
                    print('if so, we can latch to a new target')
                    print('otherwise we have to delete the edge i guess')

                    # Search for nodes in region around target? but is the target ofset here?
                    x, y = n['node']['pose']['position']['x'], n['node']['pose']['position']['y']
                    polygon = Polygon( [(x+0,y+0), (x+n,y+0), (x+n,y+n), (x+0,y+n)] )

                    for n2 in tmap['nodes']:
                        P1 = Point(n2['node']['pose']['position']['x'], n2['node']['pose']['position']['y'])


            # Add the node to the tmap
            tmap['nodes'] += [n]





    # Apply restrictions to nodes
    for n in tmap['nodes']:
        P1 = Point(n['node']['pose']['position']['x'], n['node']['pose']['position']['y'])

        # Actions are applied to an edge if both nodes of an edge are contained within a Polygon.
        for polygon in paste_polygons:
            paste, poly = polygon[0], polygon[1]

            if poly.contains(P1):

                # Identify priority level
                copy_uuid = paste.split('/')[1]

                # Apply restriction to node if this restriction is a higher or equal priority
                if ('restriction_priority' not in n) or (restriction_priority >= n['restriction_priority']):
                    print(f"node restricted with priority {restriction_priority} as {restriction}: {n['node']['name']}")
                    n['restrictions'] = restriction
                    n['restrictions_priority'] = restrictions_priority




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
