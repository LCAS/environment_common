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
                elif 'Polygon' in tags: continue
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

    # Extract the waypoints
    placemarks = KlmRead.get_coords(root)
    name_rules = []
    name_points = []
    name_regions = []

    # Convert polygons to datum
    for name, place in placemarks.items():

        # Convert gps points of polygon to netric relative to datum
        for c in place:
           c['y'], c['x'] = calculate_distance_changes(lat, lon, c['latitude'], c['longitude'])
        if len(place) > 1:
            placemark = [name, Polygon([(c['x'],c['y']) for c in place])]
        else:
            placemark = [name, place]

        # If placemark is a region a naming convention should be applied
        ## Naming rule polygons should be labelled such like 'renaming_rule/r[*]-c[*]'
        if name.startswith('renaming_rule/'):
            print(f'Polygon of type rule: {name}')
            name_rules += [placemark]

        # If placemark is a node point
        ## Points should be labelled such like 'renaming_point/WayPoint121
        elif name.startswith('renaming_point/'):
            print(f'Point of type name:   {name}')
            name_points += [placemark]

        # If placemark is a polygon to surround a node
        ## Namey polygons should be labelled such like 'renaming_polygon/WayPoint120'
        elif name.startswith('renaming_polygon/'):
            print(f'Polygon of type name: {name}')
            name_regions += [placemark]

        # If placemark does not match known format
        else:
            print(f'Polygon does not match any pattern: {name}')


    # Create dictionary to use for renaming process
    rename_dict = dict()

    # Find closest node to point
    for nam in name_points:
        name, point = nam[0], nam[1]
        closest = None
        closest_distance = -1
        for n in tmap['nodes']:
            dist = math.sqrt((point['x'] - n['node']['pose']['position']['x'])**2 + \
                             (point['y'] - n['node']['pose']['position']['y'])**2)
            if closest_distance < 0 or dist < closest_dist:
                closest = n['node']['name']
                closest_distance = dist
        print(f"Closest node to {name} is {closest}")
        rename_dict[closest] = name

    # Find node within polygon (assume only one)
    for nam in name_regions:
        name, poly = nam[0], nam[1]
        for n in tmap['nodes']:
            P1 = Point(n['node']['pose']['position']['x'], n['node']['pose']['position']['y'])
            if poly.contains(P1):
                print(f"Polygon {name} contains {n['node']['name']}")
                rename_dict[n['node']['name']] = name
                break

    # Loop through each node replacing the names marked for replacement
    for n in tmap['nodes']:

        # Rename node details
        if n['node']['name'] in rename_dict:
            new = rename_dict[n['node']['name']]
            n['meta']['node'] = new
            n['node']['name'] = new

        # Rename edge content
        for e in n['edges']:

            # Rename destination nodes
            if e['node'] in rename_dict:
                new = rename_dict[n['node']['name']]

            # Rename edge_id components
            eids = e['edge_id'].split('_')
            if eids[0] in rename_dict:
                eids[0] = rename_dict[eids[0]]
            if eids[1] in rename_dict:
                eids[1] = rename_dict[eids[1]]

    # Save result
    tmap_path = os.path.join(args['src'], 'config', 'topological', 'network_autogen.tmap2.yaml')
    with open(tmap_path, 'w') as f:
        f.write(yaml.dump(tmap))



def main(args=None):
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
