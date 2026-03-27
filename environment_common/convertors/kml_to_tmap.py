import sys, os
import yaml
from ament_index_python.packages import get_package_share_directory, get_package_prefix
import xml.etree.ElementTree as ET
from pprint import pprint

from environment_common.convertors.templating.tmap import TMapTemplates
from environment_common.convertors.tools.gps import calculate_displacement, calculate_distance_changes

from environment_common.convertors.tools.files import load_kml_root


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
        for i, base in enumerate(root):
            print(i)
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


def group_similar_coords(coord_dict_list):

    # Add initial labels for use in filtering later
    for i in range(len(coord_dict_list)):
        coord_dict_list[i]['name'] = str(i)
        coord_dict_list[i]['keep'] = False
        coord_dict_list[i]['clear'] = False

    # Mark each node as to be cleared or kept
    for node in coord_dict_list:

        # Skip if node is already to be cleared
        if node['clear']: continue

        # Mark node to keep (first time seeing node)
        node['keep'] = True
        node['cleared_by'] = node['raw_name']
        la, lo = node['latitude'], node['longitude']

        for node2 in coord_dict_list:

            # Skip if node has been viewed already
            if node2['keep'] or node2['clear']: continue

            # If node 1 and node 2 are within close proximity, clear node 2
            la2, lo2 = node2['latitude'], node2['longitude']
            if abs(la-la2) < 0.000002 and abs(lo-lo2) < 0.000002:

                # Mark node for clearance so it is not viewed anymore
                node2['clear'] = True
                node2['cleared_by'] = node['raw_name']

                # Copy the cleared nodes connections into the kept node's details
                node['raw_connections'] += node2['raw_connections']

    keeps = ['latitude', 'longitude', 'elevation', 'raw_name', 'raw_connections']
    kept = [{f:n[f] for f in keeps} for n in coord_dict_list if not n['clear']]
    for i in range(len(kept)):
        kept[i]['name'] = f"WayPoint{i+1}"

    final_convertor = {cdl['raw_name']: cdl['name'] for cdl in kept}
    convertor = {cdl['raw_name']: final_convertor[cdl['cleared_by']] for cdl in coord_dict_list}
    for k in kept:
        print('keeping? '+str(k))
        k['connections'] = set(convertor[c] if c in convertor else c for c in k['raw_connections'])
        del k['raw_connections'], k['raw_name']

    return kept


def format_tmap(lesspoints, place_id):

    # Prepare dictionary
    tmap = TMapTemplates.vert_sample
    tmap += TMapTemplates.opening.format(**{'gen_time':0, 'location':place_id})

    # Outline basic node/edge contents
    node = {'location':place_id,
            'vert': 'vert1',
            'restrictions':'"True"',
            'connections':None}

    edge = {'action':'move_base',
            'action_type':'move_base_msgs/MoveBaseGoal',
            'restrictions':'"True"'}

    # Add nodes
    for l in lesspoints:
        node.update({'name':l['name'], 'x':l['x'], 'y':l['y']})
        tmap += TMapTemplates.node.format(**node)

        # Add edges
        if not l['connections']:
            tmap += TMapTemplates.edges_empty
        else:
            tmap += TMapTemplates.edges_start
            for c in l['connections']:
                if l['name'] == c:
                    continue
                edge.update({'name':l['name'], 'name2':c})
                tmap += TMapTemplates.edges.format(**edge)

    return tmap

def run(args=None):
    place_id = args['location_name']

    # 1) load datum file so we can convert to metric space
    datum_path = os.path.join(args['src'], 'config', 'location', 'datum.yaml')
    if not os.path.isfile(datum_path):
        datum_path = os.path.join(args['src'], 'config', 'location', 'datum_autogen.yaml')
    with open(datum_path) as f:
        data = f.read()
        datum = yaml.safe_load(data)
    lat = datum['datum_latitude']
    lon = datum['datum_longitude']

    # 2) load kml file containing network points
    ENV = get_package_share_directory('environment_template')
    kml_path = os.path.join(args['src'], 'config', 'topological', f'network.kml')
    root = load_kml_root(kml_path)
    coords = KlmRead.get_coords(root)

    # 3) flatten down the points into a single list and merge points which are in a similar position
    allpoints = sum(coords.values(),[])
    lesspoints = group_similar_coords(allpoints)
    print(lesspoints)
    for l in lesspoints:
        l['y'], l['x'] = calculate_distance_changes(lat, lon, l['latitude'], l['longitude'])
    [print(f"{l['name']} ({l['longitude']}:{l['latitude']})  -  {l['connections']}") for l in lesspoints]
    print(f"Reduced {len(allpoints)} raw points down to {len(lesspoints)}")

    # 4) begin formatting the tmap dictionary
    tmap = format_tmap(lesspoints, place_id)

    # 5) write the tmap file
    tmap_path = os.path.join(args['src'], 'config', 'topological', 'network_autogen.tmap2.yaml')
    print(tmap_path)
    with open(tmap_path, 'w') as f:
        f.write(tmap)


def main(args=None):
    e = 'environment_template'
    src = '/'.join(get_package_prefix(e).split('/')[:-2]) + f'/src/{e}'
    location_name = os.getenv('FIELD_NAME')
    if not location_name:
        print('missing ENVVAR FIELD_NAME, not continuing')
        print('first populate config/properties.sh')
        print('then source config/properties.sh')
        return
    print('Generating map for field: '+location_name)
    args = {'src': src, 'location_name':location_name, 'line_col':'ff2f2fd3', 'line_width':'4', 'fill_col':'c02f2fd3', 'shape_size':0.000005}
    run(args)

if __name__ == '__main__':
    main()
