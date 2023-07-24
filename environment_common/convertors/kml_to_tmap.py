import sys, os
import yaml
from ament_index_python.packages import get_package_share_directory, get_package_prefix
import xml.etree.ElementTree as ET
from pprint import pprint

from environment_common.convertors.templating.tmap import TMapTemplates
from environment_common.convertors.tools.gps import calculate_displacement, calculate_distance_changes


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


def group_similar_coords(coord_dict_list):

    for i in range(len(coord_dict_list)):
        coord_dict_list[i]['name'] = str(i)
        coord_dict_list[i]['keep'] = False
        coord_dict_list[i]['clear'] = False
    #pprint(coord_dict_list)

    for node in coord_dict_list:
        if node['clear']: continue
        node['keep'] = True
        node['cleared_by'] = node['raw_name']
        la, lo = node['latitude'], node['longitude']

        for node2 in coord_dict_list:
            if node2['keep'] or node2['clear']: continue
            la2, lo2 = node2['latitude'], node2['longitude']
            if abs(la-la2) < 0.00001 and abs(lo-lo2) < 0.00001:
                node2['clear'] = True
                node2['cleared_by'] = node['raw_name']
                node['raw_connections'] += node2['raw_connections']
    #pprint(coord_dict_list)

    keeps = ['latitude', 'longitude', 'elevation', 'raw_name', 'raw_connections']
    kept = [{f:n[f] for f in keeps} for n in coord_dict_list if not n['clear']]
    for i in range(len(kept)):
        kept[i]['name'] = f"WayPoint{i+1}"

    final_convertor = {cdl['raw_name']: cdl['name'] for cdl in kept}
    convertor = {cdl['raw_name']: final_convertor[cdl['cleared_by']] for cdl in coord_dict_list}
    for k in kept:
        k['connections'] = set(convertor[c] if c in convertor else c for c in k['raw_connections'])
        del k['raw_connections'], k['raw_name']

    #[print(c) for c in kept]
    return kept


def run(args=None):

    datum_path = os.path.join(args['src'], 'config', 'location', 'datum.yaml')
    if not os.path.isfile(datum_path):
        datum_path = os.path.join(args['src'], 'config', 'location', 'datum_autogen.yaml')
    with open(datum_path) as f:
        data = f.read()
        datum = yaml.safe_load(data)

    #datum_path = os.path.join(args['src'], 'config', 'location', 'datum_autogen.yaml')
    #while True:
    #    print(f'Constructing gnss_fence kml from `{datum_path}`. \nTo use a different Datum, place the `.yaml` file in: `environment_template/config/location/` \n\n\nEnter the name of the file below, or press [ENTER] to continue:')
    #    inp = input('>> environment_template/config/location/')
    #    print('\n')
    #    if inp != '':
    #        if not inp.endswith('.yaml'):
    #            print('Ensure you have included the correct file extension of: `.yaml`\n\n')
    #        datum_path = os.path.join(args['src'], 'config', 'location', inp)
    #    else:
    #        break
    #
    #with open(datum_path) as f:
    #    data = f.read()
    #    datum = yaml.safe_load(data)

    place_id = args['location_name']
    lat = datum['datum_latitude']
    lon = datum['datum_longitude']

    klm_path = os.path.join(args['src'], 'config', 'topological', 'raw_connections.kml')
    locations = dict()

    tree = ET.parse(klm_path)
    root = tree.getroot()
    coords = KlmRead.get_coords(root)
    #pprint(coords)

    allpoints = sum(coords.values(),[])
    #[print(f"{l['longitude']} : {l['latitude']}) \t-- {l['raw_name']} {l['raw_connections']}") for l in allpoints]

    lesspoints = group_similar_coords(allpoints)
    for l in lesspoints:
        l['y'], l['x'] = calculate_distance_changes(lat, lon, l['latitude'], l['longitude'])
    [print(f"{l['name']} ({l['longitude']}:{l['latitude']})  -  {l['connections']}") for l in lesspoints]
    print(f"Reduced {len(allpoints)} raw points down to {len(lesspoints)}")

    #[print(f"{l['name']} ({round(l['x'],1)}:{round(l['y'],1)})  -  {l['connections']}") for l in lesspoints]

    tmap = TMapTemplates.vert_sample
    #tmap += TMapTemplates.vert_opening
    #tmap += TMapTemplates.vert_ring.format(**{'id':'vert2', 'sz':1})

    tmap += TMapTemplates.opening.format(**{'gen_time':0, 'location':place_id})

    node = {'location':place_id, 'vert': 'vert1', 'restrictions':'robot'}
    edge = {'action':'move_base', 'action_type':'move_base_msgs/MoveBaseGoal', 'restrictions':'robot'}
    for l in lesspoints:
        node.update({'name':l['name'], 'x':l['x'], 'y':l['y']})
        tmap += TMapTemplates.node.format(**node)
        for c in l['connections']:
            edge.update({'name':l['name'], 'name2':c})
            tmap += TMapTemplates.edges.format(**edge)

    tmap_path = os.path.join(args['src'], 'config', 'topological', 'network_autogen.tmap2')
    with open(tmap_path, 'w') as f:
        f.write(tmap)

    #gdrive_path = os.path.join(os.getenv('GDRIVE_PATH'), 'Google Earth', 'kml', 'network_autogen.tmap2')
    #with open(gdrive_path, 'w') as f:
    #    f.write(tmap)


def main(args=None):
    e = 'environment_template'
    src = '/'.join(get_package_prefix(e).split('/')[:-2]) + f'/src/{e}'
    location_name = 'riseholme_polytunnel'
    args = {'src': src, 'location_name':location_name, 'line_col':'ff2f2fd3', 'line_width':'4', 'fill_col':'c02f2fd3', 'shape_size':0.000005}
    run(args)

if __name__ == '__main__':
    main()
