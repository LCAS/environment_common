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


def group_similar_coords(coord_dict_list, aoe=1.5):
    print(f'\n------------------------\n\nGrouping Nodes with aoe of {aoe}')

    # Add initial labels for use in filtering later
    for node in coord_dict_list:
        node['keep'] = False
        node['clear'] = False
        node['victims'] = [node['name']]

    # Show connections
    #print('\nConnections')
    #pprint([[c['name'], [int(w.replace('WayPoint', '')) for w in c['connections']]] for c in coord_dict_list])

    # Mark each node as to be cleared or kept
    for node in coord_dict_list:


        # Skip if node is already to be cleared
        if node['clear']: continue


        # Mark node to keep (first time seeing node)
        node['keep'] = True
        node['cleared_by'] = node['name']
        x1, y1 = node['x'], node['y']

        for node2 in coord_dict_list:


            # Skip if node has been viewed already
            if node2['keep'] or node2['clear']: continue


            # If node 1 and node 2 are within close proximity, clear node 2
            x2, y2 = node2['x'], node2['y']
            if abs(x1-x2) < aoe and abs(y1-y2) < aoe:


                # Mark node for clearance so it is not viewed anymore
                node2['clear'] = True

                # Copy all victim nodes into the one being kept
                #     node is absobing all nodes already absorbed by the target
                #print('@', node['name'], '=', node['victims'], '\n|', node2['name'], '=', node2['victims'], '\n')
                node['victims'] += node2['victims']

                ## Copy the absorbed nodes connections into the kept node's details
                node['connections'].update(node2['connections'])


    # Create reference dictionary
    cdl = {node['name']:node for node in coord_dict_list}
    rename = {node['name']:node['name'] for node in coord_dict_list}

    # Loop through the cdl victims and apply the parent node to the rename block
    for n in coord_dict_list:
        for v in n['victims']:
            if v != n['name']:
                rename[v] = n['name']

    # Show new owners
    print('\n\nRenaming: dict[victim] = king')
    pprint({int(k.replace('WayPoint', '')):v for k,v in rename.items() if k != v})

    # Show mergings
    print('\n\nMergings: dict[king] = victims')
    pprint({c['name']: [int(w.replace('WayPoint', '')) for w in c['victims'] if w != c['name']] for c in coord_dict_list if not c['clear'] and len(c['victims']) > 1})

    # Begin tidying the node details
    keeps = ['name', 'connections', 'x', 'y']
    kept = [{f:n[f] for f in keeps} for n in coord_dict_list if not n['clear']]

    # Rename connections using rename dictionary
    for node in kept:
        node['connections'] = {rename[c] for c in node['connections']}

    # When a neighbour is copied in, it brings its edge to the new owner
    # ... we just need to filter that out
    for node in kept:
        node['connections'] = {n for n in node['connections'] if n != node['name']}


    print(f"\n\nReduced {len(coord_dict_list)} raw points down to {len(kept)} by {len(coord_dict_list)-len(kept)}")

    check_diff(kept)

    return kept


def get_intersecting_point(line1node1, line1node2, line2node1, line2node2):
    c1a = [line1node1['x'], line1node1['y']]
    c1b = [line1node2['x'], line1node2['y']]
    c2a = [line2node1['x'], line2node1['y']]
    c2b = [line2node2['x'], line2node2['y']]

    a1 = (c1b[1] - c1a[1]) / (c1b[0] - c1a[0])
    a2 = (c2b[1] - c2a[1]) / (c2b[0] - c2a[0])

    b1 = -1
    b2 = -1

    c1 = c1a[1] - (a1 * c1a[0])
    c2 = c2a[1] - (a2 * c2a[0])

    try:
        x = ( (b1*c2) - (b2*c1) ) / ( (a1*b2) - (a2*b1) )
        y = ( (c1*a2) - (c2*a1) ) / ( (a1*b2) - (a2*b1) )
    except Exception as e:
        print('------\n\n\n', line1node1, '\n', line1node2, '\n', line2node1, '\n', line2node2, '\n\n\n')

    if (x < min([c1a[0],c1b[0]]) or x > max([c1a[0], c1b[0]])):
        #print('x outside of line 1')
        return (None, None)
    elif (y < min([c1a[1],c1b[1]]) or y > max([c1a[1], c1b[1]])):
        #print('y outside of line 1')
        return (None, None)
    elif (x < min([c2a[0],c2b[0]]) or x > max([c2a[0], c2b[0]])):
        #print('x outside of line 2')
        return (None, None)
    elif (y < min([c2a[1],c2b[1]]) or y > max([c2a[1], c2b[1]])):
        #print('y outside of line 2')
        return (None, None)
    else:
        #print('intersection occured')
        return (x, y)


def split_intersecting_lines(coord_dict_list):
    print(f'\n------------------------\n\nSplitting edges at intersections')

    cdl = {c['name']:c for c in coord_dict_list}
    highest_wp = max([int(node['name'].replace('WayPoint','')) for node in coord_dict_list])

    # Loop through all edges
    counter = 0
    new_intersections = []
    checked_intersections = []
    for line1node1 in coord_dict_list:
        for c1 in line1node1['connections']:
            line1node2 = cdl[c1]

            for line2node1 in coord_dict_list:
                for c2 in line2node1['connections']:
                    line2node2 = cdl[c2]

                    # Check the ends of the lines are different
                    lines = {'line1': [line1node1['name'], line1node2['name']], 'line2':[line2node1['name'], line2node2['name']]}
                    names = set([line1node1['name'], line1node2['name'], line2node1['name'], line2node2['name']])
                    if len(names) < 4:
                        continue
                    if names in checked_intersections:
                        continue

                    # Determine if intersection is within bounds
                    x, y = get_intersecting_point(line1node1, line1node2, line2node1, line2node2)
                    if x:

                        # Compile details for new node
                        counter += 1
                        wp = 'WayPoint'+str(highest_wp+counter)
                        new_intersections += [{'name': wp, 'x':x, 'y':y, 'connections': names, 'lines': lines}]


                    checked_intersections += [names]


    print('Total Intersections:', counter, '\n\n---\n\n')
    for i, new in enumerate(new_intersections):
        #print(i, ':', new['name'])

        for ln in ['line1', 'line2']:
            n1, n2 = cdl[new['lines'][ln][0]], cdl[new['lines'][ln][1]]
            #print('line:', ln, 'connecting', n1['name'], 'and', n2['name'])


            # Swap the edges to reference the new intersection node instead
            n1['connections'].add(new['name'])
            if n2['name'] in n1['connections']: n1['connections'].remove(n2['name'])

            n2['connections'].add(new['name'])
            if n1['name'] in n2['connections']: n2['connections'].remove(n1['name'])

        #print('---\n')

    no_intersections_coord_dict_list = [v for v in cdl.values()] + new_intersections
    return no_intersections_coord_dict_list




def refresh_names(coord_dict_list):

    # Assign proper names to each node
    for i, node in enumerate(allpoints):
        node['name'] = f"WayPoint{i+1}"


def convert_raw_to_wp(coord_dict_list):

    # Create dictionary to convert raw_connection ids to WayPoint names
    convertor = {n['raw_name']:n['name'] for n in coord_dict_list}

    # Apply convertor to nodes and edges
    for n in coord_dict_list:
        n['name'] = convertor[n['raw_name']]
        n['connections'] = set([convertor[c] for c in n['raw_connections']])
        del n['raw_connections'], n['raw_name']

    return coord_dict_list



def check_diff(coord_dict_list):
    nodes_check = set([c['name'] for c in coord_dict_list])
    edges_check = set(sum([list(c['connections']) for c in coord_dict_list],[]))
    edge_length = [len(c['connections']) for c in coord_dict_list]
    #print('unique node targets', len(nodes_check))
    #print('unique edge targets', len(edges_check))

    checks = edges_check.symmetric_difference(nodes_check)

    if checks:
        print('\n')
        print('nodes without edges', sorted(nodes_check.difference(edges_check)))
        print('edges without nodes', sorted(edges_check.difference(nodes_check)))
        print('lens?', sorted(edge_length))
        print('difference between?', sorted([int(i.replace('WayPoint', '')) for i in list(checks)]), '\n\n')

        print('quitting...')
        quit()



def run(args=None):

    # Load the datum file to use for cvonverting gps to metric
    datum_path = os.path.join(args['src'], 'config', 'location', 'datum.yaml')
    if not os.path.isfile(datum_path):
        datum_path = os.path.join(args['src'], 'config', 'location', 'datum_autogen.yaml')
    with open(datum_path) as f:
        data = f.read()
        datum = yaml.safe_load(data)

    place_id = args['location_name']
    lat = datum['datum_latitude']
    lon = datum['datum_longitude']


    # Acquire the kml file to convert to tmap
    ENV = get_package_share_directory('environment_template')
    kml_path = os.path.join(args['src'], 'config', 'topological', 'raw_connections.kml')
    while True:
#        break
        print(f'Constructing tmap from `{kml_path}`. \nTo use a different KML, place the `.kml` file in: `environment_template/config/topological/` \n\n\nEnter the name of the file below, or press [ENTER] to continue:')
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


    # Parse the kml file into a dictionary for us to use
    locations = dict()
    tree = ET.parse(kml_path)
    root = tree.getroot()
    coords = KlmRead.get_coords(root)


    # Get the coordinates and connections for each node
    allpoints = sum(coords.values(),[])
    print(f"Identified {len(allpoints)} points")
    print(f"\n> allpoints")
    print("\t", allpoints[0].keys())


    # Assign proper names to each node
    for i, node in enumerate(allpoints):
        node['name'] = f"WayPoint{i+1}"


    # Convert raw_connections to connections
    allnodes = convert_raw_to_wp(allpoints)
    print(f"\n> allnodes")
    print("\t", allnodes[0].keys())
    check_diff(allnodes)


    # Convert nodes to metric
    for a in allnodes:
        a['y'], a['x'] = calculate_distance_changes(lat, lon, a['latitude'], a['longitude'])
        del a['longitude'], a['latitude'], a['elevation']
    print(f"\n> allnodes")
    print("\t", allnodes[0].keys())
    check_diff(allnodes)


    # Group the points within 1.5m into single nodes
    lessnodes = group_similar_coords(allnodes, aoe=1.5)
    print(f"Reduced {len(allnodes)} raw points down to {len(lessnodes)}")
    print(f"\n> lessnodes")
    print("\t", lessnodes[0].keys())
    check_diff(lessnodes)
    #lessnodes = allnodes


    print('\n')
    pprint({c['name']:[c['connections'], c['x'], c['y']] for c in lessnodes})


    # Segment overlapping lines
    nocrosses = split_intersecting_lines(lessnodes)
    print(f"Added an additional {len(nocrosses)-len(lessnodes)} nodes at intersecting lines.")
    print(f"\n> nocrosses")
    print("\t", nocrosses[0].keys())
    check_diff(nocrosses)
    #nocrosses = lessnodes


    print('\n')
    pprint({c['name']:[c['connections'], c['x'], c['y']] for c in nocrosses})


    # Group the points within 1.5m into single nodes
    lessnocrosses = group_similar_coords(nocrosses, aoe=1.5)
    print(f"Reduced {len(nocrosses)} intersecting points down to {len(lessnocrosses)}")
    print(f"\n> lessnocrosses")
    print("\t", lessnocrosses[0].keys())
    check_diff(lessnocrosses)
    #lessnocrosses = nocrosses


    print('\n')
    pprint({c['name']:[c['connections'], c['x'], c['y']] for c in lessnocrosses})

    print('\n\n\n\n\n')
    print('WARNING: EACH INTERSECTION NODE CONNECTS ONLY TO ITS BOUNDARIES, NOT TO ANY OTHER NODES')

    # Begin construction of the tmap file
    tmap = TMapTemplates.vert_sample


    # Create a set of standard vertices
    #tmap += TMapTemplates.vert_opening
    #tmap += TMapTemplates.vert_ring.format(**{'id':'vert2', 'sz':1})


    # Begin formatting of nodes for file
    tmap += TMapTemplates.opening.format(**{'gen_time':0, 'location':place_id})


    # Define common properties to apply
    node = {'location':place_id, 'vert': 'vert1', 'restrictions':'robot', 'connections':None}
    edge = {'action':'move_base', 'action_type':'move_base_msgs/MoveBaseGoal', 'restrictions':'robot'}


    # Loop through each node and create the node object
    for l in lessnocrosses:
        node.update({'name':l['name'], 'x':l['x'], 'y':l['y'], 'connections':l['connections']})
        tmap += TMapTemplates.node.format(**node)


        # If the node has no edges, apply empty edge template
        if not node['connections']:
            tmap += TMapTemplates.edges_empty

        # If the node has edges, create edge objects to insert
        else:
            tmap += TMapTemplates.edges_start
            for c in l['connections']:
                edge.update({'name':l['name'], 'name2':c})
                tmap += TMapTemplates.edges.format(**edge)
        #print(l['name'], l['connections'])


    # Save file
    tmap_path = os.path.join(args['src'], 'config', 'topological', 'network_autogen.tmap2.yaml')
    with open(tmap_path, 'w') as f:
        f.write(tmap)


    # Copy to GDRIVE link for quick verification
    if os.getenv('GDRIVE_PATH', ""):
        gdrive_path = os.path.join(os.getenv('GDRIVE_PATH'), 'Google Earth', 'kml', 'network_autogen.tmap2.yaml')
        with open(gdrive_path, 'w') as f:
            f.write(tmap)


def main(args=None):
    e = 'environment_template'
    src = '/'.join(get_package_prefix(e).split('/')[:-2]) + f'/src/{e}'
    location_name = 'riseholme_polytunnel'
    args = {'src': src, 'location_name':location_name}
    run(args)

if __name__ == '__main__':
    main()
