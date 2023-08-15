import environment_common.convertors.tools.xml as XML
import xml.etree.ElementTree as ET

def getitems(xml):
    return {k:v for k,v in xml.items()}

def gettree(root):
    tree = {tag:dict() for tag in set([r.tag for r in root])}

    # Format osm into referable dictionary
    for xml in root:
        items = XML.getitems(xml)
        id = items['id'] if 'id' in items.keys() else 'bound'
        tree[xml.tag][id] = items
        tree[xml.tag][id]['xml'] = xml

    # Format extra osm properties
    add_way_tags(tree['way'])
    add_way_coords(tree['way'], tree['node'])

    return tree

def get_way_data(xml):
    ref_list = [nd.items()[0][1] for nd in xml if nd.tag == 'nd']
    tag_dict = {nd.items()[0][1]:nd.items()[1][1] for nd in xml if nd.tag != 'nd'}
    return {'ref_list':ref_list, 'tag_dict':tag_dict}

def add_way_tags(ways):
    for w in ways.values():
        w.update(get_way_data(w['xml']))

def add_way_coords(ways, nodes):
    for w in ways.values():
        coords = [{'lat':nodes[r]['lat'],'lon':nodes[r]['lon']} for r in w['ref_list']]
        w['coords'] = coords


