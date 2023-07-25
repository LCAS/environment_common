import xml.etree.ElementTree as ET


def getroot(path):
    tree = ET.parse(path)
    return tree.getroot()


def cleantag(xml):
    return xml.tag.split('}')[-1]


def gettags(xml):
    return [cleantag(x) for x in xml]


def getname(xml):
    tags = gettags(xml)
    if 'name' in tags:
        return xml[tags.index('name')].text
    return cleantag(xml)


def simplecoordinates(text):
    if text:
        return polyline_to_list(text)
    return None


def getplacemarks(xml, prefix):
    out = sum([getplacemarks(x, prefix+'/'+getname(x)) for x in xml],[])
    if len(out) < 1:
        return [tuple([prefix+'/'+getname(xml), simplecoordinates(xml.text), xml])]
    return out


def gettree(root):
    tree = getplacemarks(root, prefix='root')
    p = '/Polygon/outerBoundaryIs/LinearRing/coordinates/coordinates'
    s = 'root/riseholme_fields/'
    c = 'coordinates'
    return {t[0].replace(p,'').replace(s,''):{'fence':t[1],'xml':t[2]} for t in tree if c in t[0]}


def polyline_to_list(polyline_str):
    pls = polyline_str.replace('\n','').replace('\t','').split(' ')[:-1]
    return [g.split(',') for g in pls]


def polyline_to_dictlist(polyline_str, name, tagtype):
    coords = polyline_to_list(polyline_str)
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
