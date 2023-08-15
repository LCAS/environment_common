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

def getitems(xml):
    return {k:v for k,v in xml.items()}

#def gettree(root):
#    tree = getplacemarks(root, prefix='root')
#    p = '/Polygon/outerBoundaryIs/LinearRing/coordinates/coordinates'
#    s = 'root/riseholme_fields/'
#    c = 'coordinates'
#    return {t[0].replace(p,'').replace(s,''):{'fence':t[1],'xml':t[2]} for t in tree if c in t[0]}

