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
        return text.replace('\t', '').replace('\n','')
    return None


def getplacemarks(xml, prefix):
    out = sum([getplacemarks(x, prefix+'/'+getname(x)) for x in xml],[])
    if len(out) < 1:
        return [tuple([prefix+'/'+getname(xml), simplecoordinates(xml.text)])]
        # put the xml object here               ^
    return out


def gettree(root):
    tree = getplacemarks(root, prefix='root')
    p = '/Polygon/outerBoundaryIs/LinearRing/coordinates/coordinates'
    s = 'root/riseholme_fields/'
    c = 'coordinates'
    return {t[0].replace(p,'').replace(s,''):t[1] for t in tree if c in t[0]}

