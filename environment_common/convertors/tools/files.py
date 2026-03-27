import os, yaml
import xml.etree.ElementTree as ET
from environment_common.convertors.tools.kml import getroot, gettree

def load_yaml_root(path):
    if not os.path.isfile(path):
        path = path.replace('.yaml', '_autogen.yaml')
    if not os.path.isfile(path):
        return {}
    with open(path) as f:
        data = f.read()
    return yaml.safe_load(data)


def load_kml_root(path):
    if not os.path.isfile(path):
        path = path.replace('.kml', '_autogen.kml')
    if not os.path.isfile(path):
        return {}
    tree = ET.parse(path)
    root = tree.getroot()
    return root
