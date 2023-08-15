import sys, os
from ament_index_python.packages import get_package_share_directory, get_package_prefix

import yaml
from pprint import pprint

from environment_common.convertors.templating.datum import DatumTemplates

from environment_common.convertors.tools.gps import get_datumrelative_metric_from_gps
from environment_common.convertors.tools.xml import getroot
from environment_common.convertors.tools.osm import gettree


def run(args=None):

    # Load the osm.xml file
    osm_path = os.path.join(args['src'], 'config', 'topological', 'osm.xml')
    if not os.path.isfile(osm_path):
        osm_path = os.path.join(args['src'], 'config', 'topological', 'osm_autogen.xml')
    root = getroot(osm_path)
    tree = gettree(root)

    # Get latlon details
    bound = tree['bounds']['bound']
    n,s,e,w = [bound['maxlat'], bound['minlat'], bound['maxlon'], bound['minlon']]
    longitude, latitude = w, s
    gnss_fence = [[n,e],[n,w],[s,w],[s,e]]

    # Format gmapping details
    sw = {'latitude':s, 'longitude':w, 'elevation':1}
    ne = {'latitude':n, 'longitude':e, 'elevation':1}
    xyz = get_datumrelative_metric_from_gps(sw, ne)
    xmin = -xyz['x']/2
    xmax =  xyz['x']/2
    ymin = -xyz['y']/2
    ymax =  xyz['y']/2

    # Save file
    datum_path = os.path.join(args['src'], 'config', 'location', 'datum_autogen.yaml')
    with open(datum_path, 'w') as f:
        txt = DatumTemplates.template%(latitude, longitude, str(gnss_fence).replace("'",""), xmin, xmax, ymin, ymax)
        f.write(txt)
    print(f'\n\nSaved `datum_autogen.yaml` to `{datum_path}`\n\n')


def main(args=None):
    e = 'environment_template'
    src = '/'.join(get_package_prefix(e).split('/')[:-2]) + f'/src/{e}'
    location_name = 'r_gep'
    args = {'src': src, 'location_name':location_name}
    run(args)


if __name__ == '__main__':
    main()
