# -*- coding: utf-8 -*-
#! /usr/bin/env python3
# ----------------------------------
# @author: jheselden
# @email: jheselden@lincoln.ac.uk
# @date:
# ----------------------------------

import sys, os
from ament_index_python.packages import get_package_share_directory, get_package_prefix

import yaml

from environment_common.convertors.tools.mapbox import get_image
from environment_common.convertors.tools.gps import get_bounds


def run(args=None):
    # Load the datum.yaml file
    datum_path = os.path.join(args['src'], 'config', 'location', 'datum.yaml')
    if not os.path.isfile(datum_path):
        datum_path = os.path.join(args['src'], 'config', 'location', 'datum_autogen.yaml')
    with open(datum_path) as f:
        data = f.read()
        datum_raw = yaml.safe_load(data)

    # Load the image
    TOKEN = os.getenv('MAPBOX_TOKEN')
    img = get_image(datum_raw['gnss_fence']['gnss_fence_coords'], TOKEN)

    # Save the image
    img_path = os.path.join(args['src'], 'config', 'location', f"satellite_autogen.png")
    img.save(img_path, "PNG")


def main(args=None):
    e = 'environment_template'
    src = '/'.join(get_package_prefix(e).split('/')[:-2]) + f'/src/{e}'
    location_name = 'riseholme_general_east_pathway'
    args = {'src': src, 'location_name':location_name}
    run(args)


if __name__ == '__main__':
    main()

