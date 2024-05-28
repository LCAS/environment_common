import sys, os
from ament_index_python.packages import get_package_share_directory, get_package_prefix

import random
import yaml
from pprint import pprint

def run(args=None):

    totRows = 20
    totCols = 10
    rowWidth = 5
    colWidth = 3

    buffX = (totRows+0) * rowWidth
    buffY = (totCols+0) * colWidth
    maxX = (totRows) * rowWidth
    maxY = (totCols) * colWidth
    #print(buffX, buffY, maxX, maxY)
    #print('---')

    # Populate plane
    data = dict()
    data['components'] = []
    data['components'] += [{
        'type': {'reference': 'plane', 'object': 'primative'},
        'width': buffX,
        'length': buffY,
        'material': {'name': 'DarkGrey'}
    }]

    # Include trees
    for i in range(totRows):
        for j in range(totCols):
            x, y = i*rowWidth, j*colWidth
            #print(x, y)

            xoff = (random.random() - 0.5)/2
            yoff = (random.random() - 0.5)/2
            #print(xoff, yoff)

            xalign = -((maxX - rowWidth)/2)
            yalign = -((maxY - colWidth)/2)
            #print(xalign, yalign)

            #print('---')
            data['components'] += [{
              'type': {'reference': 'tree.yaml', 'object': 'custom'},
              'anchor': { 'position': {'x': x+xoff+xalign, 'y': y+yoff+yalign} }
            }]

    # Save objects.yaml file
    from pprint import pprint

    objects_path = os.path.join(args['src'], 'config', 'world', 'objects_autogen.yaml')
    with open(objects_path, 'w') as f:
        yaml.dump(data, f)


def main(args=None):
    e = 'environment_template'
    src = '/'.join(get_package_prefix(e).split('/')[:-2]) + f'/src/{e}'
    args = {'src': src}
    run(args)


if __name__ == '__main__':
    main()

