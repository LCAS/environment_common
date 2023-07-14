import sys, os
from ament_index_python.packages import get_package_share_directory, get_package_prefix

import yaml
from pprint import pprint

from PIL import Image


def run(args=None):
    # Load the map.yaml file
    mapyaml_path = os.path.join(args['src'], 'config', 'metric', 'map', 'map.yaml')
    with open(mapyaml_path) as f:
        data = f.read()
        mapyaml = yaml.safe_load(data)

    # Load the map image
    img_path = os.path.join(args['src'], 'config', 'metric', 'map', mapyaml['image'])
    img = Image.open(img_path)
    img = img.convert("RGBA")
    datas = img.getdata()

    # Make parts transparent
    newData = []
    thresh1 = 255 * (1-mapyaml['free_thresh'])
    thresh2 = 255 * (1-mapyaml['occupied_thresh'])
    for item in datas:
        if item[0] < thresh1 and item[1] < thresh1 and item[2] < thresh1\
        and item[0] > thresh2 and item[1] > thresh2 and item[2] > thresh2:
            newData.append((255, 255, 255, 0))
        elif item[0] >= thresh1 and item[1] >= thresh1 and item[2] >= thresh1:
            newData.append((255, 255, 255, 224))
        else:
            newData.append(item)
    img.putdata(newData)

    # Save
    file = mapyaml['image'].split('.')
    tp_img_path = os.path.join(args['src'], 'config', 'metric', 'map', f"{file[0]}-tp.{file[1]}")
    img.save(tp_img_path, "PNG")

    gdrive_path = os.path.join(os.getenv('GDRIVE_PATH'), 'Google Earth', 'kml', f"{file[0]}-tp.{file[1]}")
    img.save(gdrive_path, "PNG")

def main(args=None):
    e = 'environment_template'
    src = '/'.join(get_package_prefix(e).split('/')[:-2]) + f'/src/{e}'
    args = {'src': src}
    run(args)

if __name__ == '__main__':
    main()
