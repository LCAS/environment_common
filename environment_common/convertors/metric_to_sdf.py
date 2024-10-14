import sys, os
import yaml
from ament_index_python.packages import get_package_share_directory, get_package_prefix
import xml.etree.ElementTree as ET
from pprint import pprint

from environment_common.convertors.templating.sdf import SDFTemplates

# consider using trace2cad.sh

def run(args=None):

    # Identify the filepath for the map file
    if 'map_yaml_filepath' in args:
        map_yaml_path = args['map_yaml_filepath']
    else:
        map_yaml_path = os.path.join(args['src'], 'config', 'map', 'map', 'map.yaml')
        if not os.path.isfile(map_yaml_filepath):
            map_yaml_path = os.path.join(args['src'], 'config', 'map', 'map', 'map_autogen.yaml')


    # Open the map yaml
    with open(map_yaml_path) as f:
        data = f.read()
        map_yaml = yaml.safe_load(data)
    image = dimensions['image']
    resolution = dimensions['resolution']
    

    # Identify the filepath for the map file
    map_path = os.path.join(args['src'], 'config', 'map', 'map', image)
    if map_path.endswith('.pgm'):
        with open(map_path, 'rb') as file:
            # Read the first two lines: PGM type and dimensions
            header = file.readline() + file.readline()
            if b'#' in header:  # Handle comment line if present
                header += file.readline()
            # Decode header to parse dimensions
            header = header.decode('ascii').split()
            dimensions = [int(s) for s in header if s.isdigit()]
            return dimensions[0], dimensions[1]
    
    """
    1. open map yaml
    2. get image file size (what about alternative image like blueprint)
    3. 
    
    
    """








    # Save tmap2 file
    if 'tmap2_filepath' in args:
        tmap_path = args['tmap2_filepath']
    else:
        tmap_path = os.path.join(args['src'], 'config', 'topological', 'network_autogen.tmap2.yaml')

    # Write the tmap
    print(tmap_path)
    with open(tmap_path, 'w') as f:
        f.write(tmap)

    # Upload to google drive path if included
    if os.getenv('GDRIVE_PATH', ""):
        gdrive_path = os.path.join(os.getenv('GDRIVE_PATH'), 'Google Earth', 'kml', 'network_autogen.tmap2.yaml')
        with open(gdrive_path, 'w') as f:
            f.write(tmap)


def main(args=None):
    e = 'environment_template'
    src = '/'.join(get_package_prefix(e).split('/')[:-2]) + f'/src/{e}'
    location_name = os.getenv('FIELD_NAME')
    if not location_name:
        print('missing ENVVAR FIELD_NAME, not continuing')
        return
    print('Generating map for field: '+location_name)
    args = {'src': src,
            'location_name':location_name,
            'line_col':'ff2f2fd3',
            'line_width':'4',
            'fill_col':'c02f2fd3',
            'shape_size':0.000005}
    run(args)

def main_no_template(datum_filepath, kml_filepath, tmap2_filepath):
    args = {'datum_filepath': datum_filepath,
            'kml_filepath': kml_filepath,
            'tmap2_filepath': tmap2_filepath,
            'location_name':'none',
            'line_col':'ff2f2fd3',
            'line_width':'4',
            'fill_col':'c02f2fd3',
            'shape_size':0.000005}
    run(args)

if __name__ == '__main__':
    if len(sys.argv) >= 3:
        d, k, t = sys.argv[1], sys.argv[2], sys.argv[3]
        print(f'Datum filepath: {d}')
        print(f'  KML filepath: {k}')
        print(f'TMap2 filepath: {t}')
        main_no_template(d, k, t)
    else:
        main()



