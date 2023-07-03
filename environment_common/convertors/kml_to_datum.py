import sys, os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
import xml.etree.ElementTree as ET
from pprint import pprint

from environment_common.convertors.templating.datum import DatumTemplates


def run(args=None):
    ENV = get_package_share_directory('environment_template')
    kml_path = os.path.join(args['src'], 'config', 'location', 'fences.kml')
    locations = dict()

    while True:
        print(f'Constructing datum from `{kml_path}`. \nTo use a different KML, place the `.kml` file in: `environment_template/config/location/` \n\n\nEnter the name of the file below, or press [ENTER] to continue:')
        inp = input('>> environment_template/config/location/')
        print('\n')
        print(inp)
        if inp != '':
            if not inp.endswith('.kml'):
                print('Ensure you have included the correct file extension of: `.kml`\n\n')
            else:
                kml_path = os.path.join(args['src'], 'config', 'location', inp)
                break
        else:
            break

    tree = ET.parse(kml_path)
    root = tree.getroot()
    for i, base in enumerate(root[0]):
        if not base: continue
        if base[0].text == '\n\t\t\t': continue
        locations[base[0].text] = base
    #pprint(locations)

    while True:
        print("\nPlease select which Placemark to use for the gnss_fence:")
        print(f"Available Placemarks: {list(locations.keys())}")
        loc = input('>> ')
        if loc in locations:
            break

    environment = locations[loc]

    longitude = environment[1][0].text
    latitude = environment[1][1].text

    fency = environment[3][0][0][0].text.replace('\n','').replace('\t','').split(' ')
    gnss_fence = [lle.split(',')[:2] for lle in fency][:-1]

    xmin = -100
    xmax = 100
    ymin = -100
    ymax = 100

    datum_path = os.path.join(args['src'], 'config', 'location', 'datum_autogen.yaml')
    with open(datum_path, 'w') as f:
        txt = DatumTemplates.template%(latitude, longitude, str(gnss_fence).replace("'",""), xmin, xmax, ymin, ymax)
        f.write(txt)
    print(f'\n\nSaved `datum_autogen.yaml` to `{datum_path}`\n\n')

def main(args=None):
    e = 'environment_template'
    src = '/'.join(get_package_prefix(e).split('/')[:-2]) + f'/src/{e}'
    location_name = 'riseholme_field_1'
    run({'src': src, 'location_name':location_name})

if __name__ == '__main__':
    main()
