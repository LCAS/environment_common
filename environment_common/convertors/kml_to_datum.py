import sys, os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from pprint import pprint

from environment_common.convertors.templating.datum import DatumTemplates
from environment_common.convertors.tools.kml import getroot, gettree, polyline_to_list

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

    root = getroot(kml_path)
    locations = gettree(root)

    while True:
        print("\nPlease select which Placemark to use for the gnss_fence:")
        print("Available Placemarks:")
        for l in sorted(list(locations.keys())):
            print(f"| {l}")
        print('|')
        loc = input('| >> ')
        if loc in locations.keys():
            break
    environment = locations[loc]

    longitude = environment['fence'][0][0]
    latitude = environment['fence'][0][1]
    gnss_fence = [lle[:2][::-1] for lle in environment['fence']][:-1]

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
