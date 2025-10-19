import sys, os, yaml
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from pprint import pprint

import environment_common.convertors as C


def main():

    # 1) load in the list of convertors
    CONVERTORS = '/home/ros/ros2_ws/src/environment_common/environment_common/included_convertors.yaml'
    with open(CONVERTORS, 'r') as f:
        convertors = yaml.load(f.read())

    FARM_NAME = os.getenv('FARM_NAME')
    FIELD_NAME = os.getenv('FIELD_NAME')

    # 2) Begin loop
    while True:

        print('\n###############################')

        # 3) identify the files which are currently available
        CONFIG_DIR = '/home/ros/ros2_ws/src/environment_template/config/'
        walk = list(os.walk(CONFIG_DIR))
        config_files = [[w[0], [p for p in w[2] if '~' not in p and 'placeholder' not in p]] for w in walk]
        filepaths_simple = [os.path.join(d, f) for d, files in config_files if files for f in files]
        filepaths = [os.path.join(f.split('/config/')[1]) for f in filepaths_simple]

        # 4) identify which of the scripts can generate new outputs
        useful = dict()
        print('\nMissing maps can be generated using:')
        for k, v in convertors['builds'].items():
            # Check if the output files already exist
            intersection = list(set(filepaths) & set(v['outputs']))
            if not intersection:
                print(f"\n    {k}")
                [print(f"    ---| {i}") for i in v['outputs']]
                useful[k] = v

        # 5) identify which scripts have available inputs
        possible = dict()
        print('\nPossible systems to generate maps:')
        for k, v in useful.items():

            # Loop through each input requirement
            count_i = 0
            for i in v['inputs']:
                # If it is an 'either', then check intersection of sub-options
                if type(i) == dict and 'either' in list(i.keys()):
                    intersection = list(set(filepaths) & set(list(i['either'])))
                    #print('\n\n\n')
                    #print(i)
                    #print(filepaths)
                    #print(intersection)
                else:
                    intersection = list(set(filepaths) & set([i]))
                    #print('\n\n\n')
                    #print(i)
                    #print(filepaths)
                    #print(intersection)
                # If any intersections exist increment the counter
                if len(intersection) > 0:
                    count_i += 1

            # Loop through each required environment variable
            count_r = 0
            for r in v.get('requires',[]):
                if os.getenv(r):
                    count_r += 1

            # If the counter matches the total file requirements, save the convertor as possible
            if len(v.get('inputs',[])) == count_i and len(v.get('requires',[])) == count_r:
                print(f"\n    {k}")
                [print(f"    ---| {i}") for i in intersection]
                possible[k] = v

        # 6) if none can be executed, break
        if len(possible) == 0:
            print('\n\nNo more files are able to be generated with this instance. Add more data.\n')
            break

        # 7) execute any of them which can be executed
        src = '/home/ros/ros2_ws/src/environment_template/'
        for k,c in possible.items():
            print(f"Executing: {k}")
            args = {'src': src, 'files': c, 'location_name': FIELD_NAME}
            getattr(C, k).run(args)

        # 8) continue the main loop to check again
        print('sleeping...')
        import time
        time.sleep(5)
        continue


if __name__ == '__main__':
    main()
