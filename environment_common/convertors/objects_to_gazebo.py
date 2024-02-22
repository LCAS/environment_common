import sys, os
from ament_index_python.packages import get_package_share_directory, get_package_prefix

import yaml
from pprint import pprint

import environment_common
from environment_common.convertors.templating.gazebo import GazeboTemplates
from environment_common.convertors.templating.gazebo_models import GazeboModels


def run(args=None):

    # Load objects.yaml file
    objects_path = os.path.join(args['src'], 'config', 'world', 'objects.yaml')
    if not os.path.isfile(objects_path):
        objects_path = os.path.join(args['src'], 'config', 'world', 'objects_autogen.yaml')
    with open(objects_path) as f:
        data = f.read()
        objects = yaml.safe_load(data)

    # Compile gazebo.world content
    gazebo = GazeboTemplates.quick_opening

    # Apply ground if flat
    if objects['ground']['flat']:
        w, h = objects['ground']['width'], objects['ground']['height']
        gazebo += GazeboTemplates.get_ground(w, h)

    # Spawn each model
    states = ''
    for key, obj in objects['models'].items():
        print('\n\n')
        print(key)
        pprint(obj)
        print('\n')

        # Identify properties
        name, typ = key, obj['type']
        pose = (obj['position']['x'], obj['position']['y'], obj['position']['z'],)
        pose += (obj['orientation']['roll'], obj['orientation']['pitch'], obj['orientation']['yaw'],)
        primary_link = typ+'_'+name

        # Format model object
        gazebo += getattr(GazeboModels, typ) % ((name,) + (primary_link,) + pose)

        # Format model state
        states += GazeboTemplates.state_model % ((name,) + pose + (primary_link,) + pose)

    # Add states to gazebo
    gazebo += GazeboTemplates.state_opening
    gazebo += states
    gazebo += GazeboTemplates.state_closing

    # Finish object spawning
    gazebo += GazeboTemplates.closing

    # Save gazebo.world file
    gazebo_path = os.path.join(args['src'], 'config', 'world', 'gazebo_autogen.world.xml')
    with open(gazebo_path, 'w') as f:
        f.write(gazebo)

def main(args=None):
    e = 'environment_template'
    src = '/'.join(get_package_prefix(e).split('/')[:-2]) + f'/src/{e}'
    args = {'src': src}
    run(args)

if __name__ == '__main__':
    main()
