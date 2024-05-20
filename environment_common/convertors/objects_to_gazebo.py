import sys, os
from ament_index_python.packages import get_package_share_directory, get_package_prefix

import yaml
from pprint import pprint

import environment_common
import environment_common.convertors
from environment_common.convertors.templating.gazebo import GazeboTemplates
from environment_common.convertors.templating.gazebo_models.environmental import Environment
from environment_common.convertors.templating.gazebo_models.meta import Meta


def run(args=None):


    # Load objects.yaml file
    objects_path = os.path.join(args['src'], 'config', 'world', 'objects.yaml')
    if not os.path.isfile(objects_path):
        objects_path = os.path.join(args['src'], 'config', 'world', 'objects_autogen.yaml')
    with open(objects_path) as f:
        data = f.read()
        objects = yaml.safe_load(data)


    # Pull in custom objects from references
    custom_components = []
    custom_objects_path = os.path.join(args['src'], 'config', 'world', 'custom_models')
    for obj in objects['components']:
        if obj['type']['object'] == 'custom':
            if obj['type']['reference']:
                filepath = f"{custom_objects_path}/{obj['type']['reference']}"
                with open(filepath) as f:
                    data = f.read()
                    custom_object = yaml.safe_load(data)
                    print(f"Loading in {custom_object['name']}")
                    custom_components += custom_object['components']
    objects['components'] += custom_components


    # Remove reference objects
    filtered_components = []
    for object in objects['components']:
        if 'type' not in object:
            filtered_components += [object]
            continue
        if object['type']['object'] != 'custom':
            filtered_components += [object]
            continue
    objects['components'] = filtered_components

    # Set default values for fields
    for obj in objects['components']:
        if 'size' not in obj:
            obj['size'] = {'x': 1.0, 'y': 1.0, 'z': 1.0}
        if 'position' not in obj:
            obj['position'] = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        if 'orientation' not in obj:
            obj['orientation'] = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}


    # Construct gazebo xml string
    gazebo = Meta.get(objects)


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

