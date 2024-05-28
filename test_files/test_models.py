# /bin/python

import os
import yaml
from pprint import pprint


class model():

    def all_parents(self, i=0, check=''):
        init = ''
        if i == 0:
            init = 'All Parents:\n'

        padding = '|   '*i + '| '
        i += 1

        # Handle instances of excessive recursion
        if i > 100:
            print('Recursion Crop.')
            return ''

        # Get parentb
        lst = []
        parent = self.parent_obj
        l = ''
        if parent:
            l = f'\n{parent.all_parents(i, check)}'
        check_str = eval(f'f"{check}"')
        return f'{init}{padding}{self.name}{check_str}{l}'

    def all_children(self, i=0, check=''):
        init = ''
        if i == 0:
            init = 'All Children:\n'

        padding = '|   '*i + '| '
        i += 1

        # Handle instances of excessive recursion
        if i > 100:
            print('Recursion Crop.')
            return ''

        # Get list of children
        lst = []
        for child in self.children_objs:
            lst += [f'{child.all_children(i, check=check)}']

        # Add the children to the return str
        l = ''
        if lst:
            l = f'\n' + '\n'.join(lst)
        check_str = eval(f'f"{check}"')
        return f'{init}{padding}{self.name}{check_str}{l}'


    def __init__(self, yml):
        self.name = yml['name']
        self.parent = yml['parent'] if 'parent' in yml else None
        self.parent_obj = None
        self.children = []
        self.children_objs = []
        self.sdf = ''
        self.sdf_available = False
        self.sdf_size_mb = '10'

if __name__=='__main__':

    # Load file
    with open('models.yaml') as f:
        yml = yaml.safe_load(f)

    # Construct objects to manage each item
    models = {item['name']:model(item) for item in yml['models']}
    model_list = list(models.values())

    # Nest the parent objects and child objects
    for m in model_list:
        if m.parent:
            m.parent_obj = models[m.parent]
            models[m.parent].children += [m.name]
            models[m.parent].children_objs += [m]

    # Find root
    root = [m for m in model_list if m.parent == None][0]
    print(root.all_children)
    print('\n'*5)

    oak = models['oak']
    print(oak.all_children(check=' ({self.sdf_size_mb}Mb)'))
    print('\n'*3)
    print(oak.all_parents(check=' ({self.sdf_size_mb}Mb)'))
    print('\n'*3)
