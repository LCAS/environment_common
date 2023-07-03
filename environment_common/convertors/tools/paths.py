import os

e = 'environment_template'
src = '/'.join(get_package_prefix(e).split('/')[:-2]) + f'/src/{e}'

def get_datum():
    path = os.path.join(src, 'config', 'location', 'datum.yaml')
    if not os.path.isfile(path):
        return os.path.join(src, 'config', 'location', 'datum_autogen.yaml')
    return path
