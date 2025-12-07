# #########################################################################
# Restrictions Service
#
# @author      James R. Heselden (github: iranaphor)
# @maintainer  James R. Heselden (github: iranaphor)
# @datecreated 2nd December 2025
#
# @credits     Code structure and implementation were developed by the
#              author with assistance from OpenAI's GPT-5.1 model used
#              under the maintainer's direction and supervision.
#
# @lastmodifiedby James R. Heselden (github: iranaphor)
# @datemodified   5th December 2025
#
###########################################################################

import os
import yaml
import math
import copy

# Load tmap2 file
tmap_path = os.getenv('TMAP_FILE')
if tmap_path is None:
    raise RuntimeError("Environment variable TMAP_FILE is not set")

with open(tmap_path) as f:
    tmap = yaml.safe_load(f)

# Base semantic label templates (we deep-copy these per node)
base_polytunnel_semantics = {
    'zone': {
        'labels': ['polytunnel'],
        'details': {
            'tunnel_id': 4,   # default; overwritten below
            'column_id': 2    # default; overwritten below
        }
    }
}
base_waypoint_semantics = {
    'zone': {
        'labels': ['outdoors']
    }
}
base_dock_semantics = {
    'zone': {
        'labels': ['indoors', 'charging_station']
    }
}
base_storage_semantics = {
    'zone': {
        'labels': ['indoors', 'storage']
    }
}
base_fhu_semantics = {
    'zone': {
        'labels': ['indoors']
    }
}

# Loop through each node to modify
for n in tmap.get('nodes', []):
    node = n['node']
    name = node['name']

    # If in polytunnel row
    if name.startswith('r') and '-c' in name:
        # Deep copy so we don't mutate the shared template
        s = copy.deepcopy(base_polytunnel_semantics)

        # Identify node location (e.g. "r6.5-c4")
        r_str, c_str = name.replace('r', '', 1).split('-c', 1)
        r_val = float(r_str)

        # Tunnel id based on row (customise as needed)
        t = 0 if r_val <= 5.5 else 1
        s['zone']['details']['tunnel_id'] = str(t)
        s['zone']['details']['column_id'] = c_str

        # If in polytunnel row (fractional)
        if '.' in r_str:
            res = {'max_external_width': 0.8}
            s['zone']['labels'].append('row')
            s['zone']['details']['row_id'] = r_str

            if r_str.endswith('.5'):
                s['zone']['details']['adjacent_beds'] = [
                    math.floor(r_val),
                    math.ceil(r_val)
                ]
            elif r_str.endswith('.3'):
                s['zone']['details']['adjacent_beds'] = [math.ceil(r_val)]
            elif r_str.endswith('.7'):
                s['zone']['details']['adjacent_beds'] = [math.floor(r_val)]

        # If over polytunnel bed
        else:
            res = {'min_cavity_width': 0.8, 'min_cavity_height': 1.6}
            s['zone']['labels'].append('raised_bed')
            s['zone']['details']['bed_id'] = r_str

        # Apply semantics and *copies* of restrictions to avoid YAML anchors
        node['semantics'] = copy.deepcopy(s)
        node['restrictions'] = copy.deepcopy(res)

        for e in node.get('edges', []):
            e['restrictions'] = copy.deepcopy(res)

    # If not in polytunnel
    else:
        # Default restrictions
        res = {}

        # If on dock
        if name.startswith('dock'):
            s = copy.deepcopy(base_dock_semantics)

        # If on storage
        elif name.startswith('s0'):
            s = copy.deepcopy(base_storage_semantics)

        # If in fhu
        elif int(name.replace('WayPoint','')) in [70, 71, 72, 69]:
            s = copy.deepcopy(base_fhu_semantics)

        # If on general waypoint
        else:
            s = copy.deepcopy(base_waypoint_semantics)

        # Apply restrictions and semantics to node and edges
        node['semantics'] = copy.deepcopy(s)
        node['restrictions'] = copy.deepcopy(res)
        for e in node.get('edges', []):
            if node['name'] == 'WayPoint68' and e['node'] == 'WayPoint69':
                res = {'max_width': 0.8, 'max_height': 1.6}
                e['restrictions'] = copy.deepcopy(res)
            elif node['name'] == 'WayPoint69' and e['node'] == 'WayPoint68':
                res = {'max_width': 0.8, 'max_height': 1.6}
                e['restrictions'] = copy.deepcopy(res)
            else:
                e['restrictions'] = copy.deepcopy(res)



# Save modified tmap2 file
out_path = tmap_path.replace('.tmap2.yaml', '_modified.tmap2.yaml')
with open(out_path, 'w') as f:
    yaml.safe_dump(tmap, f)
