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

# Load tmap2 file
tmap_path = os.getenv('TMAP_FILE')
with open(tmap_path) as f:
    tmap = yaml.safe_load(f)



# Define semantic labels container
polytunnel_semantics = {'zone': {
        'labels': ['polytunnel'],
        'details': {'tunnel_id': 4, 'column_id': 2}}}
waypoint_semantics = {'zone': {'labels': ['outdoors']}}
dock_semantics = {'zone': {'labels': ['indoors', 'charging_station']}}

# Loop through each node to modify
for n in tmap['nodes']:

    # If in polytunnel row
    if n['node']['name'].startswith('r') and '-c' in n['node']['name']:

        # Get semantics
        s = polytunnel_semantics

        # Identify node location
        r, c = n['node']['name'].replace('r','').split('-c')

        # Put in a tunnel id (but customise to environment manually)
        t = 0 if float(r) <= 5.5 else 1
        s['zone']['details']['tunnel_id'] = t
        s['zone']['details']['column_id'] = c

        # If in polytunnel row
        if '.' in c:
            res = {'max_external_width': 0.8}
            s['zone']['labels'].append('row')
            s['zone']['details']['row_id'] = r
            if c.endswith('.5'):
                s['zone']['details']['adjacent_beds'] = [math.floor(r), math.ceil(r)]
            elif c.endswith('.3'):
                s['zone']['details']['adjacent_beds'] = [math.ceil(r)]
            elif c.endswith('.7'):
                s['zone']['details']['adjacent_beds'] = [math.floor(r)]

        # If over polytunnel bed
        else:
            res = {'min_cavity_width': 0.8, 'min_cavity_height': 1.6}
            s['zone']['labels'].append('raised_bed')
            s['zone']['details']['bed_id'] = r

        # Apply restrictions
        n['node']['semantics'] = s
        n['node']['restrictions'] = res

        # Apply restrictions to edges
        for e in n['node']['edges']:
            e['restrictions'] = res

    # If on general waypoint
    elif n['node']['name'].startswith('WayPoint'):

        # Identify structures to apply
        s = waypoint_semantics
        res = {}

        # Apply restrictions
        n['node']['semantics'] = s
        n['node']['restrictions'] = res

        # Apply restrictions to edges
        for e in n['node']['edges']:
            e['restrictions'] = res


    # If on general waypoint
    elif n['node']['name'].startswith('dock'):

        # Identify structures to apply
        s = dock_semantics
        res = {}

        # Apply restrictions
        n['node']['semantics'] = s
        n['node']['restrictions'] = res

        # Apply restrictions to edges
        for e in n['node']['edges']:
            e['restrictions'] = res


# Load tmap2 file
tmap_path = os.getenv('TMAP_FILE').replace('.tmap2.yaml', '_modified.tmap2.yaml')
with open(tmap_path) as f:
    f.write(yaml.dump(tmap))
