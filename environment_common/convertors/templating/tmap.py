
class TMapTemplates:

    #details = {'gen_time':0, 'location':location}
    opening = """
meta:
  last_updated: {gen_time}
metric_map: {location}
name: {location}_restrict
pointset: {location}_restrict
transformation:
  child: topo_map
  parent: map
  rotation:
    w: 1.0
    x: 0.0
    y: 0.0
    z: 0.0
  translation:
    x: 0.0
    y: 0.0
    z: 0.0
nodes:""" #.format(**edge_details)

    #verts = {'vert0':'*id00vert0', 'vert1':'*id00vert1', 'vert2':'*id00vert2'}
    #node_details = {'name':name, 'location':location, 'vert': verts[type], 'x':x, 'y':y}
    node = """
- meta:
    map: {location}
    node: {name}
    pointset: {location}_restrict
  node:
    localise_by_topic: ''
    parent_frame: map
    name: {name}
    pose:
      orientation:
        w: 0.7897749049165983
        x: 0.0
        y: 0.0
        z: -0.6133967717260812
      position:
        x: {x}
        y: {y}
        z: 0.0
    properties:
      xy_goal_tolerance: 0.3
      yaw_goal_tolerance: 0.1
    restrictions_planning: {restrictions}
    restrictions_runtime: obstacleFree_1
    verts: *{vert}"""

    #edge_details = {'name':'WayPoint140', 'name2':'WayPoint142', 'action':'move_base', 'action_type':'move_base_msgs/MoveBaseGoal'}
    edges = """
    - edge_id: {name}_{name2}
      action: {action}
      action_type: {action_type}
      config: []
      fail_policy: fail
      fluid_navigation: true
      goal:
        target_pose:
          header:
            frame_id: $node.parent_frame
          pose: $node.pose
      node: {name2}
      recovery_behaviours_config: ''
      restrictions_planning: {restrictions}
      restrictions_runtime: obstacleFree_1"""

    vert_sample = """
verts:
  verts:
  - verts: &vert0
    - x: -0.13
      y:  0.213
    - x: -0.242
      y:  0.059
    - x: -0.213
      y: -0.13
    - x: -0.059
      y: -0.242
    - x:  0.13
      y: -0.213
    - x:  0.242
      y: -0.059
    - x:  0.213
      y:  0.13
    - x:  0.059
      y:  0.242
  - verts: &vert1
    - x:  0.128
      y: -0.214
    - x:  0.175
      y: -0.071
    - x:  0.148
      y:  0.118
    - x:  0.061
      y:  0.241
    - x: -0.128
      y:  0.214
    - x: -0.175
      y: 0.071
    - x: -0.148
      y: -0.118
    - x: -0.061
      y: -0.241
"""
