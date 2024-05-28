#######################

light = """
    <light name='sun' type='directional'>
      <cast_shadows>1</cast_shadows>
      <pose>0 0 10 0 -0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
      <spot>
        <inner_angle>0</inner_angle>
        <outer_angle>0</outer_angle>
        <falloff>0</falloff>
      </spot>
    </light>"""

scene = """
    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>1</shadows>
    </scene>"""


downtunnelview = """
    <gui fullscreen='0'>
      <camera name='user_camera'>
        <pose>0.3339 35.8481 3.06376 0 0.001799 -1.55494</pose>
        <view_controller>orbit</view_controller>
        <projection_type>perspective</projection_type>
      </camera>
    </gui>
"""

downtunnelortho = """
    <gui fullscreen='0'>
      <camera name='user_camera'>
        <pose>0.063391 35.9803 1.73163 0 0 -1.5708</pose>
        <view_controller>ortho</view_controller>
        <projection_type>orthographic</projection_type>
      </camera>
    </gui>
"""

outview = """
    <gui fullscreen='0'>
      <camera name='user_camera'>
        <pose>36.8821 18.6508 45.751 -0 0.877799 -2.81094</pose>
        <view_controller>orbit</view_controller>
        <projection_type>perspective</projection_type>
      </camera>
    </gui>
"""

view = downtunnelortho


#######################

class Environment:
    def get(data):
        xml = light
        xml += scene
        xml += view
        return xml


