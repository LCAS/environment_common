
class GazeboTemplates:

    opening = """
<sdf version='1.7'>
  <world name='default'>"""

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

    gravity = """
    <gravity>0 0 -9.8</gravity>"""

    magnetic_field="""
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>"""

    atmosphere="""
    <atmosphere type='adiabatic'/>"""

    physics="""
    <physics type='ode'>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>"""

    scene="""
    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>1</shadows>
    </scene>"""

    audio="""
    <audio>
      <device>default</device>
    </audio>"""

    wind="""
    <wind/>"""

    coordinates="""
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <latitude_deg>0</latitude_deg>
      <longitude_deg>0</longitude_deg>
      <elevation>0</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>"""

    gui="""
    <gui fullscreen='0'>
      <camera name='user_camera'>
        <pose>8.64986 11.0927 27.9008 -0 1.19964 -2.775</pose>
        <view_controller>orbit</view_controller>
        <projection_type>perspective</projection_type>
      </camera>
    </gui>"""

    quick_opening = opening + light + gravity + magnetic_field + atmosphere + physics + scene + audio + wind + coordinates + gui

    # GROUND DEFINITIONS
    ground = """
    <model name='ground_plane'>
      <static>1</static>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>%s %s</size>
            </plane>
          </geometry>
          <surface>
            <contact>
              <collide_bitmask>65535</collide_bitmask>
              <ode/>
            </contact>
            <friction>
              <ode>
                <mu>100</mu>
                <mu2>50</mu2>
              </ode>
              <torsional>
                <ode/>
              </torsional>
            </friction>
            <bounce/>
          </surface>
          <max_contacts>10</max_contacts>
        </collision>
        <visual name='visual'>
          <cast_shadows>0</cast_shadows>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>%s %s</size>
            </plane>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
          </material>
        </visual>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
    </model>"""
    def get_ground(w, h):
        return GazeboTemplates.ground % (w, h, w, h)

    # MODEL DEFINITIONS
    model_opening="""
    <model name='%s'>
      <pose>%s %s %s %s %s %s</pose>
      <static>1</static>"""
    model_closing="""
    </model>"""

    # STATE DEFINITIONS
    state_opening="""
    <state world_name='default'>
      <sim_time>155 86000000</sim_time>
      <real_time>100 237517715</real_time>
      <wall_time>1708451611 185616378</wall_time>
      <iterations>99859</iterations>
      <light name='sun'>
        <pose>0 0 10 0 -0 0</pose>
      </light>"""

    state_model="""
      <model name='%s'>
        <pose>%s %s %s %s %s %s</pose>
        <scale>1 1 1</scale>
        <link name='%s'>
          <pose>%s %s %s %s %s %s</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>"""

    state_model_opening_2="""
      <model name='cinder_block'>
        <pose>-4.25139 1.07298 -6e-06 -1e-06 2.8e-05 -0</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>-4.25139 1.07298 -6e-06 -1e-06 2.8e-05 -0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>-0 0 -0 0 -0 0</acceleration>
          <wrench>-0 0 -0 0 -0 0</wrench>
        </link>
      </model>"""

    state_model_opening_3="""
      <model name='ground_plane'>
        <pose>0 0 0 0 -0 0</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>0 0 0 0 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>"""

    state_closing="""
    </state>"""

    closing="""
  </world>
</sdf>"""




