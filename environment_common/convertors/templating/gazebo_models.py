
class GazeboModels:

    model_1="""
    <model name='%s'>
      <static>1</static>
      <link name='primary_link'>
        <inertial>
          <pose>0 0 -1 0 -0 0</pose>
          <inertia>
            <ixx>2.05</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>2.05</iyy>
            <iyz>0</iyz>
            <izz>2.05</izz>
          </inertia>
          <mass>25</mass>
        </inertial>
        <collision name='cabinet_bottom_plate_geom'>
          <pose>0 0 0.01 0 -0 0</pose>
          <geometry>
            <box>
              <size>0.45 0.45 0.02</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='cabinet_bottom_plate_geom_visual'>
          <pose>0 0 0.01 0 -0 0</pose>
          <geometry>
            <box>
              <size>0.45 0.45 0.02</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Wood</name>
            </script>
          </material>
        </visual>
        <collision name='cabinet_bottom_plate_geom_cabinet_back_plate'>
          <pose>0.235 0 0.51 0 -0 0</pose>
          <geometry>
            <box>
              <size>0.02 0.45 1.02</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='cabinet_bottom_plate_geom_cabinet_back_plate_visual'>
          <pose>0.235 0 0.51 0 -0 0</pose>
          <geometry>
            <box>
              <size>0.02 0.45 1.02</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Wood</name>
            </script>
          </material>
        </visual>
        <collision name='cabinet_bottom_plate_geom_cabinet_left_plate'>
          <pose>0 0.235 0.51 0 -0 0</pose>
          <geometry>
            <box>
              <size>0.45 0.02 1.02</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='cabinet_bottom_plate_geom_cabinet_left_plate_visual'>
          <pose>0 0.235 0.51 0 -0 0</pose>
          <geometry>
            <box>
              <size>0.45 0.02 1.02</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Wood</name>
            </script>
          </material>
        </visual>
        <collision name='cabinet_bottom_plate_geom_cabinet_middle_plate'>
          <pose>0 0 0.51 0 -0 0</pose>
          <geometry>
            <box>
              <size>0.45 0.45 0.02</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='cabinet_bottom_plate_geom_cabinet_middle_plate_visual'>
          <pose>0 0 0.51 0 -0 0</pose>
          <geometry>
            <box>
              <size>0.45 0.45 0.02</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Wood</name>
            </script>
          </material>
        </visual>
        <collision name='cabinet_bottom_plate_geom_cabinet_right_plate'>
          <pose>0 -0.235 0.51 0 -0 0</pose>
          <geometry>
            <box>
              <size>0.45 0.02 1.02</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='cabinet_bottom_plate_geom_cabinet_right_plate_visual'>
          <pose>0 -0.235 0.51 0 -0 0</pose>
          <geometry>
            <box>
              <size>0.45 0.02 1.02</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Wood</name>
            </script>
          </material>
        </visual>
        <collision name='cabinet_bottom_plate_geom_cabinet_top_plate'>
          <pose>0 0 1.01 0 -0 0</pose>
          <geometry>
            <box>
              <size>0.45 0.45 0.02</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='cabinet_bottom_plate_geom_cabinet_top_plate_visual'>
          <pose>0 0 1.01 0 -0 0</pose>
          <geometry>
            <box>
              <size>0.45 0.45 0.02</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Wood</name>
            </script>
          </material>
        </visual>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <pose>%s %s %s %s %s %s</pose>
    </model>"""

    model_2="""
    <model name='%s'>
      <pose>%s %s %s %s %s %s</pose>
      <static>1</static>
      <link name='link'>
        <visual name='visual'>
          <geometry>
            <mesh>
              <uri>model://number1/meshes/number.dae</uri>
            </mesh>
          </geometry>
          <material>
            <script>
              <uri>model://number2/materials/scripts</uri>
              <uri>model://number2/materials/textures</uri>
              <name>Number/Two</name>
            </script>
          </material>
        </visual>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
    </model>"""


    model_3="""
    <model name='%s'>
      <link name='%s'>
        <inertial>
          <pose>0 0 0.1016 0 -0 0</pose>
          <mass>5</mass>
          <inertia>
            <ixx>0.028705</ixx>
            <ixy>0</ixy>
            <iyy>0.073708</iyy>
            <ixz>0</ixz>
            <iyz>0</iyz>
            <izz>0.06908</izz>
          </inertia>
        </inertial>
        <visual name='visual'>
          <geometry>
            <mesh>
              <uri>model://cinder_block/meshes/cinder_block.dae</uri>
            </mesh>
          </geometry>
        </visual>
        <collision name='left'>
          <pose>0 -0.08465 0.1016 0 -0 0</pose>
          <geometry>
            <box>
              <size>0.43495 0.0339 0.2032</size>
            </box>
          </geometry>
          <surface>
            <contact>
              <ode>
                <max_vel>0.1</max_vel>
                <min_depth>0.001</min_depth>
              </ode>
            </contact>
            <friction>
              <ode>
                <mu>1</mu>
                <mu2>1</mu2>
              </ode>
              <torsional>
                <ode/>
              </torsional>
            </friction>
            <bounce/>
          </surface>
          <max_contacts>10</max_contacts>
        </collision>
        <collision name='right'>
          <pose>0 0.08465 0.1016 0 -0 0</pose>
          <geometry>
            <box>
              <size>0.43495 0.0339 0.2032</size>
            </box>
          </geometry>
          <surface>
            <contact>
              <ode>
                <max_vel>0.1</max_vel>
                <min_depth>0.001</min_depth>
              </ode>
            </contact>
            <friction>
              <ode>
                <mu>1</mu>
                <mu2>1</mu2>
              </ode>
              <torsional>
                <ode/>
              </torsional>
            </friction>
            <bounce/>
          </surface>
          <max_contacts>10</max_contacts>
        </collision>
        <collision name='front'>
          <pose>0.18625 0 0.1016 0 -0 0</pose>
          <geometry>
            <box>
              <size>0.0339 0.1354 0.203</size>
            </box>
          </geometry>
          <surface>
            <contact>
              <ode>
                <max_vel>0.1</max_vel>
                <min_depth>0.001</min_depth>
              </ode>
            </contact>
            <friction>
              <ode>
                <mu>1</mu>
                <mu2>1</mu2>
              </ode>
              <torsional>
                <ode/>
              </torsional>
            </friction>
            <bounce/>
          </surface>
          <max_contacts>10</max_contacts>
        </collision>
        <collision name='middle'>
          <pose>0 0 0.1016 0 -0 0</pose>
          <geometry>
            <box>
              <size>0.0339 0.1354 0.203</size>
            </box>
          </geometry>
          <surface>
            <contact>
              <ode>
                <max_vel>0.1</max_vel>
                <min_depth>0.001</min_depth>
              </ode>
            </contact>
            <friction>
              <ode>
                <mu>1</mu>
                <mu2>1</mu2>
              </ode>
              <torsional>
                <ode/>
              </torsional>
            </friction>
            <bounce/>
          </surface>
          <max_contacts>10</max_contacts>
        </collision>
        <collision name='back'>
          <pose>-0.18625 0 0.1016 0 -0 0</pose>
          <geometry>
            <box>
              <size>0.0339 0.1354 0.203</size>
            </box>
          </geometry>
          <surface>
            <contact>
              <ode>
                <max_vel>0.1</max_vel>
                <min_depth>0.001</min_depth>
              </ode>
            </contact>
            <friction>
              <ode>
                <mu>1</mu>
                <mu2>1</mu2>
              </ode>
              <torsional>
                <ode/>
              </torsional>
            </friction>
            <bounce/>
          </surface>
          <max_contacts>10</max_contacts>
        </collision>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <pose>%s %s %s %s %s %s</pose>
    </model>"""

    model_4="""
    <model name='%s'>
      <static>1</static>
      <link name='%s'>
        <pose>0 0 1.4 0 -0 0</pose>
        <collision name='collision'>
          <geometry>
            <box>
              <size>7.5 0.2 2.8</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='visual'>
          <cast_shadows>0</cast_shadows>
          <geometry>
            <box>
              <size>7.5 0.2 2.8</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>model://grey_wall/materials/scripts</uri>
              <uri>model://grey_wall/materials/textures</uri>
              <name>vrc/grey_wall</name>
            </script>
          </material>
        </visual>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <pose>%s %s %s %s %s %s</pose>
    </model>"""
