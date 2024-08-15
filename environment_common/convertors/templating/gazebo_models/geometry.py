#########################

sphere_geometry = """
          <geometry>
            <sphere>
              <radius>%s</radius>
            </sphere>
          </geometry>"""

cylinder_geometry = """
          <geometry>
            <cylinder>
              <radius>%s</radius>
              <length>%s</length>
            </cylinder>
          </geometry>"""

box_geometry = """
          <geometry>
            <box>
              <size>%s %s %s</size>
            </box>
          </geometry>"""

plane_geometry = """
          <geometry>
            <plane>
              <normal>%s %s %s</normal>
              <size>%s %s</size>
            </plane>
          </geometry>"""

mesh_geometry = """
          <geometry>
            <mesh>
              <uri>%s</uri>
            </mesh>
          </geometry>"""

#########################

class Geometry:

    def get(data):
        geometries = {'sphere': Geometry.sphere,
                      'cylinder': Geometry.cylinder,
                      'box': Geometry.box,
                      'plane': Geometry.plane,
                      'mesh': Geometry.mesh}
        geometry = geometries[data['type']['reference']](data)
        return geometry

    def sphere(data):
        r = data['radius']
        geom = sphere_geometry % (r)
        return geom

    def cylinder(data):
        r, l = data['radius'], data['length']
        geom = cylinder_geometry % (r, l)
        return geom

    def box(data):
        sz = data['size']
        w, h, l = sz['x'], sz['y'], sz['z']
        geom = box_geometry % (w, h, l)
        return geom

    def plane(data):
        nx, ny, nz = 0, 0, 1
        w, h = data['width'], data['length']
        geom = plane_geometry % (nx, ny, nz, w, h)
        return geom

    def mesh(data):
        dae = data['dae']
        geom = mesh_geometry % (dae)
        return geom
