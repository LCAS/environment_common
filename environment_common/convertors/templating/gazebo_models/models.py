import numpy as np

#######################

model = """
    <model name='%s'>
      <pose>%s %s %s %s %s %s</pose>
      <static>1</static>
      <link name='link'>
        <visual name='visual'> %s %s
        </visual>
      </link>
    </model>"""

#######################

from .materials import Material
from .geometry import Geometry

class Model:
    def get(data):
        name = data['name']
        material = Material.get(data)

        # Position
        pos = data['position']
        x,y,z = pos['x'], pos['y'], pos['z']

        # Orientation
        ori = data['orientation']
        ro,pi,ya = ori['roll'], ori['pitch'], ori['yaw']

        # Anchor
        if 'anchor' in data:
            Ax, Ay = data['anchor']['position']['x'], data['anchor']['position']['y']
            Ayaw = data['anchor']['orientation']['yaw']
            Nx, Ny, Nyaw = Model.transform_pose(x, y, ya, Ax, Ay, Ayaw)
            x, y, ya = Nx, Ny, Nyaw

        # Geometry
        geometry = Geometry.get(data)

        # Model
        xml = model % (name, x,y,z,ro,pi,ya, geometry, material)
        return xml

    def transform_pose(x, y, yaw, Ax, Ay, Ayaw):

        # Calculate relative position
        x_rel = x - Ax
        y_rel = y - Ay

        # Create the rotation matrix for the anchor's yaw
        cos_psi_a = np.cos(Ayaw)
        sin_psi_a = np.sin(Ayaw)

        rotation_matrix = np.array([
            [cos_psi_a, -sin_psi_a],
            [sin_psi_a, cos_psi_a]
        ])

        # Apply rotation to relative position
        x_y_rel_rotated = np.dot(rotation_matrix, np.array([x_rel, y_rel]))

        # Calculate new values
        x_new = x_y_rel_rotated[0]
        y_new = x_y_rel_rotated[1]
        yaw_new = yaw + Ayaw

        # Return the new pose
        return x_new, y_new, yaw_new
