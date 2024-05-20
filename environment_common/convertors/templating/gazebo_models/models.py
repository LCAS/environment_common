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

        # Geometry
        geometry = Geometry.get(data)

        # Model
        xml = model % (name, x,y,z,ro,pi,ya, geometry, material)
        return xml
