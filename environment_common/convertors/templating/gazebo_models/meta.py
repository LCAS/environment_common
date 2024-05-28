#######################

opening = """
<sdf version='1.7'>
  <world name='default'>"""

closing = """
  </world>
</sdf>"""

#######################

from .models import Model
from .environmental import Environment

class Meta:
    def get(data):
        xml = Meta.opening(data)

        xml += Environment.get(data)

        for i, m in enumerate(data['components']):
            m['name'] = str(i)+'_'+m['type']['reference']
            print(m['name'])
            xml += Model.get(m)

        xml += Meta.closing(data)
        return xml

    def opening(data):
        xml = opening
        return xml

    def scene(data):
        xml = scene
        return xml

    def closing(data):
        xml = closing
        return xml
