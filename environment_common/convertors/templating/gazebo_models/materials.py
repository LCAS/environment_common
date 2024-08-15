#########################
# Reference: https://wiki.ros.org/simulator_gazebo/Tutorials/ListOfMaterials

simple_uri = """
          <material>
            <script>
              <name>%s</name>
              <uri>%s</uri>
            </script>
          </material>"""

simple_dae = """
         <material>
            <script>
              <name>%s</name>
              <uri>%s</uri>
              <uri>%s</uri>
            </script>
          </material>"""

#########################

class Material:

    def get(data):
        if 'uri' in data['material']:
            return Material.uri(data)
        if 'dae' in data['material']:
            return Material.dae(data)

    def uri(data):
        name = data['material']['name']
        uri = data['material']['uri']
        mat = simple_uri % (name, uri)
        return mat

    def dae(data):
        name = data['material']['name']
        uri_s = data['material']['dae_scripts']
        uri_t = data['material']['dae_textures']
        mat = simple_dae % (name, dae)
        return mat
