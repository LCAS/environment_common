#########################

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

    def get(cls, data):
        if 'uri' in data:
            return cls.uri(data)
        if 'dae' in data:
            return cls.dae(data)

    def uri(cls, data):
        name = data['material_name']
        uri = data['material_uri']
        mat = simple_uri % (name, uri)
        return mat

    def dae(cls, data):
        name = data['material_name']
        uri_s = data['material_dae_scripts']
        uri_t = data['material_dae_textures']
        mat = simple_dae % (name, dae)
        return mat
