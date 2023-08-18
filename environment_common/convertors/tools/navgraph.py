import yaml

# Possible directional tags for the connections:
# !bidir (bidirectional, default), !dir (directed)

node_constructors = ['unconnected']
edge_constructors = ['split-intersection', 'dir']
def getroot(path):
    with open(path) as f:
        data = f.read()
    for c in node_constructors:
        data = data.replace(c, '')
    for c in edge_constructors:
        data = data.replace('!'+c+' [', '['+c+', ')
    return yaml.safe_load(data)

