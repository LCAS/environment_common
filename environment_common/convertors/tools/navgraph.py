import yaml

# Possible directional tags for the connections:
# !bidir (bidirectional, default), !dir (directed)

constructors = ['!unconnected', '!split-intersection', '!dir']
def getroot(path):
    with open(path) as f:
        data = f.read()
    for c in constructors:
        data = data.replace(c, '')
    return yaml.safe_load(data)

