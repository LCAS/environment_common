import yaml

def getdict(path):
    with open(path) as f:
        data = f.read()
    return yaml.safe_load(data)

