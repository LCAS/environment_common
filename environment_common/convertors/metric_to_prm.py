import sys, os
from ament_index_python.packages import get_package_prefix

import yaml
from pprint import pprint

from PIL import Image
import numpy as np
import random
import matplotlib.pyplot as plt
import networkx as nx
from scipy.spatial import KDTree


def is_free(x, y, occupancy):
    h, w = occupancy.shape
    if 0 <= x < w and 0 <= y < h:
        return occupancy[y, x] > 250
    return False

def collision_free(p1, p2, occupancy):
    x0, y0 = map(int, p1)
    x1, y1 = map(int, p2)
    num = max(abs(x1 - x0), abs(y1 - y0))
    if num == 0:
        return False  # Skip zero-length edges
    for i in range(num + 1):
        t = i / num
        x = int(x0 + t * (x1 - x0))
        y = int(y0 + t * (y1 - y0))
        if not is_free(x, y, occupancy):
            return False
    return True

def sample_free_points(occupancy, num_samples):
    h, w = occupancy.shape
    samples = []
    while len(samples) < num_samples:
        x = random.randint(0, w - 1)
        y = random.randint(0, h - 1)
        if is_free(x, y, occupancy):
            samples.append((x, y))
    return np.array(samples)


def generate_prm(params):
    datas = params['datas']
    image_size = params['image_size']
    num_samples = params['num_samples']
    k = params['k']
    max_dist = params['max_dist']

    img_array = np.array(datas).reshape((image_size[1], image_size[0], 4))
    grayscale = img_array[:, :, 0]
    occupancy = grayscale

    roadmap = nx.Graph()
    samples = sample_free_points(occupancy, num_samples)

    # ✅ Use tuples for node positions
    roadmap.add_nodes_from((i, {'pos': tuple(p)}) for i, p in enumerate(samples))

    tree = KDTree(samples)
    for i, p1 in enumerate(samples):
        distances, indices = tree.query(p1, k=k+1)
        for dist, j in zip(distances[1:], indices[1:]):
            if dist > max_dist:
                continue
            p2 = samples[j]
            if not roadmap.has_edge(i, j) and collision_free(p1, p2, occupancy):
                roadmap.add_edge(i, j, weight=dist)

    node_list = [{'id': int(n), 'x': int(p[0]), 'y': int(p[1])} for n, p in enumerate(samples)]
    edge_list = [{'from': int(u), 'to': int(v), 'weight': float(d['weight'])} for u, v, d in roadmap.edges(data=True)]
    yaml_output = yaml.dump({'nodes': node_list, 'edges': edge_list}, sort_keys=False)

    return yaml_output, roadmap, samples, occupancy


def draw_prm_overlay(samples, roadmap, occupancy, save_path):
    """Visualize and save the PRM overlayed on the map."""
    plt.figure(figsize=(10, 10))
    plt.imshow(occupancy, cmap='gray')

    for u, v in roadmap.edges():
        x1, y1 = samples[u]
        x2, y2 = samples[v]
        plt.plot([x1, x2], [y1, y2], 'g-', linewidth=0.5)

    plt.scatter(samples[:, 0], samples[:, 1], c='r', s=3)
    plt.gca().invert_yaxis()
    plt.axis('off')
    plt.tight_layout()
    plt.savefig(save_path, dpi=300)
    plt.close()
    print(f"[✔] Saved PRM overlay to: {save_path}")


def run(args=None):
    # Load the map.yaml file
    mapyaml_path = os.path.join(args['src'], 'config', 'metric', 'map', 'map.yaml')
    if not os.path.isfile(mapyaml_path):
        mapyaml_path = os.path.join(args['src'], 'config', 'metric', 'map', 'map_autogen.yaml')

    with open(mapyaml_path) as f:
        mapyaml = yaml.safe_load(f)

    # Load the map image
    img_path = os.path.join(args['src'], 'config', 'metric', 'map', mapyaml['image'])
    img = Image.open(img_path).convert("RGBA")
    datas = list(img.getdata())

    # Generate PRM
    params = {
        'datas': datas,
        'image_size': img.size,
        'num_samples': 1000,
        'k': 20,
        'max_dist': 200,
    }
    prm_yaml, roadmap, samples, occupancy = generate_prm(params)

    # Save YAML
    prm_path = os.path.join(args['src'], 'config', 'topological', 'prm_autogen.yaml')
    os.makedirs(os.path.dirname(prm_path), exist_ok=True)
    with open(prm_path, 'w') as f:
        f.write(prm_yaml)
    print(f"[✔] Saved PRM YAML to: {prm_path}")

    # Save PNG overlay
    overlay_path = prm_path.replace('.yaml', '_overlay.png')
    draw_prm_overlay(samples, roadmap, occupancy, overlay_path)


def main(args=None):
    pkg_name = 'environment_template'
    src = '/'.join(get_package_prefix(pkg_name).split('/')[:-2]) + f'/src/{pkg_name}'
    args = {'src': src}
    run(args)


if __name__ == '__main__':
    main()
