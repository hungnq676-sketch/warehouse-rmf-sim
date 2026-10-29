#!/usr/bin/env python3
"""Chuyen do thi duong di cua kho (config/*_paths.yaml) sang nav graph cua Open-RMF.

    make_rmf_nav_graph.py config/kho_30x50_paths.yaml /tmp/kho_nav_graph.yaml

- Moi node -> 1 vertex (giu ten). Node type 'charge' -> is_charger.
  Node type 'home' / 'charge' -> is_parking_spot.
- Moi canh -> 2 lane (2 chieu).
Dinh dang giong output cua `building_map_generator nav` ma fleet adapter doc.
"""
import sys

import yaml

LEVEL = 'L1'


def convert(paths, building_name='kho'):
    names = list(paths['nodes'].keys())
    index = {n: i for i, n in enumerate(names)}
    vertices = []
    for n in names:
        nd = paths['nodes'][n]
        opts = {'name': n}
        if nd.get('type') == 'charge':
            opts['is_charger'] = True
            opts['is_parking_spot'] = True
        if nd.get('type') == 'home':
            opts['is_parking_spot'] = True
        if nd.get('type') in ('station', 'charge', 'home'):
            opts['is_holding_point'] = True
        vertices.append([float(nd['x']), float(nd['y']), opts])
    lanes = []
    for a, b in paths['edges']:
        lanes.append([index[a], index[b], {'speed_limit': 0.0}])
        lanes.append([index[b], index[a], {'speed_limit': 0.0}])
    return {'building_name': building_name,
            'levels': {LEVEL: {'vertices': vertices, 'lanes': lanes}},
            'lifts': {}, 'doors': {}}


def main():
    if len(sys.argv) != 3:
        print(__doc__)
        sys.exit(1)
    with open(sys.argv[1]) as f:
        graph = convert(yaml.safe_load(f))
    with open(sys.argv[2], 'w') as f:
        yaml.safe_dump(graph, f, default_flow_style=None, sort_keys=False)
    lv = graph['levels'][LEVEL]
    print(f"{len(lv['vertices'])} vertex, {len(lv['lanes'])} lane -> {sys.argv[2]}")


if __name__ == '__main__':
    main()
