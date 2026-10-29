#!/usr/bin/env python3
"""Tao ban do 2D (PGM + YAML cho Nav2 map_server) tu collision cua kho URDF.

Khong can chay SLAM: moi khoi collision (box / cylinder) co phan nam trong dai
do cao [z_min, z_max] duoc chieu xuong mat san va to den.

    ros2 run warehouse_world make_map.py --urdf kho.urdf --out maps/kho_30x50
    (hoac de trong: tu chay xacro tren kho_30x50.urdf.xacro)
"""
import argparse
import math
import os
import subprocess
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image, ImageDraw


def rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr]])


def parse_origin(el):
    o = el.find('origin') if el is not None else None
    xyz = [0.0, 0.0, 0.0]
    rpy = [0.0, 0.0, 0.0]
    if o is not None:
        if 'xyz' in o.attrib:
            xyz = [float(v) for v in o.attrib['xyz'].split()]
        if 'rpy' in o.attrib:
            rpy = [float(v) for v in o.attrib['rpy'].split()]
    T = np.eye(4)
    T[:3, :3] = rpy_matrix(*rpy)
    T[:3, 3] = xyz
    return T


def link_transforms(robot):
    """Bien doi tu link goc toi moi link (chuoi khop fixed)."""
    parent_of = {}
    for j in robot.findall('joint'):
        parent_of[j.find('child').attrib['link']] = (j.find('parent').attrib['link'], parse_origin(j))
    cache = {}

    def tf(link):
        if link in cache:
            return cache[link]
        if link not in parent_of:
            cache[link] = np.eye(4)
        else:
            parent, T = parent_of[link]
            cache[link] = tf(parent) @ T
        return cache[link]
    return {l.attrib['name']: tf(l.attrib['name']) for l in robot.findall('link')}


def shape_corners(geom):
    """Cac diem bien (trong khung collision) cua box / cylinder / sphere."""
    box = geom.find('box')
    if box is not None:
        sx, sy, sz = (float(v) / 2 for v in box.attrib['size'].split())
        return np.array([[x, y, z] for x in (-sx, sx) for y in (-sy, sy) for z in (-sz, sz)])
    cyl = geom.find('cylinder')
    if cyl is not None:
        r, h = float(cyl.attrib['radius']), float(cyl.attrib['length']) / 2
        return np.array([[r * math.cos(a), r * math.sin(a), z]
                         for a in np.linspace(0, 2 * math.pi, 16, endpoint=False) for z in (-h, h)])
    sph = geom.find('sphere')
    if sph is not None:
        r = float(sph.attrib['radius'])
        return np.array([[r * math.cos(a), r * math.sin(a), z]
                         for a in np.linspace(0, 2 * math.pi, 16, endpoint=False) for z in (-r, r)])
    return None


def convex_hull(points):
    pts = sorted(set(map(tuple, np.round(points, 4))))
    if len(pts) <= 2:
        return pts

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--urdf', help='file URDF da qua xacro (mac dinh: tu chay xacro)')
    ap.add_argument('--xacro', default=None)
    ap.add_argument('--out', required=True, help='duong dan khong duoi, vd maps/kho_30x50')
    ap.add_argument('--resolution', type=float, default=0.05)
    ap.add_argument('--z-min', type=float, default=0.03, help='bo qua vat thap hon (vach son)')
    ap.add_argument('--z-max', type=float, default=1.8)
    ap.add_argument('--extra-box', action='append', default=[],
                    help='them vat can "x0,y0,x1,y1" (vd mat bang tai la model rieng)')
    ap.add_argument('--margin', type=float, default=1.0)
    a = ap.parse_args()

    if a.urdf:
        robot = ET.parse(a.urdf).getroot()
    else:
        robot = ET.fromstring(subprocess.check_output(['xacro', a.xacro]))

    tfs = link_transforms(robot)
    polys = []
    xmin = ymin = math.inf
    xmax = ymax = -math.inf
    for link in robot.findall('link'):
        Tl = tfs[link.attrib['name']]
        for col in link.findall('collision'):
            corners = shape_corners(col.find('geometry'))
            if corners is None:
                continue
            T = Tl @ parse_origin(col)
            w = (T[:3, :3] @ corners.T).T + T[:3, 3]
            xmin, xmax = min(xmin, w[:, 0].min()), max(xmax, w[:, 0].max())
            ymin, ymax = min(ymin, w[:, 1].min()), max(ymax, w[:, 1].max())
            if w[:, 2].max() < a.z_min or w[:, 2].min() > a.z_max:
                continue
            polys.append(convex_hull(w[:, :2]))
    for b in a.extra_box:
        x0, y0, x1, y1 = (float(v) for v in b.split(','))
        polys.append([(x0, y0), (x1, y0), (x1, y1), (x0, y1)])

    ox, oy = math.floor(xmin - a.margin), math.floor(ymin - a.margin)
    W = int(math.ceil((xmax + a.margin - ox) / a.resolution))
    H = int(math.ceil((ymax + a.margin - oy) / a.resolution))
    img = Image.new('L', (W, H), 254)
    draw = ImageDraw.Draw(img)
    for poly in polys:
        pix = [((x - ox) / a.resolution, H - (y - oy) / a.resolution) for x, y in poly]
        draw.polygon(pix, fill=0)

    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    img.save(a.out + '.pgm')
    with open(a.out + '.yaml', 'w') as f:
        f.write(f'image: {os.path.basename(a.out)}.pgm\n'
                f'mode: trinary\nresolution: {a.resolution}\n'
                f'origin: [{ox:.3f}, {oy:.3f}, 0.0]\n'
                'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
    print(f'{len(polys)} vat can -> {a.out}.pgm ({W}x{H}, goc {ox},{oy})')


if __name__ == '__main__':
    main()
