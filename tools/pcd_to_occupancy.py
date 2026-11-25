#!/usr/bin/env python3
"""Project an aligned localization PCD relative to a frozen height surface.

Free cells require height observations; missing terrain stays unknown. This is a
2.5D projection for a single surface, not 3D collision checking or registration.
"""
from pathlib import Path
import argparse
import hashlib
import json
import numpy as np
from scipy.ndimage import distance_transform_edt
import yaml


def project(points, ground, resolution, origin, minimum, maximum, support_radius):
    if (not np.isfinite([resolution, *origin, minimum, maximum, support_radius]).all()
            or resolution <= 0 or minimum <= 0 or maximum <= minimum or support_radius < 0):
        raise ValueError('Invalid projection geometry/height band')
    known = np.isfinite(ground)
    if not known.any(): raise ValueError('No observed height surface')
    image = np.full(ground.shape, 205, dtype=np.uint8)
    image[known] = 254
    distance, nearest = distance_transform_edt(~known, return_indices=True)
    surface = ground[tuple(nearest)]
    points = np.asarray(points)
    points = points[np.isfinite(points).all(axis=1)]
    xy = np.floor((points[:, :2] - origin) / resolution).astype(np.int64)
    inside = (xy[:, 0] >= 0) & (xy[:, 1] >= 0) & (xy[:, 0] < ground.shape[1]) & (xy[:, 1] < ground.shape[0])
    xy, points = xy[inside], points[inside]
    x, y = xy[:, 0], xy[:, 1]
    above = points[:, 2] - surface[y, x]
    obstacle = (above >= minimum) & (above <= maximum) & (distance[y, x] * resolution <= support_radius)
    image[y[obstacle], x[obstacle]] = 0
    return image


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for key in ('pcd','height','output'): parser.add_argument('--'+key, type=Path, required=True)
    parser.add_argument('--min-obstacle-height', type=float, default=.15)
    parser.add_argument('--max-obstacle-height', type=float, default=1.)
    parser.add_argument('--ground-support-radius', type=float, default=.20)
    args = parser.parse_args()
    if args.output.exists(): parser.error('Output directory exists; choose a new version directory')
    from rubi_heightmap_postprocess.map_io import read_snapshot
    import open3d as o3d
    snapshot = read_snapshot(args.height)
    if snapshot.frame != 'map': parser.error('Height frame must be map; this tool does not align maps')
    cloud = o3d.io.read_point_cloud(str(args.pcd))
    points = np.asarray(cloud.points)
    if not len(points): parser.error('PCD is missing or empty')
    origin = np.array(snapshot.origin) + np.array(snapshot.offset)*snapshot.resolution
    image = project(points, snapshot.grid, snapshot.resolution, origin,
                    args.min_obstacle_height, args.max_obstacle_height, args.ground_support_radius)
    args.output.mkdir(parents=True)
    pgm = args.output/'map.pgm'
    # PGM top row is highest world Y; Nav2 origin denotes the lower left boundary.
    pgm.write_bytes(f'P5\n{image.shape[1]} {image.shape[0]}\n255\n'.encode()+np.flipud(image).tobytes())
    metadata = dict(image='map.pgm',mode='trinary',resolution=snapshot.resolution,
                    origin=[float(origin[0]),float(origin[1]),0.],negate=0,occupied_thresh=.65,free_thresh=.196)
    (args.output/'map.yaml').write_text(yaml.safe_dump(metadata,sort_keys=False))
    provenance = {'pcd':str(args.pcd.resolve()),'pcd_sha256':hashlib.sha256(args.pcd.read_bytes()).hexdigest(),
        'height':str(snapshot.source),'height_sha256':snapshot.digest,
        'min_obstacle_height':args.min_obstacle_height,'max_obstacle_height':args.max_obstacle_height,
        'ground_support_radius':args.ground_support_radius,'frame':'map',
        'alignment':'Inputs must already share coordinates; visual verification required.'}
    (args.output/'projection.json').write_text(json.dumps(provenance,indent=2)+'\n')
    print(args.output.resolve()/'map.yaml')
    print('Review the projected map with the PCD and height surface before hybrid navigation.')


if __name__ == '__main__': main()
