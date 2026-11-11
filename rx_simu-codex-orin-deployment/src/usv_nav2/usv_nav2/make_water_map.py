"""Rasterize a metric navigation boundary and exclusion polygons to PGM/YAML."""

import argparse
import json
import math
from pathlib import Path


def validate_polygon(polygon):
    if len(polygon) < 3 or any(len(p) != 2 for p in polygon):
        raise ValueError('A polygon needs at least three [x, y] points')
    if not all(math.isfinite(v) for p in polygon for v in p):
        raise ValueError('Polygon coordinates must be finite metres')
    area = sum(a[0] * b[1] - b[0] * a[1]
               for a, b in zip(polygon, polygon[1:] + polygon[:1]))
    if abs(area) < 1e-9:
        raise ValueError('Polygon area is zero')


def contains(x, y, polygon):
    inside = False
    for a, b in zip(polygon, polygon[1:] + polygon[:1]):
        ax, ay = a
        bx, by = b
        if (ay > y) != (by > y):
            crossing = ax + (y - ay) * (bx - ax) / (by - ay)
            if x < crossing:
                inside = not inside
    return inside


def rasterize(area):
    boundary = area['navigable_boundary']
    obstacles = area.get('exclusions', [])
    for polygon in [boundary] + obstacles:
        validate_polygon(polygon)
    resolution = float(area.get('resolution', 0.2))
    padding = float(area.get('padding', 1.0))
    if not math.isfinite(resolution) or resolution <= 0.0:
        raise ValueError('resolution must be positive')
    if not math.isfinite(padding) or padding < resolution:
        raise ValueError('padding must be at least one cell')
    origin_x = math.floor((min(p[0] for p in boundary) - padding) / resolution) * resolution
    origin_y = math.floor((min(p[1] for p in boundary) - padding) / resolution) * resolution
    width = math.ceil((max(p[0] for p in boundary) + padding - origin_x) / resolution)
    height = math.ceil((max(p[1] for p in boundary) + padding - origin_y) / resolution)
    if width * height > 4_000_000:
        raise ValueError('Map exceeds four million cells; increase resolution or reduce extent')
    # Store bottom row first internally, matching OccupancyGrid coordinates.
    pixels = bytearray(width * height)
    for row in range(height):
        y = origin_y + (row + 0.5) * resolution
        for col in range(width):
            x = origin_x + (col + 0.5) * resolution
            free = contains(x, y, boundary) and not any(contains(x, y, p) for p in obstacles)
            pixels[row * width + col] = 254 if free else 0
    return width, height, resolution, (origin_x, origin_y), pixels


def write_map(area, output_prefix, overwrite=False):
    prefix = Path(output_prefix)
    pgm = prefix.with_suffix('.pgm')
    yaml = prefix.with_suffix('.yaml')
    if not overwrite and (pgm.exists() or yaml.exists()):
        raise FileExistsError('Output exists; choose a new path or pass --overwrite')
    width, height, resolution, origin, pixels = rasterize(area)
    prefix.parent.mkdir(parents=True, exist_ok=True)
    # PGM top row is map's greatest y; map_server flips image rows on loading.
    image = b''.join(bytes(pixels[r * width:(r + 1) * width]) for r in reversed(range(height)))
    pgm.write_bytes(f'P5\n{width} {height}\n255\n'.encode('ascii') + image)
    metadata = {
        'image': pgm.name, 'mode': 'trinary', 'resolution': resolution,
        'origin': [origin[0], origin[1], 0.0], 'negate': 0,
        'occupied_thresh': 0.65, 'free_thresh': 0.25,
    }
    # JSON syntax is valid YAML and keeps this standalone tool dependency-free.
    yaml.write_text(json.dumps(metadata, indent=2) + '\n', encoding='utf-8')
    return yaml, width, height


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('area_json', help='Simple, non-self-intersecting polygons in map-frame metres')
    parser.add_argument('output_prefix', help='Output path without .pgm or .yaml suffix')
    parser.add_argument('--overwrite', action='store_true')
    args = parser.parse_args()
    with open(args.area_json, encoding='utf-8') as stream:
        area = json.load(stream)
    result, width, height = write_map(area, args.output_prefix, args.overwrite)
    print(f'Wrote {result} ({width} x {height} cells)')


if __name__ == '__main__':
    main()
