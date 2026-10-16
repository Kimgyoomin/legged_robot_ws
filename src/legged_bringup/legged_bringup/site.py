"""Read a versioned map set without guessing frame alignment."""
from pathlib import Path
import hashlib
import yaml
from .map_io import read_snapshot


def load_site(filename, require_height=True):
    path = Path(filename).expanduser().resolve()
    site = yaml.safe_load(path.read_text())
    if site.get('format') != 1 or site.get('frame') != 'map':
        raise ValueError('site.yaml requires format: 1 and frame: map')
    if not site.get('alignment', {}).get('verified', False):
        raise ValueError('Map alignment is not verified in site.yaml; complete map QC first')
    assets = {}
    required = ['occupancy', 'image', 'pcd'] + (['height'] if require_height else [])
    for key in required:
        item = site['assets'][key]
        file = (path.parent / item['path']).resolve()
        digest = hashlib.sha256(file.read_bytes()).hexdigest()
        if digest != item['sha256']:
            raise ValueError(f'{key} hash mismatch: {file}')
        assets[key] = str(file)
    occ = yaml.safe_load(Path(assets['occupancy']).read_text())
    image = (Path(assets['occupancy']).parent / occ['image']).resolve()
    if str(image) != assets['image']:
        raise ValueError('Occupancy YAML image is not the hashed image')
    if require_height:
        height = read_snapshot(assets['height'])
        if height.frame != 'map' or abs(height.resolution-site['height_resolution']) > 1e-9:
            raise ValueError('Frozen height frame/resolution does not match the site')
    return site, assets
