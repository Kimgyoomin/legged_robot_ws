#!/usr/bin/env python3
"""Copy the legacy RUBI map pair without overwriting files or approving map QC."""
import argparse
import hashlib
from pathlib import Path
import shutil

import yaml


def digest(path):
    checksum = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            checksum.update(block)
    return checksum.hexdigest()


def contained_path(root, relative):
    if Path(relative).is_absolute():
        raise ValueError(f'Expected a relative asset path: {relative}')
    path = (root / relative).resolve()
    if not path.is_relative_to(root):
        raise ValueError(f'Asset path escapes its root: {relative}')
    return path


def import_maps(source_root, site_file):
    source_root = Path(source_root).expanduser().resolve()
    site_file = Path(site_file).expanduser().resolve()
    site = yaml.safe_load(site_file.read_text())
    jobs = []
    # Validate the entire set before creating any files/directories.
    for key in ('pcd', 'occupancy', 'image'):
        asset = site['assets'][key]
        source = contained_path(source_root, site['provenance']['source_assets'][key])
        target = contained_path(site_file.parent, asset['path'])
        if digest(source) != asset['sha256']:
            raise ValueError(f'Source hash mismatch ({key}): {source}')
        if target.exists() and digest(target) != asset['sha256']:
            raise ValueError(f'Refusing to overwrite different data: {target}')
        jobs.append((source, target, asset['sha256']))
    for source, target, expected in jobs:
        if not target.exists():
            target.parent.mkdir(parents=True, exist_ok=True)
            # Exclusive creation also prevents overwriting a concurrently created file.
            with source.open('rb') as src, target.open('xb') as dst:
                shutil.copyfileobj(src, dst)
        if digest(target) != expected:
            raise ValueError(f'Copied file hash mismatch: {target}')
    return [target for _, target, _ in jobs]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-root', required=True,
                        help='Legacy navigation_real root, or an asset archive with the same layout')
    parser.add_argument('--site', type=Path, default=(
        Path(__file__).resolve().parents[1] / 'maps/rubi_1f_legacy/site.yaml'))
    args = parser.parse_args()
    try:
        targets = import_maps(args.source_root, args.site)
    except (OSError, ValueError, KeyError, yaml.YAMLError) as exc:
        parser.exit(1, f'Map import failed: {exc}\n')
    for target in targets:
        print(f'Verified: {target}')
    print('Source files and site alignment status were not changed.')


if __name__ == '__main__':
    main()
