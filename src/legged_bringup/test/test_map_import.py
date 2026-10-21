"""Map import is relocatable, checks hashes, and never silently replaces data."""
import hashlib
import importlib.util
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[3]
SPEC = importlib.util.spec_from_file_location('map_import', ROOT / 'tools/import_legacy_rubi_maps.py')
IMPORTER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(IMPORTER)


@pytest.fixture
def bundle(tmp_path):
    source = tmp_path / 'original'
    site_dir = tmp_path / 'relocated_workspace/maps/site'
    source.mkdir()
    site_dir.mkdir(parents=True)
    site = {'alignment': {'verified': False}, 'assets': {},
            'provenance': {'source_assets': {}}}
    for key in ('pcd', 'occupancy', 'image'):
        content = f'synthetic {key}'.encode()
        (source / key).write_bytes(content)
        site['assets'][key] = {'path': f'data/{key}',
                               'sha256': hashlib.sha256(content).hexdigest()}
        site['provenance']['source_assets'][key] = key
    manifest = site_dir / 'site.yaml'
    manifest.write_text(yaml.safe_dump(site))
    return source, manifest


def test_import_is_exact_idempotent_and_preserves_qc(bundle):
    source, manifest = bundle
    original_manifest = manifest.read_bytes()
    targets = IMPORTER.import_maps(source, manifest)
    assert targets == IMPORTER.import_maps(source, manifest)
    for target in targets:
        assert target.read_bytes() == (source / target.name).read_bytes()
    assert manifest.read_bytes() == original_manifest


@pytest.mark.parametrize('problem', ['hash', 'missing', 'escape'])
def test_preflight_failure_creates_no_data(bundle, problem):
    source, manifest = bundle
    if problem == 'hash':
        (source / 'image').write_bytes(b'wrong')
    elif problem == 'missing':
        (source / 'image').unlink()
    else:
        site = yaml.safe_load(manifest.read_text())
        site['assets']['image']['path'] = '../outside'
        manifest.write_text(yaml.safe_dump(site))
    with pytest.raises((ValueError, FileNotFoundError)):
        IMPORTER.import_maps(source, manifest)
    assert not (manifest.parent / 'data').exists()


def test_existing_different_data_is_preserved(bundle):
    source, manifest = bundle
    target = manifest.parent / 'data/image'
    target.parent.mkdir()
    target.write_bytes(b'keep this')
    with pytest.raises(ValueError, match='Refusing to overwrite'):
        IMPORTER.import_maps(source, manifest)
    assert target.read_bytes() == b'keep this'
    assert not (target.parent / 'pcd').exists()
