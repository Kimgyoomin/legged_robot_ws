#!/usr/bin/env python3
"""Export a bounded, portable navigation report archive; never run ROS or motion.

Source captures remain under validation/. Only linked report assets and the
explicit evidence list below are copied. The manifest records original hashes.
"""
from pathlib import Path
import hashlib
import html
import json
import os
import re
from urllib.parse import unquote, urlsplit

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'validation'
DEST = ROOT / 'docs/navigation/reports'
MAX_BYTES = 2 * 1024 * 1024
REPORT_DIRS = [
    'tracking_20261019', 'stuck_20261019', 'narrow_20261020',
    'self_returns_20261020', 'lio_time_20261020', 'stuck_gate_20261020',
    'range_trial_20261027', 'cube_dwb_20261027', 'yaw_abort_20261027',
    'east_stuck_20261027', 'turn_choice_20261027', 'sim_crash_20261027',
    'fastlio_velocity_20261027',
]
EXTRAS = [
    'height_band_20261027/offline_check.json',
    'height_band_20261027/live_check.json',
    'height_band_20261027/upper_obstacle.json',
    'footprint_model_20261027/comparison.json',
    'footprint_model_20261027/comparison.png',
    'site_lower10_v3_20261027/bundle_check.json',
    'site_lower10_v3_20261027/launch_check.json',
    'clearance_off_20261027/launch_results.json',
    'inflation_20261027/runtime.json',
    'front_sector_20261020/sector_comparison.json',
    'front_sector_20261020/tf.json',
    'front_sector_20261020/bench.txt',
    'front_sector_20261020/bench.cpp',
    'turn_braking_20261027/initial_summary.json',
    'turn_braking_20261027/source_analysis.json',
    'fastlio_velocity_20261027/dwb_live_samples_analysis.json',
]
SUFFIXES = {'.html', '.png', '.svg', '.jpg', '.jpeg', '.json', '.yaml',
            '.yml', '.txt', '.log', '.csv', '.py', '.cpp', '.md'}
REF = re.compile(r'((?:href|src)\s*=\s*[\"\'])([^\"\']+)([\"\'])', re.I)
included, omitted = {}, {}


def digest(data):
    return hashlib.sha256(data).hexdigest()


def relative(path, parent):
    return Path(os.path.relpath(path, parent)).as_posix()


def export(path):
    path = path.resolve()
    key = path.relative_to(SOURCE).as_posix()
    output = DEST / key
    if key in included:
        return output
    reason = None
    if not path.is_file():
        reason = 'directory or missing local capture'
    elif path.suffix.lower() not in SUFFIXES:
        reason = 'raw/binary capture outside report archive'
    elif path.stat().st_size > MAX_BYTES:
        reason = 'larger than 2 MiB report asset limit'
    if reason:
        record = {'source': 'validation/' + key, 'reason': reason}
        if path.is_file():
            record.update(bytes=path.stat().st_size, sha256=digest(path.read_bytes()))
        omitted[key] = record
        return DEST / 'LOCAL_DATA.md'
    data = path.read_bytes()
    included[key] = {'source': 'validation/' + key, 'bytes': len(data),
                     'source_sha256': digest(data)}
    if path.suffix == '.html':
        def rewrite(match):
            ref = html.unescape(match[2])
            parts = urlsplit(ref)
            if parts.scheme and parts.scheme != 'file' or ref.startswith(('#', '//')):
                return match[0]
            target = (path.parent / unquote(parts.path)).resolve()
            if target.is_relative_to(SOURCE):
                exported = export(target)
            elif target.is_relative_to(ROOT) and target.exists():
                exported = target
            else:
                omitted[ref] = {'source': ref, 'reason': 'external local path'}
                exported = DEST / 'LOCAL_DATA.md'
            new_ref = relative(exported, output.parent)
            if parts.fragment and exported.name != 'LOCAL_DATA.md':
                new_ref += '#' + parts.fragment
            return match[1] + html.escape(new_ref, quote=True) + match[3]
        text = REF.sub(rewrite, data.decode('utf-8'))
        current = relative(ROOT / 'docs/navigation/README.md', output.parent)
        banner = ('<aside style="padding:12px;background:#fff3cd;color:#302600;'
                  'font:15px sans-serif;border:1px solid #ba9b39">'
                  '<strong>과거 실험 기록 보관본</strong> — 수치·설정·상태는 기록 당시 기준입니다. '
                  f'<a href="{current}">현재 인계 문서</a>를 함께 확인하세요. '
                  '링크 경로와 이 안내만 추가했으며 실험 결과는 수정하지 않았습니다.</aside>')
        text, count = re.subn(r'(<body\b[^>]*>)', lambda m: m[1] + banner,
                             text, count=1, flags=re.I)
        if not count:
            text = banner + text
        data = text.encode('utf-8')
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_bytes(data)
    included[key]['archive_sha256'] = digest(data)
    return output


def main():
    DEST.mkdir(parents=True, exist_ok=True)
    roots = [p for name in REPORT_DIRS for p in sorted((SOURCE / name).glob('*.html'))]
    if len(roots) != 15:
        raise RuntimeError(f'Expected 15 reports, found {len(roots)}; review export selection')
    for path in roots + [SOURCE / name for name in EXTRAS]:
        export(path)
    text = '''# 로컬 원본과 보고서 배포본

이 디렉터리는 2026-10-27 navigation 인계용 보고서 묶음이다. HTML의 본문 수치는
원본 그대로이며, 현재 설정 안내 배너와 이동 가능한 상대 링크만 추가했다.
원본은 workspace의 `validation/<실험명>/`에 있다.

점군/bag/대형 dump, 전체 before/ 디렉터리 등은 이 묶음에 포함하지 않는다.
미포함 데이터 링크는 이 문서로 연결했다. 도식과 작은 수치 결과는 함께 포함한다.
그림만으로 원시 데이터 전체를 재분석할 수는 없다. 원본 보유 장비의 validation
디렉터리를 별도 데이터 저장소로 복사해야 전체 재현이 가능하다.

포함·미포함 파일의 출처·크기·SHA-256은 [manifest.json](manifest.json)을 참고한다.
JSON 안의 `/home/kim/...` 경로와 PID는 실험 당시 provenance이며 실행 인수가 아니다.
배포본 생성기는 [export_navigation_reports.py](../../../tools/export_navigation_reports.py)다.
생성기는 ROS 노드를 실행하거나 주행 명령을 발행하지 않는다.

## 링크된 원본 중 이번 묶음에 포함하지 않은 자료

'''
    for record in sorted(omitted.values(), key=lambda r: r['source']):
        text += f"- `{record['source']}` — {record['reason']}\n"
    (DEST / 'LOCAL_DATA.md').write_text(text, encoding='utf-8')
    manifest = {'date': '2026-10-27', 'report_count': len(roots),
                'asset_size_limit_bytes': MAX_BYTES,
                'included': [dict(archive=k, **v) for k, v in sorted(included.items())],
                'omitted_link_targets': sorted(omitted.values(), key=lambda r: r['source'])}
    (DEST / 'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps({'reports': len(roots), 'files': len(included), 'omitted': len(omitted),
                      'bytes': sum((DEST / k).stat().st_size for k in included)}, indent=2))


if __name__ == '__main__':
    main()
