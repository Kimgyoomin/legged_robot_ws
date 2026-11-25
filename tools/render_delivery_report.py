#!/usr/bin/env python3
"""Regenerate docs/report.html from the canonical guides.

Optional documentation tool: apt install python3-markdown. No ROS/GPU dependency.
"""
from pathlib import Path
import argparse
import html
import re


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,default=Path(__file__).resolve().parents[1])
    args=parser.parse_args()
    try:
        import markdown
    except ImportError:
        parser.error('Install python3-markdown to regenerate HTML; the delivered HTML needs no Python')
    root=args.root.resolve()
    pages=[('overview','개요','README.md'),('install','설치','docs/INSTALL.md'),
           ('operations','실물 실행','docs/OPERATIONS.md'),('replay','Rosbag 재생','docs/HEIGHT_REPLAY.md'),
           ('dds','DDS','docs/DDS.md'),
           ('tuning','튜닝','docs/TUNING.md'),('step-cost','단차 비용','docs/STEP_COST_TUNING.md'),('validation','검증','docs/DELIVERY_VALIDATION.md')]
    targets={(root/relative).resolve():anchor for anchor,_,relative in pages}
    sections=[]
    for anchor,label,relative in pages:
        source=root/relative
        content=markdown.markdown(source.read_text(),extensions=['tables','fenced_code'])
        def link(match):
            destination=html.unescape(match.group(1))
            if re.match(r'[a-zA-Z]+:',destination) or destination.startswith('#'): return match.group(0)
            file,_,fragment=destination.partition('#')
            path=(source.parent/file).resolve()
            if path in targets:return 'href="#'+targets[path]+'"'
            # Remaining repository links are relative to docs/report.html.
            import os
            target=os.path.relpath(path,root/'docs')+('#'+fragment if fragment else '')
            return 'href="'+html.escape(target,quote=True)+'"'
        content=re.sub(r'href="([^"]+)"',link,content)
        sections.append(f'<section id="{anchor}">{content}</section>')
    nav=' · '.join(f'<a href="#{a}">{label}</a>' for a,label,_ in pages)
    style='''body{margin:0;background:#f1f5f8;color:#213448;font:16px/1.8 system-ui,sans-serif}
nav{position:sticky;top:0;padding:14px;background:#fff;border-bottom:1px solid #ccd8e0;text-align:center}
main{max-width:1100px;margin:auto;padding:24px}section{background:#fff;padding:32px;margin:0 0 24px;scroll-margin-top:70px}
h1,h2,h3{line-height:1.5}h2{border-top:1px solid #dde5ea;padding-top:22px;margin-top:36px}
a{color:#146895}pre{overflow:auto;background:#142d42;color:#e6f2fa;padding:18px;line-height:1.5}
code{font-family:monospace;font-size:.9em}table{border-collapse:collapse;width:100%;display:block;overflow:auto}
th,td{border:1px solid #ccd8e0;padding:9px 12px;text-align:left}th{background:#edf4f8}
@media(max-width:700px){main{padding:8px}section{padding:16px}}@media print{nav{position:static}body{background:#fff}section{padding:0;break-before:page}}'''
    document='<!doctype html><html lang="ko"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1"><title>legged_robot_ws 운용 안내</title><style>'+style+'</style></head><body><nav>'+nav+'</nav><main>'+''.join(sections)+'</main></body></html>\n'
    output=root/'docs/report.html';output.write_text(document)
    print(output)


if __name__=='__main__':main()
