#!/usr/bin/env python3
"""Render repository Markdown as an offline HTML report; no runtime dependencies."""
from pathlib import Path
import html
import re

ROOT=next(p for p in Path(__file__).resolve().parents if (p / 'src/legged_bringup').is_dir())
DOC=ROOT/'docs'


def inline(text):
    text=html.escape(text)
    text=re.sub(r'`([^`]+)`',r'<code>\1</code>',text)
    text=re.sub(r'\*\*([^*]+)\*\*',r'<strong>\1</strong>',text)
    text=re.sub(r'!\[([^\]]*)\]\(([^)]+)\)',r'<img alt="\1" src="\2">',text)
    text=re.sub(r'\[([^\]]+)\]\(([^)]+)\)',r'<a href="\2">\1</a>',text)
    return text


def render(text,prefix):
    lines=text.splitlines();out=[];i=0;heading=0
    while i<len(lines):
        line=lines[i]
        if line.startswith('```'):
            language=line[3:];i+=1;body=[]
            while i<len(lines) and not lines[i].startswith('```'):body.append(lines[i]);i+=1
            if language!='mermaid':out.append('<pre><code>'+html.escape('\n'.join(body))+'</code></pre>')
        elif line.startswith('#'):
            n=len(line)-len(line.lstrip('#'));heading+=1
            out.append(f'<h{n} id="{prefix}-{heading}">'+inline(line[n:].strip())+f'</h{n}>')
        elif line.startswith('|'):
            rows=[]
            while i<len(lines) and lines[i].startswith('|'):
                row=lines[i]
                if not re.match(r'^\|[\s|:\-]+$',row):rows.append([c.strip() for c in row.strip('|').split('|')])
                i+=1
            out.append('<div class="table"><table>')
            for j,row in enumerate(rows):
                tag='th' if j==0 else 'td';out.append('<tr>'+''.join(f'<{tag}>{inline(c)}</{tag}>' for c in row)+'</tr>')
            out.append('</table></div>');i-=1
        elif re.match(r'^(?:- |\d+\. )',line):
            ordered=line[0].isdigit();tag='ol' if ordered else 'ul';out.append('<'+tag+'>')
            while i<len(lines) and re.match(r'^(?:- |\d+\. )',lines[i]):
                out.append('<li>'+inline(re.sub(r'^(?:- |\d+\. )','',lines[i]))+'</li>');i+=1
            out.append('</'+tag+'>');i-=1
        elif line.strip():out.append('<p>'+inline(line)+'</p>')
        i+=1
    return '\n'.join(out)


def diagram(filename,title,subtitle,items,links,height):
    s=[f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 1120 {height}" role="img" aria-label="{html.escape(title)}">',
       '<defs><marker id="arrow" markerWidth="8" markerHeight="8" refX="7" refY="4" orient="auto"><path d="M0,0 L8,4 L0,8" fill="#68859c"/></marker></defs>',
       '<style>text{font-family:"Noto Sans CJK KR","Noto Sans",sans-serif} .title{font-size:24px;font-weight:700;fill:#102b43}.small{font-size:15px;fill:#536f83}.name{font-size:19px;font-weight:700;fill:#102b43}.desc{font-size:15px;fill:#355368}</style>',
       f'<rect width="1120" height="{height}" rx="16" fill="#f3f7fa"/>',
       f'<text x="30" y="40" class="title">{html.escape(title)}</text>',
       f'<text x="30" y="67" class="small">{html.escape(subtitle)}</text>']
    for path in links:s.append(f'<path d="{path}" fill="none" stroke="#68859c" stroke-width="2.3" marker-end="url(#arrow)"/>')
    for x,y,w,h,label,lines,color in items:
        s += [f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="10" fill="{color}" stroke="#aec5d6"/>',
              f'<text x="{x+16}" y="{y+30}" class="name">{html.escape(label)}</text>']
        for j,line in enumerate(lines):s.append(f'<text x="{x+16}" y="{y+56+22*j}" class="desc">{html.escape(line)}</text>')
    s.append('</svg>');(DOC/filename).write_text('\n'.join(s))


def main():
    diagram('height_pipeline.svg','PointCloud → Height Map','Scan의 원래 시각과 좌표계를 끝까지 유지합니다.',[
      (30,100,300,105,'① 입력 + 거리 필터',['XYZ / layout 검사','유한점 · 0.6–3.0 m'],'#ffffff'),
      (410,100,300,105,'② 시각별 pose',['Odometry 보간 또는 stamped TF','장착 TF 합성 → 변환 행렬'],'#eaf4ff'),
      (790,100,300,105,'③ GPU 후보 계산',['좌표 변환 → 5 cm 셀','고정 지도 읽기 · 높이 / ray 후보'],'#dff4ec'),
      (790,280,300,105,'④ GPU 확정 → 지역 GridMap',['셀별 확정 · 관측 우선','elevation / variance 발행'],'#dff4ec'),
      (410,280,300,105,'⑤ C++ 전역 누적',['유효 셀 교체 · 가림은 retain','미관측은 unknown 유지'],'#eaf4ff'),
      (30,280,300,105,'⑥ 동결 + PC 후처리',['RGHM / PCD / QC 보고서','경계 보호 · 평면 내부 잔노이즈'],'#fff0d6')],
      ['M330 152 H405','M710 152 H785','M940 205 V275','M790 332 H715','M410 332 H335'],425)
    diagram('height_snapshot.svg','GPU 높이 갱신: 읽기와 쓰기 분리','같은 scan의 모든 점은 같은 이전 지도를 읽습니다. 후보 집계가 끝난 뒤 셀별로 확정합니다.',[
      (30,100,300,105,'이전 지도 Mₖ',['재사용 snapshot · float32','높이 / 분산 / 유효성 / age'],'#eaf4ff'),
      (410,100,300,105,'점군 → 높이 후보',['동일 기준으로 이상치 판정','높이·분산 합 / 채택 수'],'#dff4ec'),
      (410,280,300,105,'점군 → ray 정리 후보',['기존 표면과 광선 모순 검사','삭제 증거 / upper bound 최솟값'],'#fff0d6'),
      (790,190,300,105,'셀별 최종 확정',['셀마다 하나의 writer','채택된 관측을 cleanup보다 우선'],'#dff4ec'),
      (410,470,300,105,'새 지도 Mₖ₊₁',['float32 · 지역 GridMap 발행','다음 scan의 이전 지도로 사용'],'#eaf4ff')],
      ['M330 152 H405','M180 205 V332 H405','M710 152 H750 V223 H785',
       'M710 332 H750 V260 H785','M940 295 V522 H715'],615)
    diagram('architecture.svg','RUBI 배포 구성','하나의 map 좌표계 · 하나의 localization · 하나의 주행 controller',[
      (30,100,300,105,'MID-360S driver',['180° cloud + IMU 보정','PointCloud2 / Imu'],'#ffffff'),
      (410,100,300,105,'Localization',['FAST-LIO 또는 GenZ-ICP','공통 map pose'],'#eaf4ff'),
      (790,100,300,105,'Height mapping',['CuPy GPU 지역 추정','C++ 전역 누적'],'#dff4ec'),
      (790,290,300,105,'Offline map release',['동결 · 후처리 · 정합 · QC','PCD + occupancy + height'],'#fff0d6'),
      (410,290,300,105,'Height global planner',['같은 Grid A* + CoT core','실시간 costmap으로 재계획'],'#eaf4ff'),
      (30,290,300,105,'실시간 장애물',['거리 제외 → ObstacleLayer','Frozen terrain과 별도 갱신'],'#ffffff'),
      (410,480,300,105,'선택한 경로 추종',['Nav2 DWB / Custom PP','/legged/cmd_raw'],'#eaf4ff'),
      (790,480,300,105,'정지 단계 → 보행 제어기',['Collision Monitor + watchdog','/cmd_vel'],'#dff4ec')],
      ['M330 152 H405','M710 152 H785','M940 205 V285','M790 342 H715',
       'M330 342 H405','M180 205 V285','M560 395 V475','M710 532 H785'],625)
    sections=[('report','REPORT.md','기술 보고서'),('install','INSTALL.md','설치'),
              ('operations','OPERATIONS.md','운영'),('timing','TIMING.md','입력·시간'),
              ('validation','VALIDATION.md','검증'),('sources','THIRD_PARTY.md','출처')]
    body=''.join(f'<section id="{key}">'+render((DOC/name).read_text(),key)+'</section>'
                 for key,name,_ in sections if (DOC/name).exists())
    nav=''.join(f'<a href="#{key}">{label}</a>' for key,_,label in sections)
    style='''*{box-sizing:border-box}html{scroll-behavior:smooth;scroll-padding-top:80px}body{margin:0;color:#20374b;background:#edf2f6;font-family:"Noto Sans CJK KR",system-ui,sans-serif;font-size:16px;line-height:1.85}header{background:#112e47;color:white;padding:42px max(24px,calc((100vw - 1100px)/2)) 34px}header h1{font-size:34px;line-height:1.4;margin:8px 0}header p{color:#c4d7e8;margin:0}header .tag{font-size:13px;letter-spacing:.13em;color:#86dbc3}nav{position:sticky;top:0;background:#fff;border-bottom:1px solid #cbd9e4;padding:12px 20px;display:flex;gap:24px;justify-content:center;z-index:2;flex-wrap:wrap}nav a{text-decoration:none;color:#1b536f;font-size:14px;font-weight:650}main{max-width:1160px;padding:28px;margin:auto}section{background:white;border:1px solid #d8e3ec;border-radius:14px;padding:36px 46px;margin-bottom:28px}h1{font-size:28px;line-height:1.5}h2{font-size:23px;margin-top:48px;border-top:1px solid #e0e9f0;padding-top:24px}h3{font-size:19px;margin-top:30px}p{margin:16px 0}a{color:#146895;overflow-wrap:anywhere}code{font-family:ui-monospace,monospace;background:#edf3f7;border-radius:4px;padding:2px 5px;font-size:.88em;overflow-wrap:anywhere}pre{padding:20px 22px;border-radius:10px;background:#112d43;color:#deedf7;overflow:auto;font-size:14px;line-height:1.6}pre code{background:none;color:inherit;padding:0;white-space:pre}table{border-collapse:collapse;width:100%;font-size:14px}th{background:#eaf1f7;text-align:left;color:#123a55}td,th{border:1px solid #d4e1eb;padding:12px 14px;vertical-align:top}.table{overflow:auto;margin:22px 0}img{display:block;width:100%;height:auto;margin:28px 0}li{margin:6px 0}.cards{display:flex;gap:14px;margin:0 0 25px}.card{flex:1;background:#fff;border:1px solid #d5e2ed;padding:18px;border-radius:12px}.card strong{display:block;color:#115c58}.small{font-size:13px;color:#637c90}@media(max-width:700px){main{padding:12px}section{padding:22px}nav{gap:12px}.cards{display:block}.card{margin-bottom:10px}header h1{font-size:27px}}@media print{nav{position:static}body{background:#fff}main{max-width:none;padding:0}section{border:0;padding:15px;break-before:page}pre{white-space:pre-wrap}.cards{display:none}}'''
    (DOC/'report.html').write_text('<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1"><title>RUBI · legged_robot_ws 배포 보고서</title><style>'+style+'</style><header><div class="tag">RUBI / FIELD DELIVERY</div><h1>Height mapping에서 주행까지</h1><p>legged_robot_ws · 구성, 시간 계약, 지도 생성, 운영과 검증</p></header><nav>'+nav+'</nav><main><div class="cards"><div class="card"><strong>독립 GPU 높이 지도</strong><span class="small">CuPy/CUDA 추정 + 전역 누적</span></div><div class="card"><strong>두 가지 주행 모드</strong><span class="small">Nav2 / Custom Pure Pursuit</span></div><div class="card"><strong>검증 범위 명시</strong><span class="small">PC 통합시험 완료 · 실물 확인 별도</span></div></div>'+body+'</main></html>')


if __name__=='__main__':main()
