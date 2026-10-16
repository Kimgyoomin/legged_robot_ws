"""Standalone plots and HTML; shared scales and explicit provenance/limits."""
import html
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import Polygon
from .qc import median_residual


def write_report(snapshot, result, metrics, directory, rois):
    directory = Path(directory)
    font = Path('/usr/share/fonts/truetype/nanum/NanumGothic.ttf')
    # Plot labels are English for portability; Korean HTML uses system fonts.
    if font.exists():
        font_manager.fontManager.addfont(str(font))
    z = snapshot.grid; valid = np.isfinite(z); center = np.median(z[valid])
    x, y = snapshot.xy(); r = snapshot.resolution
    extent = [x.min()-r/2, x.max()+r/2, y.min()-r/2, y.max()+r/2]
    methods = [('raw', z), ('baseline', result['baseline']), ('candidate', result['candidate'])]
    titles = ['Frozen original', 'Circular mean (6 cm default)', 'Protected piecewise/local planes']
    fig, axes = plt.subplots(2, 3, figsize=(15, 10), layout='constrained')
    for col, ((name, a), title) in enumerate(zip(methods, titles)):
        im = axes[0, col].imshow((a-center)*100, origin='lower', extent=extent, cmap='viridis', vmin=-10, vmax=20)
        axes[0, col].set_title(title)
        fig.colorbar(im, ax=axes[0, col], shrink=.7, label='Height relative to raw median [cm]', extend='both')
        # All observed cells shown, while numerical roughness excludes raw-selected edges.
        im = axes[1, col].imshow(median_residual(a)*1000, origin='lower', extent=extent, cmap='RdBu_r', vmin=-3, vmax=3)
        p95 = metrics['methods'][name]['interior_local_median_abs_residual_p95_m']
        axes[1, col].set_title(f'Interior roughness P95: {p95*1000:.3f} mm' if p95 is not None else 'No interior cells')
        fig.colorbar(im, ax=axes[1, col], shrink=.7, label='3x3 median residual [mm]', extend='both')
    for ax in axes.flat:
        ax.set_xlabel('map x [m]'); ax.set_ylabel('map y [m]'); ax.set_facecolor('#e8edf2')
    fig.suptitle('Same frozen map / same coordinates, observed mask and color scales\nBroad deformation is retained; residual colors also include protected boundaries')
    fig.savefig(directory/'comparison.png', dpi=145); plt.close(fig)

    fig, axes = plt.subplots(1, 3, figsize=(15, 5.7), layout='constrained')
    for ax, key in zip(axes[:2], ['baseline', 'candidate']):
        im = ax.imshow((result[key]-z)*1000, origin='lower', extent=extent, cmap='RdBu_r', vmin=-10, vmax=10)
        ax.set_title(key+' - original')
        fig.colorbar(im, ax=ax, shrink=.7, label='Height change [mm]', extend='both')
    classified = result['method'].astype(float)
    classified[result['protected']] = 3
    classified[~valid] = np.nan
    from matplotlib.colors import ListedColormap, BoundaryNorm
    cmap = ListedColormap(['#c5bebb', '#2b83ba', '#53aa70', '#c87448'])
    im = axes[2].imshow(classified, origin='lower', extent=extent, cmap=cmap, norm=BoundaryNorm([-.5,.5,1.5,2.5,3.5], 4))
    axes[2].set_title('Candidate processing mask')
    cb = fig.colorbar(im, ax=axes[2], shrink=.7, ticks=[0,1,2,3])
    cb.ax.set_yticklabels(['Unresolved', 'Regional plane', 'Local plane', 'Protected'])
    for ax in axes:
        ax.set_xlabel('map x [m]'); ax.set_ylabel('map y [m]'); ax.set_facecolor('#e8edf2')
    fig.savefig(directory/'changes.png', dpi=145); plt.close(fig)

    # Fixed-height cross sections through the requested step ROIs, nearest cells.
    step_rois = [item for item in rois if item['kind'] == 'step']
    if step_rois:
        fig, axes = plt.subplots(1, len(step_rois), figsize=(5*len(step_rois), 4), squeeze=False, layout='constrained')
        for ax, item in zip(axes[0], step_rois):
            t = np.arange(-.85, .8501, r/2); angle = np.deg2rad(item.get('yaw_deg', 0.))
            px = item['center_xy_m'][0]+t*np.cos(angle); py = item['center_xy_m'][1]+t*np.sin(angle)
            ix = np.floor((px-snapshot.origin[0])/r).astype(int)-snapshot.offset[0]
            iy = np.floor((py-snapshot.origin[1])/r).astype(int)-snapshot.offset[1]
            inside = (ix>=0)&(iy>=0)&(ix<z.shape[1])&(iy<z.shape[0])
            raw_roi = next(v for v in metrics['methods']['raw']['rois'] if v['name']==item['name'])
            if raw_roi['status'] != 'measured':
                ax.set_title(item['name']+': insufficient observations'); continue
            a,b,c = raw_roi['plane_coeff_local_uv']
            for (name, heights), color in zip(methods, ['#202a33','#dc8841','#277aaa']):
                section = np.full(t.shape, np.nan)
                # Subtract the SAME original reference plane for every method.
                section[inside] = heights[iy[inside],ix[inside]]-(a*t[inside]+c)
                ax.plot(t, section*100, label=name, color=color, linewidth=1.6)
            ax.set_title('Step '+item['name']); ax.set_xlabel('Distance through center [m]')
            ax.set_ylabel('Above original local plane [cm]'); ax.legend(); ax.grid(alpha=.2)
        fig.savefig(directory/'step_profiles.png', dpi=150); plt.close(fig)

    # Local floor zoom, still drawn from the same original selection.
    floors = [item for item in rois if item['kind']=='floor']
    if floors:
        item = floors[0]; cx,cy=item['center_xy_m']; hx,hy=item['core_half_extent_m']
        fig, axes = plt.subplots(1, 3, figsize=(12,4), layout='constrained')
        for ax, (name, heights) in zip(axes, methods):
            im=ax.imshow(median_residual(heights)*1000, origin='lower', extent=extent, cmap='RdBu_r', vmin=-3,vmax=3, interpolation='nearest')
            ax.set_xlim(cx-hx,cx+hx); ax.set_ylim(cy-hy,cy+hy); ax.set_title(name)
            ax.set_xlabel('map x [m]'); ax.set_ylabel('map y [m]')
            fig.colorbar(im,ax=ax,shrink=.7,label='Median residual [mm]')
        fig.suptitle('Fixed floor ROI '+item['name']+' / same +/-3 mm scale')
        fig.savefig(directory/'floor_zoom.png',dpi=170);plt.close(fig)

    def mm(value): return 'N/A' if value is None else f'{value*1000:.3f}'
    rows = ''.join('<tr><td>'+html.escape(name)+'</td><td>'+mm(v['interior_local_median_abs_residual_p95_m'])+'</td><td>'+str(v['changed_cells_gt_1um'])+'</td><td>'+mm(v['abs_change_max_m'])+'</td><td>'+str(v['protected_cells_changed_gt_1um'])+'</td></tr>' for name,v in metrics['methods'].items())
    roi_rows = ''
    for i,item in enumerate(rois):
        cells=[]
        for name,_ in methods:
            q=metrics['methods'][name]['rois'][i]
            value=(q['relative_height_p05_p50_p95_m'][1]*100 if item['kind']=='step' else q['core_plane_abs_residual_p95_m']*1000) if q['status']=='measured' else None
            cells.append('N/A' if value is None else f'{value:.3f}')
        label=item['name']+(' 단차 [cm]' if item['kind']=='step' else ' 평면 잔차 P95 [mm]')
        roi_rows+='<tr><td>'+html.escape(label)+'</td>'+''.join('<td>'+c+'</td>' for c in cells)+'</tr>'
    pictures='<img src="step_profiles.png" alt="단차 단면 비교">' if step_rois else ''
    floor_picture='<img src="floor_zoom.png" alt="바닥 내부 확대">' if floors else ''
    page='''<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>RUBI 후처리 비교</title>
<style>body{margin:0;background:#eef3f7;color:#193044;font:17px/1.7 system-ui,sans-serif}main{max-width:1250px;padding:32px;margin:auto}h1{font-size:32px}h2{margin-top:38px}img{width:100%;background:white;border-radius:10px;margin:14px 0}table{width:100%;border-collapse:collapse;background:white}td,th{padding:12px;text-align:left;border-bottom:1px solid #dbe3ea}code{overflow-wrap:anywhere;font-size:.86em}.box{padding:24px;background:white;border-radius:12px}a{color:#1565ab}</style><main>
<p>RUBI · offline comparison · experimental 0.1.0</p><h1>잔노이즈와 단차 경계의 후처리 비교</h1>
<p class="box">입력 원본은 변경하지 않았습니다. 원본·기본 필터·평면 기반 결과 모두 같은 XY, 해상도, 관측 셀을 사용합니다. 이 보고서는 자동 보행 적합 판정이 아닌 후처리 비교입니다.</p>
<h2>바닥 내부의 작은 편차</h2><p>통계는 원본에서 정한 경계 제외 영역을 모든 방법에 동일하게 적용했습니다. 이전 보고서의 전체 3×3 관측 영역 P95 4.1 mm와 평가 마스크가 다릅니다. 값이 작아져도 전체 바닥의 절대 정확도가 좋아졌다는 뜻은 아닙니다.</p>
<table><tr><th>방법</th><th>국소 잔차 P95 [mm]</th><th>변경 셀 (&gt;1 µm)</th><th>최대 변경 [mm]</th><th>보호 영역 변경 셀</th></tr>__ROWS__</table>
__FLOOR__<img src="comparison.png" alt="동일 색상 범위의 세 지도와 국소 잔차">
<h2>어디를 어떻게 바꿨는지</h2><img src="changes.png" alt="높이 변경과 보정 방식 마스크">
<p>평면에 잘 맞는 연결 영역은 해당 평면으로 보정했습니다. 큰 영역이 단일 평면 검사에 실패하면 작은 이웃 평면으로 제한된 노이즈 완화만 수행했습니다. 모든 candidate 변경은 설정된 최대 변경량 이내이며 보호된 경계·미관측 영역은 그대로입니다. 보호 영역에는 잡음이 남을 수 있습니다.</p>
<p>baseline은 grid_map의 MeanInRadiusFilter에 해당하는 원형 평균과 원본 관측 마스크 복원을 사용합니다. 전체 inpainting 체인이나 ETH convex_plane_decomposition을 실행한 결과는 아닙니다. candidate는 이번에 구현한 독립적인 평면 기반 필터입니다.</p>
<h2>고정 ROI의 단차와 평면 비교</h2><table><tr><th>지표</th><th>원본</th><th>기본 필터</th><th>평면 기반</th></tr>__ROI_ROWS__</table>__PROFILES__
<p>단차는 주변 바닥 평면 대비 내부 중앙 높이입니다. 바닥 지표는 선택한 영역 자체에 맞춘 평면의 잔차입니다. 정답 world와 정밀 정합한 RMSE가 아니며 ROI 선택은 <code>qc_regions.json</code>에 보존했습니다.</p>
<h2>남아 있는 넓은 높이 변화</h2><p>잔노이즈와 넓은 휨은 별개입니다. 호 형태나 완만한 높이 변화는 이번 제한된 후처리로 평평하게 만들지 않았습니다. 실제 경사와 누적 자세 오차를 구분하려면 원본 점군·궤적 또는 확인된 평면 영역이 추가로 필요합니다. 입력은 world 정답을 이용하지 않았습니다.</p>
<h2>저장 결과와 재현</h2><p><a href="candidate/map.rghm">candidate/map.rghm</a> · <a href="candidate/surface.pcd">candidate/surface.pcd</a> · <a href="metrics.json">전체 지표·파라미터·해시</a> · <a href="layers.npz">레이어 배열</a></p>
<p>처리본의 분산과 시각은 원본 값을 보존한 출처 정보입니다. 수정한 높이의 오차 분산을 새로 검증한 값은 아닙니다. <code>metrics.json</code>과 레이어 마스크를 함께 보관하십시오.</p><p>원본: <code>__INPUT__</code></p></main></html>'''
    page=page.replace('__ROWS__',rows).replace('__ROI_ROWS__',roi_rows).replace('__PROFILES__',pictures).replace('__FLOOR__',floor_picture).replace('__INPUT__',html.escape(str(snapshot.source)))
    (directory/'report.html').write_text(page)
