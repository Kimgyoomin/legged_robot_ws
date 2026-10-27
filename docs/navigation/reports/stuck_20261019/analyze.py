"""Reproduce the read-only diagnostic summary and log timeline."""
from pathlib import Path
from collections import Counter
import datetime
import json
import re

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

out = Path(__file__).resolve().parent
launch = Path('/home/kim/.ros/log/2026-10-19-18-01-13-060710-Kim9975-270149/launch.log')
controller = Path('/home/kim/.ros/log/pure_pursuit_270179_1792400473260.log')
start, align_end = 1792403235.144651004, 1792403241.344657440
end = 1792403308.387589955
events = []
for line in launch.read_text().splitlines():
    m = re.search(r'\[INFO\] \[([\d.]+)\] \[command_watchdog\]: Motion status: (\w+)', line)
    if m and start <= float(m[1]) <= end:
        events.append((float(m[1]), m[2]))
stops = []
for (t, state), (next_t, _) in zip(events, events[1:]):
    if state == 'predicted_costmap_collision':
        stops.append({'start_relative_wall_s': t-start, 'duration_wall_s': next_t-t})
commands = []
for line in controller.read_text().splitlines():
    m = re.search(r'\[([\d.]+)\].*PP tracking:.*cte=([\d.-]+).*cmd=\(([\d.-]+),([\d.-]+)\)', line)
    if m and start <= float(m[1]) <= end:
        commands.append([float(m[1])-start, float(m[3]), float(m[4]), float(m[2])])

fig, axes = plt.subplots(3, 1, figsize=(11, 6.5), sharex=True, gridspec_kw={'height_ratios':[1,1,.65]})
for ax, column, ylabel, color in zip(axes[:2], [1,2], ['Requested v [m/s]', 'Requested yaw [rad/s]'], ['#2563eb','#5b21b6']):
    ax.plot([v[0] for v in commands], [v[column] for v in commands], '.-', color=color, lw=1.3)
    ax.set_ylabel(ylabel); ax.grid(alpha=.2)
    ax.text(.985,.06,'Blank tail: command log unavailable', transform=ax.transAxes,
            ha='right', fontsize=8, color='#64748b')
    ax.axvspan(0, align_end-start, color='#fbbf24', alpha=.22, label='Heading alignment')
    for i, s in enumerate(stops):
        ax.axvspan(s['start_relative_wall_s'], s['start_relative_wall_s']+s['duration_wall_s'], color='#ef4444', alpha=.45, label='Guard stop' if i==0 else None)
axes[0].legend(loc='upper right', fontsize=9)
axes[2].broken_barh([(0,align_end-start)], (1.1,.6), facecolors='#fbbf24')
axes[2].broken_barh([(s['start_relative_wall_s'],s['duration_wall_s']) for s in stops], (.1,.6), facecolors='#ef4444')
axes[2].set_yticks([.4,1.4], ['Guard stop', 'Alignment'])
axes[2].set_ylim(-.3,2.1); axes[2].set_xlim(0,end-start)
axes[2].set_xlabel('Wall-clock seconds since 18:47:15 KST; goal_reached at 73.24 s')
fig.suptitle('Last completed logged run: alignment, brief safety stops, then goal reached', fontsize=13)
fig.text(.5,.01,'Command samples are throttled controller logs, not measured robot velocity. Red intervals use watchdog state transitions.',ha='center',fontsize=9)
fig.tight_layout(rect=(0,.04,1,.96)); fig.savefig(out/'logged_run.png',dpi=150); plt.close(fig)

def summarize_capture(directory):
    trace_path=directory/'trace.json'
    if not trace_path.exists(): return None
    data=json.loads(trace_path.read_text())
    result={'topic_counts':dict(Counter(d['topic'] for d in data)),
        'tracking_state_counts':dict(Counter(d['state'] for d in data if d['topic']=='/legged/tracking_state')),
        'motion_reason_message_counts':dict(Counter(json.loads(d['state'])['reason'] for d in data if d['topic']=='/legged/motion_status'))}
    clocks=[d['clock'] for d in data if d['topic']=='/clock']
    if clocks: result['sim_time_range']=[min(clocks),max(clocks)]
    for topic in ['/legged/cmd_raw','/legged/cmd_safe','/cmd_vel']:
        values=[d for d in data if d['topic']==topic]
        result[topic]={'count':len(values),'nonzero':sum(abs(d['vx'])+abs(d['vy'])+abs(d['wz'])>1.e-6 for d in values)}
    gates_path=directory/'gates.json'
    if gates_path.exists():
        gates=json.loads(gates_path.read_text())
        result['gate_samples']=len(gates)
        result['stationary_guard_counts']=dict(Counter(d.get('stationary_guard','unavailable') for d in gates))
    return result

summary={'sources':{'controller_log':str(controller),'launch_log':str(launch)},
    'logged_run':{'start_local':datetime.datetime.fromtimestamp(start).astimezone().isoformat(),
        'alignment_wall_s':align_end-start,'goal_reached_after_wall_s':end-start,
        'predicted_stop_intervals':stops,'total_predicted_stop_wall_s':sum(v['duration_wall_s'] for v in stops),
        'limitation':'Logged controller requests and gate states; not continuous measured robot velocity.'},
    'initial_capture':summarize_capture(out),'followup_capture':summarize_capture(out/'live_resume')}
(out/'summary.json').write_text(json.dumps(summary,indent=2,ensure_ascii=False)+'\n')
print(json.dumps(summary,indent=2,ensure_ascii=False))
