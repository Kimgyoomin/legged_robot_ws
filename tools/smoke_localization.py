#!/usr/bin/env python3
"""Startup/link/import checks only; no claim of real sensor localization accuracy."""
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import rclpy

ROOT=Path(__file__).resolve().parents[1]


def main():
    if os.environ.get('ROS_DOMAIN_ID')!='177':raise SystemExit('Use isolated ROS_DOMAIN_ID=177')
    results={}
    for method in ('fastlio','genz'):
        logfile=ROOT/'validation'/f'{method}_startup.log'
        with logfile.open('w') as log:
            p=subprocess.Popen(['ros2','launch','legged_bringup','localize.launch.py',
                'method:='+method,'site:='+str(ROOT/'maps/demo/site.yaml')],
                stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            rclpy.init();node=rclpy.create_node('startup_check')
            try:
                end=time.monotonic()+6
                while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.2)
                names={name for name,_ in node.get_node_names_and_namespaces()}
                required={'laser_mapping','global_localization','transform_fusion'} if method=='fastlio' else {'genz_odometry','genz_localization'}
                assert required <= names,(method,names,logfile.read_text())
                text=logfile.read_text()
                assert 'process has died' not in text and 'Traceback' not in text,text
                results[method]={'startup':True,'nodes':sorted(required),'accuracy_tested':False}
            finally:
                os.killpg(p.pid,signal.SIGINT)
                try:p.wait(timeout=8)
                except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=5)
                node.destroy_node();rclpy.shutdown()
    (ROOT/'validation/localization_startup.json').write_text(json.dumps(results,indent=2)+'\n')
    print(json.dumps(results))


if __name__=='__main__':main()
