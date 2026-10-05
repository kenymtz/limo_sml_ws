#!/usr/bin/python3
"""NAV-OFFLINE-R4-G1 — configs de simulador para replan_1hz de la cadena R4-B..R4-F. NO producción.

Para cada config FIJA ya evaluada (no se modifica) genera <fuente>_replan = fuente + GridBased + global_costmap de R3
(mismo procedimiento que make_dwb_r4g0_config.py; global update/publish_frequency 0 = arnés síncrono):
  B_A    dwb_offline_R4B_base.yaml      BaseObstacle, sin RotateToGoal            (R4-B variante A)
  B_B    dwb_offline_R4B_fp.yaml        ObstacleFootprint, sin RotateToGoal       (R4-B variante B)
  D      dwb_offline_R4D_fp_rtg.yaml    + RotateToGoal, FollowPath xy 0.25, yaw 0.25 (R4-D)
  E015   dwb_offline_R4E_015_sim.yaml   + FollowPath xy 0.15                      (R4-E)
  F010   dwb_offline_R4F_010_sim.yaml   + yaw_goal_tolerance 0.10 = CANDIDATA     (R4-F)
Salidas: config/offline_lab/dwb_offline_R4G1_<TAG>_replan_sim.yaml + dwb_offline_R4G1_manifest.json
"""
import copy
import hashlib
import json
import os

import yaml

WS = os.path.expanduser('~/limo_sml_ws')
LAB = os.path.join(WS, 'config/offline_lab')
SRC = {'B_A': 'dwb_offline_R4B_base.yaml', 'B_B': 'dwb_offline_R4B_fp.yaml', 'D': 'dwb_offline_R4D_fp_rtg.yaml',
       'E015': 'dwb_offline_R4E_015_sim.yaml', 'F010': 'dwb_offline_R4F_010_sim.yaml'}
R3 = os.path.join(LAB, 'planner_offline_R3.yaml')


def sha(b):
    return hashlib.sha256(b).hexdigest()


def main():
    r3 = yaml.safe_load(open(R3))
    gb = r3['planner_server']['ros__parameters']['GridBased']
    gc = copy.deepcopy(r3['global_costmap'])
    gc['global_costmap']['ros__parameters']['update_frequency'] = 0.0
    gc['global_costmap']['ros__parameters']['publish_frequency'] = 0.0
    man = dict(r3=R3, r3_sha256=sha(open(R3, 'rb').read()), variants={})
    for tag, src in SRC.items():
        fn = os.path.join(LAB, src)
        d = yaml.safe_load(open(fn))
        d['controller_server']['ros__parameters']['GridBased'] = copy.deepcopy(gb)
        d['global_costmap'] = copy.deepcopy(gc)
        hdr = (f'# NAV-OFFLINE-R4-G1 — GENERADA por scripts/nav_offline_lab/make_dwb_r4g1_configs.py. NO ES PRODUCCIÓN.\n'
               f'# = {src} (sha256 {sha(open(fn, "rb").read())[:16]}…) + GridBased y global_costmap de planner_offline_R3.yaml\n'
               f'#   (global update/publish_frequency 0: arnés síncrono). Para plan_mode=replan_1hz.\n')
        txt = hdr + yaml.safe_dump(d, sort_keys=False, default_flow_style=None)
        out = os.path.join(LAB, f'dwb_offline_R4G1_{tag}_replan_sim.yaml')
        open(out, 'w').write(txt)
        man['variants'][tag] = dict(src=src, src_sha256=sha(open(fn, 'rb').read()), out=os.path.basename(out), out_sha256=sha(txt.encode()))
    json.dump(man, open(os.path.join(LAB, 'dwb_offline_R4G1_manifest.json'), 'w'), indent=1)
    print(json.dumps(man, indent=1))


if __name__ == '__main__':
    main()
