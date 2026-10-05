#!/usr/bin/python3
"""NAV-OFFLINE-R4-G0 — config del simulador para replan_1hz / replan_same_1hz. NO producción.

dwb_offline_R4G0_replan_sim.yaml = dwb_offline_R4F_010_sim.yaml (candidata R4-F, NO se modifica)
  + controller_server.GridBased.{plugin, tolerance, use_astar, allow_unknown}  (= planner_server.GridBased de R3;
    DWB no lee estas claves, las usa el NavFn embebido en el simulador)
  + global_costmap  (= planner_offline_R3.yaml global_costmap; ÚNICOS cambios de arnés: update_frequency 0 y
    publish_frequency 0, porque el simulador actualiza el costmap de forma síncrona, igual que el local)
fixed_plan se sigue corriendo con dwb_offline_R4F_010_sim.yaml sin tocar.
Verificación: verify_r4g0_configs.py.
"""
import copy
import hashlib
import json
import os

import yaml

WS = os.path.expanduser('~/limo_sml_ws')
LAB = os.path.join(WS, 'config/offline_lab')
SRC_SIM = os.path.join(LAB, 'dwb_offline_R4F_010_sim.yaml')
SRC_R3 = os.path.join(LAB, 'planner_offline_R3.yaml')
OUT = os.path.join(LAB, 'dwb_offline_R4G0_replan_sim.yaml')


def sha(b):
    return hashlib.sha256(b).hexdigest()


def main():
    sim = yaml.safe_load(open(SRC_SIM))
    r3 = yaml.safe_load(open(SRC_R3))
    d = copy.deepcopy(sim)
    gb = r3['planner_server']['ros__parameters']['GridBased']
    assert gb['plugin'] == 'nav2_navfn_planner/NavfnPlanner'
    d['controller_server']['ros__parameters']['GridBased'] = copy.deepcopy(gb)
    gc = copy.deepcopy(r3['global_costmap'])
    gc['global_costmap']['ros__parameters']['update_frequency'] = 0.0
    gc['global_costmap']['ros__parameters']['publish_frequency'] = 0.0
    d['global_costmap'] = gc
    hdr = ('# NAV-OFFLINE-R4-G0 — GENERADA por scripts/nav_offline_lab/make_dwb_r4g0_config.py. NO ES PRODUCCIÓN.\n'
           f'# = dwb_offline_R4F_010_sim.yaml (sha256 {sha(open(SRC_SIM, "rb").read())[:16]}…) + GridBased y global_costmap de\n'
           f'#   planner_offline_R3.yaml (sha256 {sha(open(SRC_R3, "rb").read())[:16]}…); global update/publish_frequency 0 (arnés).\n')
    txt = hdr + yaml.safe_dump(d, sort_keys=False, default_flow_style=None)
    open(OUT, 'w').write(txt)
    man = dict(src_sim=SRC_SIM, src_sim_sha256=sha(open(SRC_SIM, 'rb').read()), src_r3=SRC_R3,
               src_r3_sha256=sha(open(SRC_R3, 'rb').read()), out=OUT, out_sha256=sha(txt.encode()))
    json.dump(man, open(os.path.join(LAB, 'dwb_offline_R4G0_manifest.json'), 'w'), indent=1)
    print(json.dumps(man, indent=1))


if __name__ == '__main__':
    main()
