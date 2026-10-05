#!/usr/bin/python3
"""NAV-OFFLINE-R4-G — variantes del progress checker sobre la candidata R4-F (yaw 0.10). NO producción.

Desde navigation2_R4F_010.yaml y dwb_offline_R4F_010_sim.yaml (NO se modifican) genera, cambiando UN parámetro:
  EXPERIMENTO A  PR005 / PR003 : progress_checker.required_movement_radius 0.10 -> 0.05 / 0.03
  EXPERIMENTO B  PT35  / PT45  : progress_checker.movement_time_allowance  25.0 -> 35.0 / 45.0
Salidas: config/offline_lab/navigation2_R4G_<TAG>.yaml, dwb_offline_R4G_<TAG>_sim.yaml, dwb_offline_R4G_manifest.json.
Verificación: verify_r4g_configs.py.
"""
import copy
import hashlib
import json
import os

import yaml

WS = os.path.expanduser('~/limo_sml_ws')
LAB = os.path.join(WS, 'config/offline_lab')
SRC_FULL = os.path.join(LAB, 'navigation2_R4F_010.yaml')
SRC_SIM = os.path.join(LAB, 'dwb_offline_R4F_010_sim.yaml')
VARIANTS = {'PR005': ('required_movement_radius', 0.05), 'PR003': ('required_movement_radius', 0.03),
            'PT35': ('movement_time_allowance', 35.0), 'PT45': ('movement_time_allowance', 45.0)}
BASE = {'required_movement_radius': 0.1, 'movement_time_allowance': 25.0}


def sha(b):
    return hashlib.sha256(b).hexdigest()


def main():
    man = dict(src_full=SRC_FULL, src_full_sha256=sha(open(SRC_FULL, 'rb').read()),
               src_sim=SRC_SIM, src_sim_sha256=sha(open(SRC_SIM, 'rb').read()), variants={})
    docs0 = {'full': yaml.safe_load(open(SRC_FULL)), 'sim': yaml.safe_load(open(SRC_SIM))}
    for d in docs0.values():
        pc = d['controller_server']['ros__parameters']['progress_checker']
        assert pc['plugin'] == 'nav2_controller::SimpleProgressChecker' and all(pc[k] == v for k, v in BASE.items()), pc
    for tag, (key, val) in VARIANTS.items():
        out = {}
        for kind, src, name in (('full', SRC_FULL, f'navigation2_R4G_{tag}.yaml'), ('sim', SRC_SIM, f'dwb_offline_R4G_{tag}_sim.yaml')):
            d = copy.deepcopy(docs0[kind])
            d['controller_server']['ros__parameters']['progress_checker'][key] = val
            hdr = (f'# NAV-OFFLINE-R4-G — GENERADA por scripts/nav_offline_lab/make_dwb_r4g_config.py. NO ES PRODUCCIÓN.\n'
                   f'# = {os.path.basename(src)} (sha256 {man["src_" + kind + "_sha256"][:16]}…) con ÚNICO cambio\n'
                   f'#   controller_server.progress_checker.{key}: {BASE[key]} -> {val}\n')
            txt = hdr + yaml.safe_dump(d, sort_keys=False, default_flow_style=None)
            open(os.path.join(LAB, name), 'w').write(txt)
            out[kind] = dict(file=name, sha256=sha(txt.encode()))
        man['variants'][tag] = dict(param=f'progress_checker.{key}', value=val, **out)
    json.dump(man, open(os.path.join(LAB, 'dwb_offline_R4G_manifest.json'), 'w'), indent=1)
    print(json.dumps(man, indent=1))


if __name__ == '__main__':
    main()
