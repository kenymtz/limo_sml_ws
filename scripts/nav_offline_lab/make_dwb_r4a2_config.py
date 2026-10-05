#!/usr/bin/python3
"""NAV-OFFLINE-R4-A2 — configs BASELINE vs EXPERIMENTAL (footprint completo) derivadas de dwb_offline_R4A.yaml.

  dwb_offline_R4A2_base.yaml  = R4A (DWB idéntico al real: BaseObstacle)            + local_costmap.update_frequency 20
  dwb_offline_R4A2_fp.yaml    = base con critic BaseObstacle -> ObstacleFootprint (misma posición, mismo scale 0.1)
update_frequency 20 Hz es solo para acelerar el banco offline (el costmap contiene el mapa estático; no cambia su
contenido). Nada de esto toca config/navigation/navigation2_real.yaml ni se usa en el robot.
"""
import copy
import hashlib
import json
import os

import yaml

WS = os.path.expanduser('~/limo_sml_ws')
LAB = os.path.join(WS, 'config/offline_lab')
SRC = os.path.join(LAB, 'dwb_offline_R4A.yaml')
REAL = os.path.join(WS, 'config/navigation/navigation2_real.yaml')

HDR = """# NAV-OFFLINE-R4-A2 — CONFIG EXPERIMENTAL OFFLINE (GENERADA por scripts/nav_offline_lab/make_dwb_r4a2_config.py).
# Fuente: config/offline_lab/dwb_offline_R4A.yaml sha256={sha} (derivada de navigation2_real.yaml, NO modificado).
# {note}
# Solo para ROS_DOMAIN_ID=99 / ROS_LOCALHOST_ONLY=1. NO usar en el robot.
"""


def main():
    raw = open(SRC, 'rb').read()
    sha = hashlib.sha256(raw).hexdigest()
    real_sha = hashlib.sha256(open(REAL, 'rb').read()).hexdigest()
    src = yaml.safe_load(raw)
    base = copy.deepcopy(src)
    base['local_costmap']['local_costmap']['ros__parameters']['update_frequency'] = 20.0
    fp = copy.deepcopy(base)
    fpp = fp['controller_server']['ros__parameters']['FollowPath']
    crit = fpp['critics']
    assert crit.count('BaseObstacle') == 1
    crit[crit.index('BaseObstacle')] = 'ObstacleFootprint'
    fpp['ObstacleFootprint.scale'] = fpp.pop('BaseObstacle.scale')
    outs = {
        'dwb_offline_R4A2_base.yaml': (base, 'BASELINE: DWB idéntico al real (BaseObstacle). Cambio vs R4A: local_costmap.update_frequency 5 -> 20 (velocidad del banco).'),
        'dwb_offline_R4A2_fp.yaml': (fp, 'EXPERIMENTAL: critics BaseObstacle -> ObstacleFootprint (footprint completo con padding), ObstacleFootprint.scale 0.1 (= BaseObstacle.scale). Resto = BASELINE.'),
    }
    man = {'source': SRC, 'source_sha256': sha, 'real_sha256': real_sha, 'outputs': {}}
    for fn, (doc, note) in outs.items():
        txt = HDR.format(sha=sha, note=note) + yaml.safe_dump(doc, sort_keys=False, default_flow_style=None)
        open(os.path.join(LAB, fn), 'w').write(txt)
        man['outputs'][fn] = {'sha256': hashlib.sha256(txt.encode()).hexdigest(), 'note': note}
    # verificación: la única diferencia DWB entre base y fp es el critic de obstáculos
    b = base['controller_server']['ros__parameters']['FollowPath']
    e = fp['controller_server']['ros__parameters']['FollowPath']
    diff = sorted(k for k in set(b) | set(e) if b.get(k) != e.get(k))
    man['dwb_param_diff_base_vs_fp'] = diff
    assert set(diff) == {'critics', 'BaseObstacle.scale', 'ObstacleFootprint.scale'}, diff
    json.dump(man, open(os.path.join(LAB, 'dwb_offline_R4A2_manifest.json'), 'w'), indent=1)
    print(json.dumps(man, indent=1))


if __name__ == '__main__':
    main()
