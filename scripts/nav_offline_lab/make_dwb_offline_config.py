#!/usr/bin/python3
"""NAV-OFFLINE-R4-A — genera la config DWB offline a partir de config/navigation/navigation2_real.yaml.

La config real NO se modifica (solo lectura). Se copian literalmente los bloques controller_server y
local_costmap y se aplican SOLO los cambios listados en OVERRIDES (documentados en el yaml de salida).

Salidas (config/offline_lab/):
  dwb_offline_R4A.yaml          local_costmap: static_layer(/map) + inflation (proxy de percepción perfecta)
  dwb_offline_R4A_realcm.yaml   local_costmap IDÉNTICO al real (obstacle_layer sin /scan -> costmap vacío)
Ambos: controller_server byte-a-byte igual al real salvo nada (0 cambios en DWB).
"""
import copy
import hashlib
import json
import os
import sys

import yaml

WS = os.path.expanduser('~/limo_sml_ws')
REAL = os.path.join(WS, 'config/navigation/navigation2_real.yaml')
OUT = os.path.join(WS, 'config/offline_lab')

HEADER = """# NAV-OFFLINE-R4-A — CONFIG EXPERIMENTAL OFFLINE (GENERADA, no editar a mano).
# Generador: scripts/nav_offline_lab/make_dwb_offline_config.py
# Fuente:    config/navigation/navigation2_real.yaml  sha256={sha}  (NO modificado)
# controller_server: copia literal del real (0 cambios en DWB / critics / límites).
# local_costmap: {cm_note}
# Solo para ROS_DOMAIN_ID=99 / ROS_LOCALHOST_ONLY=1. NO usar en el robot.
"""


def main():
    raw = open(REAL, 'rb').read()
    sha = hashlib.sha256(raw).hexdigest()
    real = yaml.safe_load(raw)
    cs = copy.deepcopy(real['controller_server'])
    lc = copy.deepcopy(real['local_costmap'])
    ms = {'ros__parameters': {'use_sim_time': False, 'yaml_filename': '', 'topic_name': 'map',
                              'frame_id': 'map'}}

    # Variante R4A: local costmap con el mapa estático en lugar de LiDAR (no hay sensores offline).
    lc_static = copy.deepcopy(lc)
    p = lc_static['local_costmap']['ros__parameters']
    p['plugins'] = ['static_layer', 'inflation_layer']
    p['static_layer'] = {'plugin': 'nav2_costmap_2d::StaticLayer', 'map_subscribe_transient_local': True,
                         'enabled': True, 'subscribe_to_updates': False, 'transform_tolerance': 0.2}
    p.pop('obstacle_layer', None)
    p.pop('voxel_layer', None)
    variants = {
        'dwb_offline_R4A.yaml': (lc_static,
                                 'plugins [obstacle_layer, inflation_layer] -> [static_layer, inflation_layer]; '
                                 'obstacle/voxel eliminados (sin /scan offline). Resto igual al real.'),
        'dwb_offline_R4A_realcm.yaml': (lc, 'IDÉNTICO al real (obstacle_layer sin /scan => costmap sin obstáculos).'),
    }
    manifest = {'source': REAL, 'source_sha256': sha, 'outputs': {}}
    for fn, (lcv, note) in variants.items():
        doc = {'map_server': ms, 'controller_server': cs, 'local_costmap': lcv}
        txt = HEADER.format(sha=sha, cm_note=note) + yaml.safe_dump(doc, sort_keys=False, default_flow_style=None)
        path = os.path.join(OUT, fn)
        with open(path, 'w') as f:
            f.write(txt)
        # verificación: controller_server idéntico al real
        back = yaml.safe_load(open(path))
        assert back['controller_server'] == real['controller_server'], 'controller_server difiere del real'
        manifest['outputs'][fn] = {'sha256': hashlib.sha256(txt.encode()).hexdigest(), 'local_costmap_note': note,
                                   'controller_server_identical_to_real': True}
    assert hashlib.sha256(open(REAL, 'rb').read()).hexdigest() == sha
    json.dump(manifest, open(os.path.join(OUT, 'dwb_offline_R4A_manifest.json'), 'w'), indent=1)
    print(json.dumps(manifest, indent=1))
    return 0


if __name__ == '__main__':
    sys.exit(main())
