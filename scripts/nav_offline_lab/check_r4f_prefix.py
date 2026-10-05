#!/usr/bin/python3
"""NAV-OFFLINE-R4-F — prueba de causalidad: el goal checker solo decide CUÁNDO terminar.

Para cada corrida (scenario + path_id, misma semilla) compara el log de control de la variante R4-F contra la baseline
R4-E 0.15 (yaw 0.25) desde t=0 hasta el último ciclo de la baseline (o de la variante, si termina antes).
Columnas: t x y th ex ey eth v w cv cw ok (sin ms, que es tiempo de pared).
Esperado:
  - variante con tolerancia más estricta: prefijo idéntico hasta el último ciclo de la baseline salvo, a lo sumo,
    ese último ciclo (allí la baseline dispara SUCCEEDED y manda cero);
  - si la baseline terminó por otra causa (ABORT_*, TIMEOUT), la corrida completa debe ser idéntica.
Escribe dwb_r4f/prefix_check.json
"""
import json
import os

import numpy as np

LAB = os.path.expanduser('~/limo_sml_ws/results/navigation_offline_lab')
SCEN = ['S1_nominal', 'S2_ruido_xy', 'S3_ruido_yaw', 'S4_latencia_0p6', 'S5_combinado_medido', 'S6_combinado_lat0p6']


def load(fn):
    return {x['run']: x for x in (json.loads(z) for z in list(open(fn))[1:])}


rep = {}
for tag in ('020', '015', '010'):
    n = ident_prefix = ident_full_nonsucc = bad = 0
    bad_list = []
    for s in SCEN:
        B = load(os.path.join(LAB, 'dwb_r4e', f'sim_{s}_015.jsonl'))
        V = load(os.path.join(LAB, 'dwb_r4f', f'sim_{s}_{tag}.jsonl'))
        for k, b in B.items():
            v = V[k]
            n += 1
            cb = np.array(b['ctrl'], float)[:, :12]
            cv = np.array(v['ctrl'], float)[:, :12]
            m = min(len(cb), len(cv))
            if b['status'] != 'SUCCEEDED':
                same = len(cb) == len(cv) and np.array_equal(cb, cv) and v['status'] == b['status']
                ident_full_nonsucc += same
                if not same:
                    bad += 1
                    bad_list.append(f'{s}:{k}:{b["status"]}->{v["status"]}')
                continue
            # baseline SUCCEEDED en su último ciclo: el prefijo previo debe coincidir
            same = np.array_equal(cb[:m - 1], cv[:m - 1]) and len(cv) >= len(cb) - 0
            ident_prefix += same
            if not same:
                bad += 1
                bad_list.append(f'{s}:{k}:len {len(cb)}->{len(cv)}')
    rep[tag] = dict(n=n, baseline_succeeded_prefix_identical=ident_prefix, baseline_not_succeeded_full_identical=ident_full_nonsucc,
                    mismatches=bad, mismatch_list=bad_list[:50], PASS=bad == 0)
    print(tag, rep[tag]['PASS'], {k: v for k, v in rep[tag].items() if k != 'mismatch_list'}, bad_list[:5])
json.dump(rep, open(os.path.join(LAB, 'dwb_r4f', 'prefix_check.json'), 'w'), indent=1)
