#!/usr/bin/python3
"""NAV-OFFLINE-R4-C — genera results/navigation_offline_lab/DWB_R4C_FAILURE_ANALYSIS.md desde dwb_r4c/r4c_summary.json."""
import csv
import json
import math
import os

LAB = os.path.expanduser('~/limo_sml_ws/results/navigation_offline_lab')
D = os.path.join(LAB, 'dwb_r4c')
SCEN = ['S1_nominal', 'S2_ruido_xy', 'S3_ruido_yaw', 'S4_latencia_0p6', 'S5_combinado_medido', 'S6_combinado_lat0p6']
CLS = ['OK', 'GOAL_TOLERANCE', 'OSCILACION', 'TIMEOUT', 'TRACKING', 'OTRO_SIN_TRAYECT', 'OTRO_EXITO_CONTACTO',
       'OTRO_EXITO_APARENTE', 'OTRO']
DESC = {
    'OK': 'SUCCEEDED correcto: sin contacto; error VERDADERO final ≤ 0.25 m y ≤ 0.25 rad',
    'GOAL_TOLERANCE': 'goal tolerance failure: llegó en xy (etapa 1 del goal checker, stateful) y no cumplió el yaw',
    'OSCILACION': 'oscilación: no llegó en xy; ABORT_PROGRESS con el robot quieto (< 0.1 m en 25 s) y ≥ 4 inversiones de ω',
    'TIMEOUT': 'timeout: no llegó en xy; se agotó t_max = 3·L/0.25 + 30 s',
    'TRACKING': 'tracking failure: no llegó en xy y el error lateral máximo superó 0.25 m',
    'OTRO_SIN_TRAYECT': 'otro: DWB sin trayectorias legales más de 0.3 s (ABORT_CONTROLLER); el robot se detiene',
    'OTRO_EXITO_CONTACTO': 'otro: SUCCEEDED, pero con contacto real del footprint durante la corrida',
    'OTRO_EXITO_APARENTE': 'otro: SUCCEEDED sobre la pose ESTIMADA, con error VERDADERO final fuera de tolerancia',
    'OTRO': 'resto',
}


def f(x, n=3):
    return '—' if x is None or x == '' else (f'{float(x):.{n}f}')


def main():
    S = json.load(open(os.path.join(D, 'r4c_summary.json')))
    fails = list(csv.DictReader(open(os.path.join(D, 'r4c_failures.csv'))))
    n, nf = S['n_runs'], S['n_fail']
    bc = S['by_cls']
    att = S['attribution']
    k_rtg = [k for k in att if k.startswith('PARAMETRO (inferido)')]
    k_tol = [k for k in att if k.startswith('PARAMETRO: tolerancias')]
    k_obs = [k for k in att if k.startswith('PARAMETRO: critic')]
    k_dwb = [k for k in att if k.startswith('DWB')]
    k_din = [k for k in att if k.startswith('DINAMICA')]
    k_loc = [k for k in att if k.startswith('LOCALIZACION')]
    cnt = lambda ks: sum(att[k] for k in ks)  # noqa: E731
    gt = S['goal_tol_detail']
    cf = S['counterfactual']
    L = []
    A = L.append
    A('# NAV-OFFLINE-R4-C — Análisis de terminaciones incorrectas (datos de R4-B, sin simulaciones nuevas)\n')
    A('Fecha: 2026-09-29. Análisis 100 % offline sobre las 768 corridas ya generadas en R4-B (`results/navigation_offline_lab/dwb_r4b/sim_*.jsonl`): '
      '6 escenarios × 64 paths × 2 variantes (A = BaseObstacle, config real; B = ObstacleFootprint). **No se ejecutó ninguna simulación nueva, '
      'no se modificó ninguna configuración**, ni `navigation2_real.yaml` ni `~/limoatwork`.\n')
    A('## Resumen\n')
    A('```')
    A('R4-C')
    A(f'Corridas analizadas: {n}')
    A(f'Fallos:              {nf}  (A {sum(v for k, v in S["by_cls_variant"]["A"].items() if k != "OK")}, '
      f'B {sum(v for k, v in S["by_cls_variant"]["B"].items() if k != "OK")})')
    A(f'Causa principal:     llegada al goal sin orientación final. {bc.get("GOAL_TOLERANCE", 0)} corridas llegan en xy '
      f'(error mediano {gt["final_xy_med"] * 1000:.0f} mm) y no cumplen el yaw')
    A(f'                     (error mediano {gt["final_yaw_med"]:.2f} rad); con los {att.get(k_din[0], 0) if k_din else 0} sobrepasos '
      f'de yaw al frenar suman {bc.get("GOAL_TOLERANCE", 0) + (att.get(k_din[0], 0) if k_din else 0)}/{nf} fallos.')
    A('```\n')

    A('## Método\n')
    A('Una clase por corrida, asignada por reglas fijas en este orden (`scripts/nav_offline_lab/analyze_r4c_failures.py`):\n')
    A('| clase | definición |')
    A('|---|---|')
    for c in CLS:
        A(f'| {c} | {DESC[c]} |')
    A('\n- Errores finales = pose **verdadera** en reposo vs goal. Error lateral = distancia de la pose verdadera al path NavFn hasta la llegada xy '
      '(o hasta el fin). Clearance = footprint real sin padding vs geometría del mapa (mismas métricas que R4-B).')
    A('- **Contrafactual de tolerancias (exacto)**: el goal checker stateful se re-evalúa sobre la pose **estimada** registrada con tolerancias mayores. '
      'Parar antes no altera la trayectoria previa, así que el instante de disparo es exacto. Cuenta como resuelta solo si dispara antes del fin, '
      'sin contacto previo, y con la pose **verdadera** dentro de las nuevas tolerancias en ese instante.')
    A('- **Atribución** (reglas explícitas):\n')
    A('| atribución | regla | evidencia |')
    A('|---|---|---|')
    A('| PARÁMETRO: tolerancias | GOAL_TOLERANCE resuelta por el contrafactual con xy ≤ 0.40 m y yaw ≤ 0.785 rad | verificada (contrafactual exacto) |')
    A('| PARÁMETRO: critic de obstáculos | la corrida pareada (mismo path y escenario, misma semilla) con el otro critic termina OK | verificada (pareado) |')
    A('| PARÁMETRO (inferido): sin orientación final | GOAL_TOLERANCE que no se resuelve con xy ≤ 0.40 / yaw ≤ 0.785 (solo con yaw ≥ 1.57 o sin chequeo) | inferida: `RotateToGoal` está comentado en la config real y ningún critic activo orienta al yaw del goal; **no simulado** |')
    A('| DWB | falla en ambas variantes y no la resuelven las tolerancias | por exclusión; no atribuible a un parámetro con estos datos |')
    A('| DINÁMICA DE PARADA | SUCCEEDED con la pose verdadera dentro de tolerancia al detectar, fuera al detenerse | verificada (pose en el disparo vs pose final) |')
    A('| LOCALIZACIÓN | SUCCEEDED con la pose verdadera ya fuera de tolerancia al detectar | verificada |')
    A('')

    A('## CONFIRMADO\n')
    A(f'- **GOAL_TOLERANCE ({bc.get("GOAL_TOLERANCE", 0)})**: el robot está en la posición del goal (error xy verdadero mediano {gt["final_xy_med"] * 1000:.1f} mm), '
      f'con el rumbo lejos del yaw del goal (mediana {gt["final_yaw_med"]:.2f} rad, p90 {gt["final_yaw_p90"]:.2f} rad). Tras llegar en xy sigue '
      f'{gt["t_after_xy_med"]:.1f} s (mediana) invirtiendo el giro (≈14 inversiones de ω en los últimos 25 s, |ω| ≈ 0.47 rad/s) hasta que el progress checker '
      'aborta. Con estos critics no hay término que lleve el rumbo al yaw del goal: GoalAlign/PathAlign alinean con el path y `RotateToGoal` está '
      'comentado en `navigation2_real.yaml` (verificado leyendo la config).')
    A('- **El ruido XY enmascara esta falla**: GOAL_TOLERANCE es 77 en S1, 61 en S3 (yaw), 56 en S4 (latencia) y cae a 3, 3 y 0 en S2, S5 y S6 (los escenarios con '
      'ruido XY). No es una mejora: el ruido mueve el goal estimado, agita el rumbo y el yaw cruza la ventana de ±0.25 rad por azar.')
    sub = S.get('subcls', {})
    A(f'- **Éxitos aparentes ({bc.get("OTRO_EXITO_APARENTE", 0)})**: {sub.get("SOBREPASO_AL_FRENAR", 0)} estaban dentro de tolerancia (pose verdadera) al detectar '
      'el goal y se salieron al frenar: el robot gira a ~0.77 rad/s (mediana) al detectar, el cero llega con la latencia de la base y la desaceleración '
      'de 1.5 rad/s² agrega el resto (sobrepaso mediano 0.24 rad, máx 0.60). 81/101 son de los escenarios con latencia 0.6 s (S4, S6). '
      f'Los otros {sub.get("ERROR_LOCALIZACION", 0)} ya estaban fuera de tolerancia al detectar (goal checker sobre la pose estimada con ruido).')
    A('- **OSCILACIÓN y TIMEOUT sin llegar** son mínimos locales: el robot se queda a ~1–2 m del goal. En around_obstacle (goal detrás de un obstáculo) '
      'pasa con ambas variantes. En corridor_N1 con BaseObstacle se debe al término de obstáculo sobre la última pose (mecanismo verificado en R4-B).')
    A(f'- **TRACKING failure = {bc.get("TRACKING", 0)}**: ninguna corrida falla por apartarse del path. El error lateral máximo de todas las fallas '
      'sin llegada es ≤ 0.22 m, y en las 768 corridas ≤ 0.37 m.')
    A(f'- **OTRO_SIN_TRAYECT ({bc.get("OTRO_SIN_TRAYECT", 0)})**: todas en la variante B (ObstacleFootprint), mayormente al entrar a corridor_N1 '
      '(t ≈ 3–7 s). El footprint con padding (0.36 m de ancho) en un pasillo de 0.45 m deja casos sin ninguna trayectoria legal. El robot se detiene '
      '(2 de ellas tras un contacto de < 1 cm con latencia 0.6 s).\n')

    A('## MEDIDO\n')
    A('### Clases por variante\n')
    A('| clase | A BaseObstacle | B ObstacleFootprint | total |')
    A('|---|---|---|---|')
    for c in CLS:
        a, b = S['by_cls_variant']['A'].get(c, 0), S['by_cls_variant']['B'].get(c, 0)
        if a or b:
            A(f'| {c} | {a} | {b} | {a + b} |')
    A('')
    A('### Clases por escenario\n')
    A('| escenario | ' + ' | '.join(c for c in CLS if bc.get(c)) + ' |')
    A('|---|' + '---|' * len([c for c in CLS if bc.get(c)]))
    for s in SCEN:
        A(f'| {s} | ' + ' | '.join(str(S['by_cls_scenario'][s].get(c, 0)) for c in CLS if bc.get(c)) + ' |')
    A('')
    A('### Clases por categoría de path\n')
    A('| categoría | ' + ' | '.join(c for c in CLS if bc.get(c)) + ' |')
    A('|---|' + '---|' * len([c for c in CLS if bc.get(c)]))
    for cat, d in sorted(S['by_cls_category'].items()):
        A(f'| {cat} | ' + ' | '.join(str(d.get(c, 0)) for c in CLS if bc.get(c)) + ' |')
    A('')
    A('### Estadísticas por clase\n')
    A('| clase | n | error lateral máx (m) | error final xy med (m) | error final yaw med (rad) | t fin med (s) | clearance mín (m) |')
    A('|---|---|---|---|---|---|---|')
    for c in CLS:
        st = S['cls_stats'].get(c)
        if st:
            A(f"| {c} | {st['n']} | {f(st['lat_max_max'])} | {f(st['final_xy_med'])} | {f(st['final_yaw_med'])} | {f(st['t_end_med'], 1)} | {f(st['clear_min'])} |")
    A('')

    A('## Respuestas\n')
    A('### 1. ¿Cuál es la causa dominante de fallos?\n')
    A(f'La llegada al goal sin orientación final. **{bc.get("GOAL_TOLERANCE", 0)}/{nf} fallos ({100 * bc.get("GOAL_TOLERANCE", 0) / nf:.0f} %)** son GOAL_TOLERANCE: '
      'el robot llega en posición y no logra el yaw. Si se suman los '
      f'{att.get(k_din[0], 0) if k_din else 0} sobrepasos de yaw al frenar, el manejo del rumbo al final explica '
      f'**{bc.get("GOAL_TOLERANCE", 0) + (att.get(k_din[0], 0) if k_din else 0)}/{nf} ({100 * (bc.get("GOAL_TOLERANCE", 0) + (att.get(k_din[0], 0) if k_din else 0)) / nf:.0f} %)**. '
      f'La segunda causa son los mínimos locales sin llegar (OSCILACIÓN {bc.get("OSCILACION", 0)} + TIMEOUT {bc.get("TIMEOUT", 0)}).\n')
    A('### 2. ¿Cuántos fallos son por DWB?\n')
    A(f'**{cnt(k_dwb)}** fallan con ambas variantes y no los resuelven las tolerancias. Por clase: '
      + ', '.join(f'{c} {v}' for c, d in S['attribution_by_cls'].items() for k, v in d.items() if k.startswith('DWB')) + '. '
      'Son sobre todo los mínimos locales de around_obstacle (T023–T029) y las paradas sin trayectorias de B en N1. Es una atribución **por exclusión**: '
      'puede haber pesos de critics no probados que los resuelvan.\n')
    A('### 3. ¿Cuántos son por parámetros?\n')
    A(f'**{cnt(k_tol) + cnt(k_obs)} verificados** ({cnt(k_tol)} por tolerancias de goal, contrafactual; {cnt(k_obs)} por el critic de obstáculos, pareado) '
      f'+ **{cnt(k_rtg)} inferidos** (GOAL_TOLERANCE que no se resuelve con yaw ≤ 0.785 rad: falta un critic de orientación final, RotateToGoal comentado). '
      f'Total atribuible a configuración: **{cnt(k_tol) + cnt(k_obs) + cnt(k_rtg)}/{nf}**. '
      f'Aparte: {cnt(k_din)} por dinámica de parada (latencia + ω alta al detectar) y {cnt(k_loc)} por error de localización.\n')
    A('### 4. ¿Cuántos desaparecerían aumentando tolerancias?\n')
    A('Contrafactual exacto sobre los fallos (se cuentan solo si la pose verdadera cumple la nueva tolerancia al disparar y no hubo contacto antes):\n')
    A('| xy tol (m) | yaw tol (rad) | GOAL_TOLERANCE resueltas | éxitos aparentes que pasan a correctos | otras clases |')
    A('|---|---|---|---|---|')
    for xt in (0.25, 0.30, 0.40, 0.50):
        for yt in (0.25, 0.35, 0.50, 0.785, 1.571, math.pi + 0.01):
            kk = f'{xt}|{yt:.3f}'
            d = cf.get(kk)
            if not d or (xt, yt) == (0.25, 0.25):
                continue
            if xt not in (0.25, 0.40, 0.50) and yt not in (0.785,):
                continue
            oth = sum(v for c, v in d['by_cls'].items() if c not in ('GOAL_TOLERANCE', 'OTRO_EXITO_APARENTE'))
            ylab = 'sin chequeo' if yt > 3 else f'{yt:.3f}'
            A(f"| {xt:.2f} | {ylab} | {d['by_cls'].get('GOAL_TOLERANCE', 0)}/{bc.get('GOAL_TOLERANCE', 0)} | "
              f"{d['by_cls'].get('OTRO_EXITO_APARENTE', 0)}/{bc.get('OTRO_EXITO_APARENTE', 0)} | {oth} |")
    A('\n- Subir yaw a 0.35 / 0.50 / 0.785 rad resuelve pocas GOAL_TOLERANCE, porque el error de yaw al llegar es grande (mediana 1.84 rad). '
      'Solo desactivar el chequeo de yaw las elimina casi todas, y eso significa aceptar llegar con cualquier orientación.')
    A('- Subir la tolerancia xy (hasta 0.50 m) **no resuelve ninguna** OSCILACIÓN, TIMEOUT ni parada sin trayectorias: esos robots se quedan a 1–2 m.')
    A('- Los "éxitos aparentes" que pasan a correctos lo hacen porque se redefine qué se considera correcto, no porque el robot quede mejor.\n')
    A('### 5. Los 10 casos más difíciles\n')
    A('Ranking por path sobre sus 12 corridas (6 escenarios × 2 variantes): fallos sin contar éxitos aparentes ↓, contactos ↓, mejor error final ↓, clearance ↑.\n')
    A('| # | path | categoría | L (m) | fallos / 12 | contactos | clearance mín (m) | error lateral máx (m) | mejor error final xy (m) | clases |')
    A('|---|---|---|---|---|---|---|---|---|---|')
    for i, h in enumerate(S['hard'], 1):
        cls = ', '.join(f'{k} {v}' for k, v in sorted(h['classes'].items(), key=lambda x: -x[1]))
        A(f"| {i} | {h['path_id']} | {h['category']} | {h['path_len']:.2f} | {h['n_fail_sin_aparente']} | {h['n_contact']} | {f(h['clear_min'])} | "
          f"{f(h['lat_max'])} | {f(h['min_final_xy'])} | {cls} |")
    A('\n- around_obstacle (goal detrás de un obstáculo): 6 de los 10 paths de la categoría (T023, T024, T025, T027, T028, T029) están en el top 10 y fallan '
      'en 11–12 de 12 corridas con ambas variantes. Mínimo local de DWB: el robot gira en el lugar o avanza sin acercarse.')
    A('- T042, T043, T046 (corridor_N1): concentran los contactos y las paradas sin trayectorias de B.')
    A('- T060 (random, 6 m): llega en xy y falla por yaw en 9/12.\n')
    A('### Detalle por fallo\n')
    A(f'Los {nf} fallos, con path_id, variante, escenario, error lateral máximo, error final xy/yaw, tiempo, clearance mínima, último comando y último '
      'estado, están en `dwb_r4c/r4c_failures.csv`. Muestra (los primeros de cada clase):\n')
    A('| path | variante | escenario | clase | atribución | lat máx | final xy | final yaw | t (s) | clearance | último cmd (v, ω) | último estado |')
    A('|---|---|---|---|---|---|---|---|---|---|---|---|')
    seen = {}
    for r in fails:
        seen.setdefault(r['cls'], 0)
        if seen[r['cls']] >= 3:
            continue
        seen[r['cls']] += 1
        A(f"| {r['path_id']} | {r['variant_name']} | {r['scenario']} | {r['cls']} | {r['attribution'].split(':')[0]} | {f(r['lat_max'])} | "
          f"{f(r['final_xy_err'])} | {f(r['final_yaw_err'])} | {f(r['t_end'], 1)} | {f(r['clear_min'])} | "
          f"({f(r['last_cmd_v'], 2)}, {f(r['last_cmd_w'], 2)}) | {r['last_state']} |")
    A('')

    A('## LIMITACIONES\n')
    A('- Todo proviene de la simulación R4-B (modelo cinemático mínimo, percepción perfecta, sin BT ni recuperaciones, plan fijo, una semilla por caso). '
      'En el robot real, un ABORT dispara recuperaciones y replanificación que aquí no existen.')
    A('- La atribución "DWB" es **por exclusión** y la atribución "sin orientación final" es **inferida**: no se simuló RotateToGoal ni otros pesos de critics. '
      'Confirmarlas requiere simular (R4-D).')
    A('- El contrafactual de tolerancias evalúa la pose verdadera **en el instante del disparo**. No modela el sobrepaso al frenar posterior (que sí existe: '
      '66 éxitos aparentes), así que puede sobreestimar las resoluciones en escenarios con latencia alta.')
    A('- Las fronteras entre clases dependen de umbrales elegidos (quieto < 0.1 m en 25 s, ≥ 4 inversiones de ω, tracking > 0.25 m). Los números por clase '
      'son sensibles a ellos; los estados de fin (ABORT_PROGRESS / TIMEOUT / ABORT_CONTROLLER / SUCCEEDED) no.')
    A('- El contrafactual **no es monótono** en la tolerancia xy (p. ej. yaw 0.25: 5 resueltas con xy 0.40 y 3 con 0.50). El goal checker es stateful: '
      'con más tolerancia xy la etapa 1 dispara antes y la condición de yaw y la pose verdadera se evalúan en otros instantes.')
    A('- La causa de que el ruido XY haga desaparecer GOAL_TOLERANCE está medida pero su mecanismo no se verificó.\n')
    A('## Archivos\n')
    A('- Script: `scripts/nav_offline_lab/analyze_r4c_failures.py`, reporte `scripts/nav_offline_lab/make_dwb_r4c_report.py`')
    A('- Resultados: `results/navigation_offline_lab/dwb_r4c/` (`r4c_all_runs.csv` 768 corridas, `r4c_failures.csv`, `r4c_summary.json`)\n')
    A('## Siguiente ajuste recomendado\n')
    A('**Activar `RotateToGoal` en una copia experimental de la config** (sus parámetros ya están en `navigation2_real.yaml`: scale 32, slowing_factor 5, '
      'lookahead_time -1; solo falta agregarlo a `critics`). Luego medirlo con el simulador R4-B en los mismos 6 escenarios × 2 variantes (R4-D). '
      f'Ataca la causa dominante: {bc.get("GOAL_TOLERANCE", 0)} GOAL_TOLERANCE y, por su `slowing_factor`, probablemente parte de los '
      f'{sub.get("SOBREPASO_AL_FRENAR", 0)} sobrepasos al frenar. Criterio de éxito: GOAL_TOLERANCE < 10/384 por variante sin aumentar contactos.\n')
    A('En segundo lugar, y por separado, los mínimos locales de around_obstacle (T023–T029): probar pesos GoalDist/GoalAlign vs PathDist/PathAlign. '
      'No se recomienda subir la tolerancia de yaw: resuelve pocas corridas, salvo desactivarla, y oculta el problema. Ningún cambio a producción sin tu confirmación.')
    open(os.path.join(LAB, 'DWB_R4C_FAILURE_ANALYSIS.md'), 'w').write('\n'.join(L) + '\n')
    print('ok')


if __name__ == '__main__':
    main()
