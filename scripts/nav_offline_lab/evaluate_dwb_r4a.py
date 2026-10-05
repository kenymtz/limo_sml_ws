#!/usr/bin/python3
"""NAV-OFFLINE-R4-A — análisis offline de resultados dwb_offline_eval (JSONL) -> CSV + resumen JSON + figuras.

Uso: evaluate_dwb_r4a.py --eval eval_full_R4A.jsonl --index cases_R4A_index.json --tag R4A
"""
import argparse
import collections
import csv
import json
import math
import os

import numpy as np

LETHAL = 254


def pct(v, q):
    return float(np.percentile(v, q)) if len(v) else None


def cmd_class(vx, wz):
    if vx is None or (isinstance(vx, float) and math.isnan(vx)):
        return 'none'
    if abs(vx) < 0.03:
        return 'rotate_in_place' if abs(wz) >= 0.1 else 'stop'
    return 'forward_turn' if abs(wz) >= 0.1 else 'forward'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--eval', required=True)
    ap.add_argument('--index', required=True)
    ap.add_argument('--tag', required=True)
    a = ap.parse_args()
    out_dir = os.path.dirname(os.path.abspath(a.eval))
    lines = [json.loads(x) for x in open(a.eval)]
    meta, R = lines[0]['meta'], lines[1:]
    idx = {c['case']: c for c in json.load(open(a.index))['cases']}
    rows = []
    for r in R:
        c = idx[r['case']]
        vx, wz = r['cmd']
        rows.append(dict(
            case=r['case'], test=c['test'], kind=c['kind'], category=c['category'],
            s_m=c.get('s_m'), path_len_m=c.get('path_len_m'), start_heading_err_deg=(
                math.degrees(c['start_heading_err_rad']) if 'start_heading_err_rad' in c else None),
            costmap_ready=r['costmap_ready'], status=r['status'], exc_type=r['exc_type'], exc_msg=r['exc_msg'],
            cmd_vx=vx, cmd_wz=wz, cmd_class=cmd_class(vx, wz) if r['status'] == 'ACCEPTED' else 'none',
            t_first_ms=r['times_ms'][0], t_med_ms=float(np.median(r['times_ms'])),
            pass2_match=(r['status'] != 'ACCEPTED') or (
                r['pass2_cmd'][0] is not None and abs(r['pass2_cmd'][0] - vx) < 1e-6 and abs(r['pass2_cmd'][1] - wz) < 1e-6),
            n_twists=r['n_twists'], n_legal=r['n_legal'],
            illegal_by_critic=';'.join(f'{k}:{v}' for k, v in r['illegal_by_critic'].items()),
            dominant_critic=r['dominant_critic'], decisive_critics=';'.join(r['decisive_critics']),
            margin=(r['second_total'] - r['best_total']) if r['best_total'] is not None and r['second_total'] is not None else None,
            center_cost=r['center_cost'], footprint_cost_at_pose=r['footprint_cost_at_pose'],
            legal_footprint_collide=r['legal_footprint_collide'], best_footprint_max_cost=r['best_footprint_max_cost'],
            best_fp_lethal=(r['best_footprint_max_cost'] is not None and r['best_footprint_max_cost'] >= LETHAL),
            **{f'best_{k}_w': v[0] * v[1] for k, v in r['best_scores'].items()}))
    keys = sorted({k for rw in rows for k in rw}, key=lambda k: list(rows[0].keys()).index(k) if k in rows[0] else 999)
    with open(os.path.join(out_dir, f'cases_{a.tag}.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)

    S = dict(tag=a.tag, meta=meta, n_cases=len(rows), costmap_not_ready=sum(not r['costmap_ready'] for r in rows),
             pass2_mismatch=sum(not r['pass2_match'] for r in rows))
    by_kind = collections.defaultdict(list)
    for r in rows:
        by_kind[r['kind']].append(r)
    S['by_kind'] = {}
    for k, rr in by_kind.items():
        acc = [r for r in rr if r['status'] == 'ACCEPTED']
        tf = [r['t_first_ms'] for r in rr]
        S['by_kind'][k] = dict(
            n=len(rr), accepted=len(acc), rejected=sum(r['status'] == 'REJECTED' for r in rr),
            error=sum(r['status'] == 'ERROR' for r in rr),
            t_first_ms=dict(p50=pct(tf, 50), p95=pct(tf, 95), max=max(tf)),
            t_med_ms_p50=pct([r['t_med_ms'] for r in rr], 50),
            cmd_class=dict(collections.Counter(r['cmd_class'] for r in rr)),
            dominant_critic=dict(collections.Counter(r['dominant_critic'] for r in acc)),
            decisive_critic_any=dict(collections.Counter(c for r in acc for c in r['decisive_critics'].split(';') if c)),
            no_decisive=sum(1 for r in acc if not r['decisive_critics']),
            rejection_reasons=dict(collections.Counter(
                (r['exc_msg'].split('\n', 1)[-1].strip()[:120] or r['exc_type']) for r in rr if r['status'] != 'ACCEPTED')),
            cases_with_legal_footprint_collision=sum(r['legal_footprint_collide'] > 0 for r in acc),
            best_traj_footprint_lethal=sum(r['best_fp_lethal'] for r in acc),
            pose_footprint_lethal=sum((r['footprint_cost_at_pose'] or 0) >= LETHAL for r in rr),
        )
    # por path (test): START + resumen de ALONG/PERT
    per_test = {}
    for r in rows:
        t = per_test.setdefault(r['test'], dict(test=r['test'], category=r['category'], start=None,
                                                along_n=0, along_acc=0, pert_n=0, pert_acc=0,
                                                along_best_fp_lethal=0, pert_best_fp_lethal=0))
        if r['kind'] == 'START':
            t['start'] = {k: r[k] for k in ('status', 'cmd_vx', 'cmd_wz', 'cmd_class', 't_first_ms', 't_med_ms',
                                            'dominant_critic', 'decisive_critics', 'exc_msg', 'start_heading_err_deg',
                                            'path_len_m', 'n_legal', 'n_twists', 'legal_footprint_collide',
                                            'best_fp_lethal')}
        elif r['kind'] == 'ALONG':
            t['along_n'] += 1
            t['along_acc'] += r['status'] == 'ACCEPTED'
            t['along_best_fp_lethal'] += bool(r['best_fp_lethal'])
        else:
            t['pert_n'] += 1
            t['pert_acc'] += r['status'] == 'ACCEPTED'
            t['pert_best_fp_lethal'] += bool(r['best_fp_lethal'])
    S['per_test'] = list(per_test.values())
    by_cat = collections.defaultdict(lambda: collections.Counter())
    for r in rows:
        c = by_cat[r['category']]
        c[f"{r['kind']}_n"] += 1
        c[f"{r['kind']}_acc"] += r['status'] == 'ACCEPTED'
        c[f"{r['kind']}_bestfp"] += bool(r['best_fp_lethal'])
    S['by_category'] = {k: dict(v) for k, v in by_cat.items()}
    json.dump(S, open(os.path.join(out_dir, f'summary_{a.tag}.json'), 'w'), indent=1, default=str)

    # figuras
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(1, 3, figsize=(16, 4.5))
        for k, rr in by_kind.items():
            ax[0].hist([r['t_first_ms'] for r in rr], bins=40, alpha=0.5, label=k)
        ax[0].axvline(100, color='r', ls='--', label='presupuesto 10 Hz')
        ax[0].set_xlabel('tiempo computeVelocityCommands (ms, 1ª llamada)')
        ax[0].legend()
        st = [r for r in rows if r['kind'] == 'START' and r['status'] == 'ACCEPTED']
        cols = {'rotate_in_place': 'tab:orange', 'forward_turn': 'tab:blue', 'forward': 'tab:green', 'stop': 'k'}
        ax[1].scatter([r['start_heading_err_deg'] for r in st], [r['cmd_vx'] for r in st],
                      c=[cols.get(r['cmd_class'], 'gray') for r in st])
        ax[1].set_xlabel('error de heading inicial vs path (deg)')
        ax[1].set_ylabel('vx comandado (m/s) en START (v0=0)')
        for n, cl in cols.items():
            ax[1].scatter([], [], c=cl, label=n)
        ax[1].legend(fontsize=7)
        cats = sorted(by_cat)
        kinds = [k for k in ('START', 'ALONG', 'PERT_LAT', 'PERT_YAW') if k in by_kind]
        wdt = 0.8 / len(kinds)
        for i, k in enumerate(kinds):
            ax[2].bar(np.arange(len(cats)) + i * wdt,
                      [by_cat[c][f'{k}_acc'] / max(1, by_cat[c][f'{k}_n']) for c in cats], wdt, label=k)
        ax[2].set_xticks(np.arange(len(cats)) + 0.4)
        ax[2].set_xticklabels(cats, rotation=30, fontsize=7)
        ax[2].set_ylabel('fracción aceptada')
        ax[2].set_ylim(0, 1.05)
        ax[2].legend(fontsize=7)
        fig.suptitle(f'NAV-OFFLINE-R4-A DWB offline — {a.tag}')
        fig.tight_layout()
        fig.savefig(os.path.join(out_dir, f'dwb_{a.tag}_overview.png'), dpi=110)
    except Exception as e:  # noqa
        print('figura no generada:', e)
    print(json.dumps({k: S[k] for k in ('tag', 'n_cases', 'costmap_not_ready', 'pass2_mismatch', 'by_kind')},
                     indent=1, default=str))


if __name__ == '__main__':
    main()
