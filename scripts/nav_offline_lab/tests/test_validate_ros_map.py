"""Tests de validate_ros_map.py. Ejecutar:  /usr/bin/python3 -m pytest -q scripts/nav_offline_lab/tests"""
import os
import subprocess
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import validate_ros_map as V  # noqa

SCRIPT = os.path.join(os.path.dirname(HERE), 'validate_ros_map.py')
WS = os.path.expanduser('~/limo_sml_ws')


def write_pgm(path, arr):
    arr = np.asarray(arr, dtype=np.uint8)
    with open(path, 'wb') as f:
        f.write(f'P5\n{arr.shape[1]} {arr.shape[0]}\n255\n'.encode())
        f.write(arr.tobytes())


def write_yaml(path, image='m.pgm', res=0.05, free=0.10, occ=0.65, origin='[0.0, 0.0, 0]', negate=0,
               mode='trinary'):
    with open(path, 'w') as f:
        f.write(f'image: {image}\nmode: {mode}\nresolution: {res}\norigin: {origin}\n'
                f'negate: {negate}\noccupied_thresh: {occ}\nfree_thresh: {free}\n')


def trinary_map():
    a = np.full((20, 30), 205, np.uint8)
    a[5:15, 5:25] = 254
    a[5, 5:25] = 0
    return a


def run(*args):
    return subprocess.run([sys.executable, SCRIPT, *args], capture_output=True, text=True).returncode


@pytest.fixture
def good(tmp_path):
    write_pgm(tmp_path / 'm.pgm', trinary_map())
    write_yaml(tmp_path / 'm.yaml')
    return tmp_path


def test_good_map_passes(good):
    r = V.validate(str(good / 'm.yaml'))
    assert r['status'] == 'PASS', r
    assert r['nav2_counts']['UNKNOWN'] == (trinary_map() == 205).sum()
    assert r['nav2_counts']['OCCUPIED'] == 20
    assert run('--map-yaml', str(good / 'm.yaml')) == 0


def test_free_025_makes_205_free_critical(good):
    write_yaml(good / 'm.yaml', free=0.25)
    r = V.validate(str(good / 'm.yaml'))
    assert r['status'] == 'FAIL'
    assert r['nav2_counts']['UNKNOWN'] == 0
    assert any('205' in c for c in r['critical'])
    assert run('--map-yaml', str(good / 'm.yaml')) == 2


def test_free_0196_fragile_warning_and_strict(good):
    write_yaml(good / 'm.yaml', free=0.196)
    r = V.validate(str(good / 'm.yaml'))
    assert r['nav2_counts']['UNKNOWN'] == (trinary_map() == 205).sum()
    assert any('frágil' in w for w in r['warnings'])
    assert run('--map-yaml', str(good / 'm.yaml'), '--strict') == 1


def test_free_010_no_fragile_warning(good):
    write_yaml(good / 'm.yaml', free=0.10)
    r = V.validate(str(good / 'm.yaml'))
    assert r['status'] == 'PASS', r
    assert not any('frágil' in w for w in r['warnings'])


def test_missing_yaml(tmp_path):
    assert run('--map-yaml', str(tmp_path / 'nope.yaml')) == 2


def test_missing_image(tmp_path):
    write_yaml(tmp_path / 'm.yaml', image='ausente.pgm')
    r = V.validate(str(tmp_path / 'm.yaml'))
    assert r['status'] == 'FAIL' and any('no existe' in c for c in r['critical'])


@pytest.mark.parametrize('res', [0, -0.05, '.nan'])
def test_bad_resolution(good, res):
    write_yaml(good / 'm.yaml', res=res)
    assert V.validate(str(good / 'm.yaml'))['status'] == 'FAIL'


def test_bad_origin(good):
    write_yaml(good / 'm.yaml', origin='[0.0, 0.0]')
    assert V.validate(str(good / 'm.yaml'))['status'] == 'FAIL'


def test_bad_thresholds(good):
    write_yaml(good / 'm.yaml', free=0.7, occ=0.65)
    assert V.validate(str(good / 'm.yaml'))['status'] == 'FAIL'


def test_bad_negate(good):
    write_yaml(good / 'm.yaml', negate=3)
    assert V.validate(str(good / 'm.yaml'))['status'] == 'FAIL'


def test_expect_resolution(good):
    assert V.validate(str(good / 'm.yaml'), expect_resolution=0.03)['status'] == 'FAIL'
    assert V.validate(str(good / 'm.yaml'), expect_resolution=0.05)['status'] == 'PASS'


def test_image_mismatch_with_sibling(tmp_path):
    # caso real: map1.yaml -> map.pgm existiendo map1.pgm
    write_pgm(tmp_path / 'map.pgm', trinary_map())
    write_pgm(tmp_path / 'map1.pgm', trinary_map()[:10])
    write_yaml(tmp_path / 'map1.yaml', image='map.pgm')
    r = V.validate(str(tmp_path / 'map1.yaml'))
    assert r['status'] == 'FAIL' and any('no corresponde' in c for c in r['critical'])


def test_shared_pgm_different_resolution_warns(tmp_path):
    write_pgm(tmp_path / 'map.pgm', trinary_map())
    write_yaml(tmp_path / 'map.yaml', image='map.pgm', res=0.03)
    write_yaml(tmp_path / 'other.yaml', image='map.pgm', res=0.05)
    r = V.validate(str(tmp_path / 'map.yaml'))
    assert any('resolución 0.05' in w for w in r['warnings'])
    assert r['status'] == 'WARN'
    assert run('--map-yaml', str(tmp_path / 'map.yaml')) == 0
    assert run('--map-yaml', str(tmp_path / 'map.yaml'), '--strict') == 1


def test_gray_levels_warning(good):
    a = trinary_map()
    a[10, 10] = 215  # FREE con 0.196, UNKNOWN con 0.10
    write_pgm(good / 'm.pgm', a)
    write_yaml(good / 'm.yaml', free=0.10)
    r = V.validate(str(good / 'm.yaml'))
    assert any('niveles de gris' in w for w in r['warnings'])
    assert r['level_classification']['215'] == 'UNKNOWN'
    write_yaml(good / 'm.yaml', free=0.196)
    assert V.validate(str(good / 'm.yaml'))['level_classification']['215'] == 'FREE'


def test_truncated_pgm(tmp_path):
    with open(tmp_path / 'm.pgm', 'wb') as f:
        f.write(b'P5\n30 20\n255\n' + b'\xfe' * 100)
    write_yaml(tmp_path / 'm.yaml')
    assert V.validate(str(tmp_path / 'm.yaml'))['status'] == 'FAIL'


def test_pgm_with_comment(tmp_path):
    a = trinary_map()
    with open(tmp_path / 'm.pgm', 'wb') as f:
        f.write(b'P5\n# CREATOR: test\n30 20\n255\n' + a.tobytes())
    write_yaml(tmp_path / 'm.yaml')
    assert V.validate(str(tmp_path / 'm.yaml'))['status'] == 'PASS'


def test_classification_matches_nav2_formula():
    c = V.classify_levels([0, 89, 90, 191, 192, 205, 206, 229, 230, 254, 255], 'trinary', False, 0.65, 0.25)
    assert c[0] == c[89] == 'OCCUPIED' and c[90] == c[191] == 'UNKNOWN'
    assert c[192] == c[205] == 'FREE'
    c = V.classify_levels([205, 206, 229, 230], 'trinary', False, 0.65, 0.10)
    assert c[205] == c[206] == c[229] == 'UNKNOWN' and c[230] == 'FREE'


# ---- integración con artefactos reales (solo lectura) ----
ARENA = os.path.join(WS, 'results/maps/arena_20260918_1900/map.yaml')
TEST_UNK = os.path.join(WS, 'results/navigation_offline_lab/maps_test/map_test_unknown.yaml')


@pytest.mark.skipif(not os.path.exists(ARENA), reason='dataset no disponible')
def test_real_arena_original_fails():
    r = V.validate(ARENA)
    assert r['status'] == 'FAIL' and r['nav2_counts'] == {'FREE': 126444, 'OCCUPIED': 1908,
                                                          'UNKNOWN': 0, 'INTERMEDIATE': 0}


@pytest.mark.skipif(not os.path.exists(TEST_UNK), reason='copia derivada no disponible')
def test_real_arena_derived_matches_map_server():
    # valores medidos con nav2_map_server real (maps_test/map_server_test_unknown.json)
    r = V.validate(TEST_UNK)
    assert r['nav2_counts'] == {'FREE': 34676, 'OCCUPIED': 1908, 'UNKNOWN': 91768, 'INTERMEDIATE': 0}
    assert not r['critical']
