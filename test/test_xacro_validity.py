"""Xacro expands to valid URDF for every robot and sim mode."""

from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import pytest
import yaml

PKG_ROOT = Path(__file__).resolve().parent.parent
ROBOTS = ['hb50', 'hb50w', 'hb40']
SIM_MODES = ['none', 'mock', 'gazebo']
LEGS = ('fr', 'fl', 'rl', 'rr')


@pytest.mark.parametrize('robot', ROBOTS)
@pytest.mark.parametrize('sim', SIM_MODES)
def test_xacro_expands_to_valid_xml(robot, sim):
    xacro_file = PKG_ROOT / 'urdf' / robot / f'{robot}.urdf.xacro'
    result = subprocess.run(['xacro', str(xacro_file), f'sim:={sim}'],
                            capture_output=True, text=True)
    assert result.returncode == 0, result.stderr

    root = ET.fromstring(result.stdout)
    types = {j.get('name'): j.get('type') for j in root.findall('joint')}
    assert len(types) == len(root.findall('joint')), 'duplicated joint'
    # floating breaks urdf2sdf, continuous ignores limits (wheels excepted)
    movable = {n for n, t in types.items() if t != 'fixed'}
    for p in LEGS:
        for j in ('j0', 'j1', 'j2'):
            assert types.get(f'{p}_{j}') == 'revolute', f'{p}_{j}'
        if robot == 'hb50w':
            assert types.get(f'{p}_j3') == 'continuous', f'{p}_j3'
    assert len(movable) == (16 if robot == 'hb50w' else 12)

    ctrl = {j.get('name') for rc in root.iter('ros2_control') for j in rc.findall('joint')}
    assert ctrl == (set() if sim == 'none' else movable)


@pytest.mark.parametrize('robot', ROBOTS)
def test_generated_urdf_up_to_date(robot):
    xacro_file = PKG_ROOT / 'urdf' / robot / f'{robot}.urdf.xacro'
    result = subprocess.run(['xacro', str(xacro_file)],
                            capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    generated = ET.canonicalize(result.stdout, strip_text=True)
    committed = ET.canonicalize(from_file=PKG_ROOT / 'urdf' / robot / f'{robot}.urdf',
                                strip_text=True)
    assert generated == committed, 'run scripts/generate_urdf.sh'


@pytest.mark.parametrize('robot', ROBOTS)
def test_config_matches_urdf(robot):
    root = ET.parse(PKG_ROOT / 'urdf' / robot / f'{robot}.urdf').getroot()
    joints = {j.get('name'): j for j in root.findall('joint') if j.get('type') != 'fixed'}
    cfg = PKG_ROOT / 'config' / robot

    ctrl = yaml.safe_load((cfg / 'ros2_control.yaml').read_text())
    for name, c in ctrl.items():
        for j in c['ros__parameters'].get('joints', []):
            assert j in joints, f'{name}: {j}'

    limits = yaml.safe_load((cfg / 'joint_limits.yaml').read_text())['joints']
    assert set(limits) == set(joints)
    for j, lim in limits.items():
        ul = joints[j].find('limit')
        for k in ('lower', 'upper', 'velocity', 'effort'):
            assert (k in lim) == (ul.get(k) is not None), f'{j} {k}'
            if k in lim:
                assert abs(float(lim[k]) - float(ul.get(k))) < 1e-6, f'{j} {k}'
