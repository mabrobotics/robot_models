"""Every MJCF scene compiles and steps without MuJoCo warnings, as Menagerie's CI checks."""

from pathlib import Path

import pytest

try:
    import mujoco
except ImportError:
    mujoco = None

pytestmark = pytest.mark.skipif(not hasattr(mujoco, 'MjModel'),
                                reason='mujoco Python package not installed')

PKG_ROOT = Path(__file__).resolve().parent.parent
SCENES = sorted((PKG_ROOT / 'mujoco').glob('*/scene*.xml'))
ROBOTS = sorted(p.parent.name for p in (PKG_ROOT / 'mujoco').glob('*/scene.xml'))


def ctrl_noise(m, d, i):
    for j in range(m.nu):
        lo, hi = m.actuator_ctrlrange[j]
        c, r = ((lo + hi) / 2, (hi - lo) / 2) if m.actuator_ctrllimited[j] else (0.0, 1.0)
        d.ctrl[j] = c + r * (2 * mujoco.mju_Halton(i, j + 2) - 1)


@pytest.mark.parametrize('xml', SCENES, ids=lambda p: f'{p.parent.name}/{p.name}')
def test_steps_without_warnings(xml):
    m = mujoco.MjModel.from_xml_path(str(xml))
    d = mujoco.MjData(m)
    i = 0
    while d.time < 0.1:
        ctrl_noise(m, d, i)
        mujoco.mj_step(m, d)
        i += 1
    warnings = {mujoco.mjtWarning(k).name: n for k, n in enumerate(d.warning.number) if n}
    assert not warnings, warnings


@pytest.mark.parametrize('robot', ROBOTS)
def test_home_keyframe_stands_on_floor(robot):
    m = mujoco.MjModel.from_xml_path(str(PKG_ROOT / 'mujoco' / robot / 'scene.xml'))
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, m.key('home').id)
    mujoco.mj_forward(m, d)
    for j in range(m.njnt):
        if m.jnt_limited[j]:
            lo, hi = m.jnt_range[j]
            assert lo <= d.qpos[m.jnt_qposadr[j]] <= hi, m.joint(j).name
    floor = m.geom('floor').id
    tag = 'wheel' if robot == 'hb50w' else 'foot'
    for leg in ('fr', 'fl', 'rl', 'rr'):
        dist = mujoco.mj_geomDistance(m, d, m.geom(f'{leg}_{tag}').id, floor, 1.0, None)
        assert abs(dist) < 2e-3, (leg, dist)
