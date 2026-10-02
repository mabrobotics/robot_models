"""mujoco/<robot>/ keeps the files and paths its README derivation steps rely on."""

from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

PKG_ROOT = Path(__file__).resolve().parent.parent
ROBOTS = sorted(p.parent.name for p in (PKG_ROOT / 'mujoco').glob('*/scene.xml'))
COMPILER = '<compiler angle="radian" autolimits="true"/>'


@pytest.mark.parametrize('robot', ROBOTS)
def test_menagerie_layout(robot):
    d = PKG_ROOT / 'mujoco' / robot
    for f in ('README.md', 'CHANGELOG.md', 'LICENSE', f'{robot}.png', f'{robot}.xml'):
        assert (d / f).is_file(), f
    assert (d / 'LICENSE').read_bytes() == (PKG_ROOT / 'LICENSE').read_bytes()
    assert '../' not in (d / 'scene.xml').read_text()

    xml = (d / f'{robot}.xml').read_text()
    assert xml.count(COMPILER) == 1
    prefix = f'../../meshes/{robot}/visual/'
    for m in ET.fromstring(xml).iter('mesh'):
        assert m.get('file').startswith(prefix), m.get('file')
        assert (d / m.get('file')).is_file(), m.get('file')
