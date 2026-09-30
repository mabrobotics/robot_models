"""URDF (from xacro) matches the RBDL Lua model: inertials, joint frames, axes."""

from pathlib import Path
import shutil
import subprocess

import pytest

from test_urdf_matches_mjcf import expand, q2m, rotate_inertia, rpy2q, vec

PKG_ROOT = Path(__file__).resolve().parent.parent
ROBOTS = ['hb50', 'hb50w', 'hb40']
LEGS = ('fr', 'fl', 'rl', 'rr')
TOL = 1e-4  # hb40 knee rotation matrices are written with 4 decimals

# One line per frame: name parent mass com[3] inertia[9] r[3] E[9] axis[3]
DUMP = """
local m = dofile(arg[1])
local function out(t) for _, v in ipairs(t) do io.write(' ', string.format('%.17g', v)) end end
local I3 = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}
local Z3 = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}
for _, f in ipairs(m.frames) do
  local b = f.body or {mass = 0, com = {0, 0, 0}, inertia = Z3}
  local jf = f.joint_frame or {}
  local E = jf.E or I3
  local ax = (f.joint and #f.joint == 1) and f.joint[1] or {0, 0, 0}
  io.write(f.name, ' ', f.parent)
  out({b.mass}); out(b.com); out(b.inertia[1]); out(b.inertia[2]); out(b.inertia[3])
  out(jf.r or {0, 0, 0}); out(E[1]); out(E[2]); out(E[3]); out({ax[1], ax[2], ax[3]})
  print()
end
"""


def load_lua(robot):
    result = subprocess.run(['lua5.1', '-', str(PKG_ROOT / 'rbdl' / robot / f'{robot}.lua')],
                            input=DUMP, capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    frames = []
    for line in result.stdout.splitlines():
        name, parent, *v = line.split()
        v = [float(x) for x in v]
        frames.append({'name': name, 'parent': parent, 'mass': v[0], 'com': v[1:4],
                       'inertia': [v[4:7], v[7:10], v[10:13]], 'r': v[13:16],
                       'E': [v[16:19], v[19:22], v[22:25]], 'axis': v[25:28]})
    return frames


def close(a, b, tol=TOL):
    return all(abs(x - y) <= tol for x, y in zip(a, b))


def origin(el):
    o = el.find('origin')
    if o is None:
        return [0.0] * 3, [0.0] * 3
    return vec(o.get('xyz')), vec(o.get('rpy'))


def compare(urdf, frames):
    bad = []
    links = {link.get('name'): link for link in urdf.findall('link')}
    joints = {j.get('name'): j for j in urdf.findall('joint')}
    link_of = {'base': 'body'}

    for f in frames:
        n = f['name']
        if n == 'base':
            link = 'body'
        else:
            parent = link_of.get(f['parent'])
            if n in joints:
                j = joints[n]
            else:  # *_foot: the one URDF joint hanging off the shin
                below = [j for j in joints.values() if j.find('parent').get('link') == parent]
                if len(below) != 1:
                    bad.append(f'{n}: no unique URDF joint below {parent}')
                    continue
                j = below[0]
            link = j.find('child').get('link')
            if j.find('parent').get('link') != parent:
                bad.append(f'{n}: parent')
            xyz, rpy = origin(j)
            if not close(xyz, f['r']):
                bad.append(f'{n}: r')
            R = q2m(rpy2q(rpy))  # RBDL E is the parent->child rotation, i.e. R^T
            if not close(sum(f['E'], []), [R[c][r] for r in range(3) for c in range(3)]):
                bad.append(f'{n}: E')
            if any(f['axis']) and not close(vec(j.find('axis').get('xyz')), f['axis']):
                bad.append(f'{n}: axis')
        link_of[n] = link

        if f['mass'] == 0:  # frame without a body
            continue
        ui = links[link].find('inertial')
        xyz, rpy = origin(ui)
        if abs(float(ui.find('mass').get('value')) - f['mass']) > TOL:
            bad.append(f'{n}: mass')
        if not close(xyz, f['com']):
            bad.append(f'{n}: com')
        i = {k: float(v) for k, v in ui.find('inertia').attrib.items()}
        ui_t = rotate_inertia(rpy2q(rpy), [[i['ixx'], i['ixy'], i['ixz']],
                                           [i['ixy'], i['iyy'], i['iyz']],
                                           [i['ixz'], i['iyz'], i['izz']]])
        if not close(sum(ui_t, []), sum(f['inertia'], []), 1e-6):
            bad.append(f'{n}: inertia')
    return bad


def link_mass(link):
    ui = link.find('inertial')
    return 0.0 if ui is None else float(ui.find('mass').get('value'))


@pytest.mark.skipif(shutil.which('lua5.1') is None, reason='lua5.1 not installed')
@pytest.mark.parametrize('robot', ROBOTS)
def test_urdf_matches_rbdl(robot):
    urdf = expand(robot)
    frames = load_lua(robot)
    bad = compare(urdf, frames)
    assert not bad, '\n'.join(bad)

    links = {link.get('name'): link for link in urdf.findall('link')}
    urdf_total = sum(link_mass(link) for link in links.values())
    if robot == 'hb50w':  # known gap: wheels (*_w) are not modelled in RBDL
        urdf_total -= sum(link_mass(links[f'{p}_w']) for p in LEGS)
    assert abs(sum(f['mass'] for f in frames) - urdf_total) < TOL
