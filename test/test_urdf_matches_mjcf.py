"""URDF (from xacro) matches the MJCF: inertials, joints, visuals, collisions."""

import math
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import pytest

PKG_ROOT = Path(__file__).resolve().parent.parent
ROBOTS = ['hb50', 'hb50w', 'hb40']
TOL = 1e-4
ROT_TOL = 1e-5


def vec(s, n=3):
    return [float(x) for x in s.split()] if s else [0.0] * n


def qmul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return [w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2]


def qaxis(axis, a):
    q = [math.cos(a / 2), 0.0, 0.0, 0.0]
    q[1 + axis] = math.sin(a / 2)
    return q


def rpy2q(rpy):
    return qmul(qaxis(2, rpy[2]), qmul(qaxis(1, rpy[1]), qaxis(0, rpy[0])))


def mj_rot(el):
    if el.get('quat'):
        q = vec(el.get('quat'), 4)
        n = math.sqrt(sum(v * v for v in q))
        return [v / n for v in q]
    if el.get('euler'):
        a, b, c = vec(el.get('euler'))
        return qmul(qaxis(0, a), qmul(qaxis(1, b), qaxis(2, c)))
    return [1.0, 0.0, 0.0, 0.0]


def qrot(q, v):
    return qmul(qmul(q, [0.0] + list(v)), [q[0], -q[1], -q[2], -q[3]])[1:]


def q2m(q):
    w, x, y, z = q
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]]


def rotate_inertia(q, inertia):
    R = q2m(q)
    return [[sum(R[i][k] * inertia[k][m] * R[j][m] for k in range(3) for m in range(3))
             for j in range(3)] for i in range(3)]


def close(a, b, tol=TOL):
    a, b = list(a), list(b)
    return len(a) == len(b) and all(abs(x - y) <= tol for x, y in zip(a, b))


def same_rot(a, b):
    return abs(sum(x * y for x, y in zip(a, b))) >= math.cos(ROT_TOL / 2)


def resolver(mjcf):
    """Return attrs(el): el's attributes with its default class (class/childclass) applied."""
    table = {}

    def rec(node, name, inherited):
        own = {t: dict(a) for t, a in inherited.items()}
        for c in node:
            if c.tag != 'default':
                own.setdefault(c.tag, {}).update(c.attrib)
        table[name] = own
        for c in node:
            if c.tag == 'default':
                rec(c, c.get('class'), own)

    if mjcf.find('default') is not None:
        rec(mjcf.find('default'), 'main', {})
    parent = {c: p for p in mjcf.iter() for c in p}

    def attrs(el):
        cls, p = el.get('class'), el
        while cls is None and p in parent:
            p = parent[p]
            cls = p.get('childclass') if p.tag == 'body' else None
        a = dict(table.get(cls or 'main', {}).get(el.tag, {}))
        a.update(el.attrib)
        return a

    return attrs


def compare(urdf, mjcf):
    bad = []
    links = {link.get('name'): link for link in urdf.findall('link')}
    joints = urdf.findall('joint')
    by_child = {j.find('child').get('link'): j for j in joints}
    colors = {m.get('name'): vec(m.find('color').get('rgba'), 4)
              for m in urdf.findall('material') if m.find('color') is not None}
    meshes = {m.get('name'): m.get('file').split('/')[-1] for m in mjcf.find('asset').iter('mesh')}
    materials = {m.get('name'): m.get('rgba') for m in mjcf.find('asset').iter('material')}
    attrs = resolver(mjcf)
    motors = {a.get('joint'): attrs(a) for a in mjcf.find('actuator').iter('motor')}

    def origin(el):
        o = el.find('origin') if el is not None else None
        if o is None:
            return [0.0] * 3, [1.0, 0.0, 0.0, 0.0]
        return vec(o.get('xyz')), rpy2q(vec(o.get('rpy')))

    def geoms(link, kind):
        out = []

        def rec(name, p, q):
            for g in links[name].findall(kind):
                gp, gq = origin(g)
                out.append((g, [a + b for a, b in zip(p, qrot(q, gp))], qmul(q, gq)))
            for j in joints:
                if j.get('type') == 'fixed' and j.find('parent').get('link') == name:
                    jp, jq = origin(j)
                    jp = [a + b for a, b in zip(p, qrot(q, jp))]
                    rec(j.find('child').get('link'), jp, qmul(q, jq))

        rec(link, [0.0] * 3, [1.0, 0.0, 0.0, 0.0])
        return out

    def shape(g):
        s = g.find('geometry')[0]
        if s.tag == 'box':
            return 'box', [v / 2 for v in vec(s.get('size'))]
        if s.tag == 'cylinder':
            return 'cylinder', [float(s.get('radius')), float(s.get('length')) / 2]
        if s.tag == 'sphere':
            return 'sphere', [float(s.get('radius'))]
        return 'mesh', s.get('filename').split('/')[-1]

    def effort(mj):
        a = motors[mj['name']]
        gear = float(a.get('gear', '1').split()[0])
        lims = []
        if mj.get('actuatorfrcrange'):
            lims.append(vec(mj.get('actuatorfrcrange'), 2)[1])
        if a.get('ctrlrange'):
            lims.append(gear * vec(a.get('ctrlrange'), 2)[1])
        if a.get('forcerange'):
            lims.append(gear * vec(a.get('forcerange'), 2)[1])
        return min(lims)

    for b in mjcf.iter('body'):
        n = b.get('name')
        if n not in links:
            bad.append(f'{n}: missing link')
            continue
        ui, mi = links[n].find('inertial'), b.find('inertial')
        if not close(origin(ui)[0], vec(mi.get('pos'))):
            bad.append(f'{n}: com')
        if abs(float(ui.find('mass').get('value')) - float(mi.get('mass'))) > TOL:
            bad.append(f'{n}: mass')
        i = {k: float(v) for k, v in ui.find('inertia').attrib.items()}
        ui_t = rotate_inertia(origin(ui)[1], [[i['ixx'], i['ixy'], i['ixz']],
                                              [i['ixy'], i['iyy'], i['iyz']],
                                              [i['ixz'], i['iyz'], i['izz']]])
        d = vec(mi.get('diaginertia'))
        mi_t = rotate_inertia(mj_rot(mi), [[d[0], 0, 0], [0, d[1], 0], [0, 0, d[2]]])
        if not close(sum(ui_t, []), sum(mi_t, []), 1e-9):
            bad.append(f'{n}: inertia')

        mj = b.find('joint')
        if mj is not None:  # root body has a <freejoint>
            mj = attrs(mj)
            j = by_child[n]
            jn = j.get('name')
            jp, jq = origin(j)
            if jn != mj.get('name'):
                bad.append(f'{n}: joint name {jn}')
            if not close(jp, vec(b.get('pos'))):
                bad.append(f'{jn}: pos')
            if not same_rot(jq, mj_rot(b)):
                bad.append(f'{jn}: rot')
            if not close(vec(j.find('axis').get('xyz')), vec(mj.get('axis'))):
                bad.append(f'{jn}: axis')
            lim = j.find('limit')
            if mj.get('range'):
                if j.get('type') != 'revolute' or lim is None:
                    bad.append(f"{jn}: type {j.get('type')}")
                elif not close([float(lim.get('lower')), float(lim.get('upper'))],
                               vec(mj.get('range'), 2)):
                    bad.append(f'{jn}: range')
            elif j.get('type') != 'continuous':
                bad.append(f"{jn}: type {j.get('type')}")
            if mj.get('name') not in motors:
                bad.append(f'{jn}: no actuator')
            elif lim is None or abs(float(lim.get('effort')) - effort(mj)) > TOL:
                bad.append(f'{jn}: effort')
            dyn = j.find('dynamics')
            dyn_ref = [float(mj.get('damping', 0)), float(mj.get('frictionloss', 0))]
            if dyn is None or not close([float(dyn.get('damping')), float(dyn.get('friction'))],
                                        dyn_ref):
                bad.append(f'{jn}: dynamics')

        mvis, mcol = [], []
        for gm in map(attrs, b.findall('geom')):
            typ = gm.get('type', 'sphere')
            size = meshes.get(gm.get('mesh')) if typ == 'mesh' else vec(gm.get('size'), 1)
            rgba = materials[gm['material']] if 'material' in gm else gm.get('rgba')
            item = (typ, size, vec(gm.get('pos')), mj_rot(gm), vec(rgba, 4))
            (mvis if gm.get('contype') == '0' else mcol).append(item)
        for kind, ms in (('visual', mvis), ('collision', mcol)):
            us = geoms(n, kind)
            if len(us) != len(ms):
                bad.append(f'{n}: {kind} count {len(us)} != {len(ms)}')
            for m in ms:
                hit = None
                for u in us:
                    ut, usz = shape(u[0])
                    if ut != m[0] or (usz != m[1] if ut == 'mesh' else not close(usz, m[1])):
                        continue
                    if close(u[1], m[2]) and same_rot(u[2], m[3]):
                        hit = u
                        break
                if hit is None:
                    bad.append(f'{n}: no {kind} {m[0]} {m[1]} at {m[2]}')
                    continue
                us.remove(hit)
                if kind == 'visual':
                    mat = hit[0].find('material')
                    if mat is None or not close(colors.get(mat.get('name'), []), m[4], 1e-3):
                        bad.append(f'{n}: visual color')

    bodies = {b.get('name') for b in mjcf.iter('body')}
    for j in joints:
        if j.get('type') != 'fixed' and j.find('child').get('link') not in bodies:
            bad.append(f"extra joint {j.get('name')}")

    # hb50 sim looks up actuator <j> and sensors <j>_pos/_vel/_trq for every joint
    refs = {
        'joint': {j.get('name') for j in mjcf.find('worldbody').iter('joint')},
        'actuator': {a.get('name') for a in mjcf.find('actuator')},
        'site': {st.get('name') for st in mjcf.iter('site')},
    }
    sensors = mjcf.find('sensor')
    for sn in sensors:
        for k, names in refs.items():
            if sn.get(k) is not None and sn.get(k) not in names:
                bad.append(f"sensor {sn.get('name')}: no {k} {sn.get(k)}")
    sensor_names = {sn.get('name') for sn in sensors}
    for jn in refs['joint']:
        if jn not in refs['actuator']:
            bad.append(f'{jn}: no actuator named after joint')
        for suffix in ('_pos', '_vel', '_trq'):
            if jn + suffix not in sensor_names:
                bad.append(f'{jn}: no sensor {jn}{suffix}')
    return bad


def expand(robot, *args):
    xacro_file = PKG_ROOT / 'urdf' / robot / f'{robot}.urdf.xacro'
    result = subprocess.run(['xacro', str(xacro_file), *args], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    return ET.fromstring(result.stdout)


def load_mjcf(robot):
    return ET.parse(PKG_ROOT / 'mujoco' / robot / f'{robot}.xml').getroot()


@pytest.mark.parametrize('robot', ROBOTS)
def test_urdf_matches_mjcf(robot):
    bad = compare(expand(robot), load_mjcf(robot))
    assert not bad, '\n'.join(bad)


@pytest.mark.parametrize('robot', ROBOTS)
def test_gazebo_imu_matches_mjcf_site(robot):
    imu = expand(robot, 'sim:=gazebo').find("joint[@name='imu_joint']")
    site = load_mjcf(robot).find(".//body[@name='body']/site[@name='imu']")
    assert imu.find('parent').get('link') == 'body'
    assert close(vec(imu.find('origin').get('xyz')), vec(site.get('pos')))
