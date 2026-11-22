"""Mesh geometry and URDF kinematics shared by export and spawning."""
import math
import struct
import numpy as np


def joint_poses(robot, positions):
    """URDF forward kinematics without changing any CAD origins."""
    links = {link.get('name'): link for link in robot.findall('link')}
    joints = robot.findall('joint')
    roots = set(links) - {j.find('child').get('link') for j in joints}
    if len(roots) != 1:
        raise ValueError('Expected a single URDF root')
    poses = {roots.pop(): np.eye(4)}
    pending = list(joints)
    while pending:
        ready = [j for j in pending if j.find('parent').get('link') in poses]
        if not ready:
            raise ValueError('Disconnected joint tree')
        for joint in ready:
            motion = np.eye(4)
            angle = positions.get(joint.get('name'), 0.)
            if angle:
                axis = np.fromstring(joint.find('axis').get('xyz'), sep=' ')
                axis /= np.linalg.norm(axis)
                x, y, z = axis
                skew = np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
                motion[:3,:3] = np.eye(3) + math.sin(angle)*skew + (1-math.cos(angle))*(skew@skew)
            poses[joint.find('child').get('link')] = poses[joint.find('parent').get('link')] @ transform(joint.find('origin')) @ motion
            pending.remove(joint)
    return poses


def posture_bounds(robot, package_root, positions):
    """Return actual collision vertex bounds after applying joint coordinates."""
    poses = joint_poses(robot, positions)
    links = {link.get('name'): link for link in robot.findall('link')}
    points = []
    for name, link in links.items():
        for collision in link.findall('collision'):
            mesh = collision.find('geometry/mesh')
            path = package_root / mesh.get('filename').removeprefix('package://shift_ostrich/')
            vertices = stl_vertices(path) * np.fromstring(mesh.get('scale', '1 1 1'), sep=' ')
            pose = poses[name] @ transform(collision.find('origin'))
            points.append(vertices @ pose[:3,:3].T + pose[:3,3])
    vertices = np.concatenate(points)
    return vertices.min(axis=0), vertices.max(axis=0)

def transform(origin):
    result = np.eye(4)
    if origin is None:
        return result
    r, p, y = map(float, origin.get('rpy', '0 0 0').split())
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    result[:3, :3] = [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                     [-sp, cp*sr, cp*cr]]
    result[:3, 3] = list(map(float, origin.get('xyz', '0 0 0').split()))
    return result


def stl_vertices(path):
    data = path.read_bytes()
    count = struct.unpack_from('<I', data, 80)[0] if len(data) >= 84 else 0
    if len(data) == 84 + count * 50:
        dtype = np.dtype([('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attr', '<u2')])
        return np.frombuffer(data, dtype=dtype, count=count, offset=84)['vertices'].reshape(-1, 3)
    return np.array([list(map(float, line.split()[1:]))
                     for line in data.decode().splitlines() if line.strip().startswith('vertex ')])

