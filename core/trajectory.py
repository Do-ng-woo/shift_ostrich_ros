"""Read the saved Gazebo trajectories without regenerating their targets."""
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET


def load_motion(share, motion, trajectory_file=''):
    share = Path(share)
    if motion not in ('transform', 'revert'):
        raise ValueError('Unknown motion')
    path = Path(trajectory_file) if trajectory_file else share/'config/motions'/f'{motion}.json'
    data = json.loads(path.read_text())
    if data.get('motion') != motion:
        raise ValueError('Trajectory direction does not match the command')
    if not trajectory_file:
        manifest = json.loads((share/'config/motions/verified.json').read_text())
        if hashlib.sha256(path.read_bytes()).hexdigest() != manifest['motions'][motion]['sha256']:
            raise ValueError('Saved trajectory has changed since verification')
    description = share/'urdf/shift_ostrich.urdf'
    if hashlib.sha256(description.read_bytes()).hexdigest() != data['model_sha256']:
        raise ValueError('CAD has changed; verify the motion for the new model first')
    for name, expected in data['mesh_sha256'].items():
        if hashlib.sha256((share/'meshes'/name).read_bytes()).hexdigest() != expected:
            raise ValueError(f'Mesh {name} has changed since motion verification')
    robot = ET.parse(description).getroot()
    actual = {j.get('name') for j in robot.findall('joint') if j.get('type') != 'fixed'}
    names = data['joint_names']
    if len(names) != len(set(names)) or set(names) != actual:
        raise ValueError('Trajectory joint names do not match this robot')
    previous = 0.
    if not data['stages']:
        raise ValueError('Empty trajectory')
    for stage in data['stages']:
        positions, seconds = stage['positions'], stage['time_sec']
        if len(positions) != len(names) or not all(math.isfinite(x) for x in positions):
            raise ValueError('Invalid waypoint positions')
        if not math.isfinite(seconds) or seconds <= previous:
            raise ValueError('Waypoint times must be positive and increasing')
        if any(positions[names.index(n)] != 0. for n in ('L_wheel','R_wheel')):
            raise ValueError('Wheels must remain stationary in these motions')
        previous = seconds
    return data
