"""Standalone READY targets in the current CAD joint coordinates."""
import math

from .trajectory import load_motion
from .hardware_limits import configured_joint_limits


def ready_targets(share):
    data = load_motion(share, 'transform')
    stage = data['stages'][0]
    if stage['name'] != 'ready':
        raise ValueError('The saved transform must start with READY')
    positions = dict(zip(data['joint_names'], stage['positions']))
    # With the feet fixed, -20 degrees at each hip pitches the pelvis +20.
    for side in ('L', 'R'):
        positions[f'{side}_hip_pitch'] -= math.radians(20.0)
    # The shoulder's link extends toward -X; the forearm extends toward +X.
    # Opposite joint signs therefore raise each link in its own local frame.
    # 10 degrees would map to 2162 ticks, below the user's Wizard limit 2200.
    positions.update(Arm_shoulder=math.radians(15.0),
                     Arm_elbow=math.radians(-10.0), Arm_waist=0.0)
    for joint, (low, high) in configured_joint_limits(share).items():
        if joint in positions and not low <= positions[joint] <= high:
            raise ValueError(f'READY {joint} lies outside configured motor limits')
    return data['joint_names'], [positions[name] for name in data['joint_names']]
