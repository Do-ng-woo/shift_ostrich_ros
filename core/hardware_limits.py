"""Convert configured raw-tick bounds to CAD joint angles; no EEPROM access."""
from pathlib import Path

import yaml


def configured_gripper_drive(share):
    cfg = yaml.safe_load((Path(share)/'config/hardware.yaml').read_text())
    motor = next(m for m in cfg['joints'] if m['joint_name'] == 'gripper_pitch')
    return dict(cfg.get('defaults', {}), **motor)


def gripper_drive_matches(drive, expected):
    if not isinstance(drive, dict):
        return False
    fields = {'mode': expected['operating_mode'], 'torque': 1, 'hardware_error': 0,
              'p_gain': expected['position_p_gain'], 'i_gain': expected['position_i_gain'],
              'd_gain': expected['position_d_gain']}
    return (all(drive.get(key) == value for key, value in fields.items())
            and 0 < drive.get('goal_current', 0) <= expected['goal_current'])


def tick_to_joint_position(share, joint, tick):
    cfg = yaml.safe_load((Path(share)/'config/hardware.yaml').read_text())
    motor = next(m for m in cfg['joints'] if m['joint_name'] == joint)
    center = motor.get('center_tick')
    direction = motor.get('direction')
    return (tick - (2048 if center is None else center)) / (
        (1 if direction is None else direction) * cfg['conversion']['tick_per_rad'])


def configured_joint_limits(share):
    cfg = yaml.safe_load((Path(share)/'config/hardware.yaml').read_text())
    factor = float(cfg['conversion']['tick_per_rad'])
    result = {}
    for motor in cfg['joints']:
        if not any(k in motor for k in ('min_position_tick', 'max_position_tick')):
            continue
        center = motor.get('center_tick')
        center = 2048 if center is None else int(center)
        direction = motor.get('direction')
        direction = 1 if direction is None else int(direction)
        low, high = int(motor.get('min_position_tick', 0)), int(motor.get('max_position_tick', 4095))
        if not 0 <= low <= high <= 4095 or direction not in (-1, 1):
            raise ValueError(f'Invalid position mapping for ID {motor["id"]}')
        values = [(tick-center)/(direction*factor) for tick in (low, high)]
        result[motor['joint_name']] = (min(values), max(values))
    return result
