"""Deterministic leader/follower control independent of ROS and serial devices."""
import math

from .ready_posture import ready_targets
from .hardware_limits import configured_joint_limits, tick_to_joint_position
from .gripper_spring import PressToClose


def finite_positions(names, values, required):
    if len(names) != len(values) or len(names) != len(set(names)):
        return None
    data = dict(zip(names, values))
    if any(n not in data or not math.isfinite(data[n]) for n in required):
        return None
    return {name: float(data[name]) for name in required}


class TeleopControl:
    def __init__(self, share, config, collision_check):
        self.gripper_enabled = config.get('gripper', {}).get('enabled', True)
        self.mapping = {key: list(values) for key, values in config['mapping'].items()}
        if not self.gripper_enabled:
            indices = [i for i, name in enumerate(self.mapping['follower_names'])
                       if name != 'gripper_pitch']
            self.mapping = {key: [values[i] for i in indices]
                            for key, values in self.mapping.items()}
        self.absolute_limits = configured_joint_limits(share)
        self.control = config['control']
        self.gripper = config.get('gripper') if self.gripper_enabled else None
        if self.gripper:
            self.trigger = PressToClose(self.gripper.get('press_threshold', 0.25),
                                        self.gripper.get('release_threshold', 0.12))
            keys = ('leader_closed_tick', 'leader_open_tick',
                    'follower_closed_tick', 'follower_open_tick')
            if any(not math.isfinite(self.gripper[k]) or not 0 <= self.gripper[k] <= 4095 for k in keys):
                raise ValueError('Invalid gripper reference ticks')
            if (self.gripper['leader_closed_tick'] == self.gripper['leader_open_tick'] or
                    self.gripper['follower_closed_tick'] == self.gripper['follower_open_tick']):
                raise ValueError('Gripper closed/open references must differ')
            self.gripper_leader_closed = (self.gripper['leader_closed_tick']-2048)*2*math.pi/4096
            self.gripper_leader_open = (self.gripper['leader_open_tick']-2048)*2*math.pi/4096
            self.gripper_follower_closed = tick_to_joint_position(share, 'gripper_pitch', self.gripper['follower_closed_tick'])
            self.gripper_follower_open = tick_to_joint_position(share, 'gripper_pitch', self.gripper['follower_open_tick'])
        self.arm = self.mapping['follower_names']
        self.leader = self.mapping['leader_names']
        fields = ('leader_names', 'follower_names', 'direction', 'scale',
                  'max_delta_rad', 'max_velocity_rad_s')
        count = 5 if self.gripper_enabled else 4
        if any(len(self.mapping[k]) != count for k in fields):
            raise ValueError(f'Shift teleop needs {count} mapped joints')
        if len(set(self.arm)) != count or len(set(self.leader)) != count:
            raise ValueError('Duplicate mapping joints')
        for field in ('direction', 'scale', 'max_delta_rad', 'max_velocity_rad_s'):
            if any(not math.isfinite(v) for v in self.mapping[field]):
                raise ValueError('Non-finite mapping values')
        if any(v <= 0 for field in ('scale', 'max_delta_rad', 'max_velocity_rad_s')
               for v in self.mapping[field]):
            raise ValueError('Mapping magnitudes must be positive')
        if any(v not in (-1, 1) for v in self.mapping['direction']):
            raise ValueError('Mapping direction must be +1 or -1')
        for value in self.control.values():
            if not math.isfinite(value) or value <= 0:
                raise ValueError('Control parameters must be finite and positive')
        names, values = ready_targets(share)
        self.lower = {n: v for n, v in zip(names, values)
                      if n.startswith(('L_', 'R_')) and not n.endswith('wheel')}
        self.wheels = ['R_wheel', 'L_wheel']
        self.names = list(self.lower) + self.arm + self.wheels
        self.hardware_missing = list(self.names)
        self.check = collision_check
        self.hardware = self.input = None
        self.hardware_time = self.input_time = -math.inf
        self.enabled = False
        self.output = None
        self.last_tick = None
        self.drift_since = None
        self.detail = 'DISABLED'
        self.blocked_contacts = []

    def update_hardware(self, names, positions, now):
        provided = dict(zip(names, positions))
        self.hardware_missing = [n for n in self.names
                                 if n not in provided or not math.isfinite(provided[n])]
        data = finite_positions(names, positions, self.names)
        if data is not None:
            self.hardware, self.hardware_time = data, now

    def update_leader(self, names, positions, now):
        data = finite_positions(names, positions, self.leader)
        if data is not None:
            self.input, self.input_time = data, now

    def enable(self, now):
        if self.enabled:
            return True, 'Already enabled; alignment retained'
        if self.gripper and not self.gripper.get('follower_endpoints_verified', False):
            return False, 'ID 17 overloaded: verify physical open/closed endpoints in config/teleop.yaml before enabling'
        if now - self.hardware_time > self.control['hardware_timeout_sec']:
            if self.hardware_missing:
                return False, 'Missing hardware joint feedback: ' + ', '.join(self.hardware_missing)
            return False, 'Fresh complete hardware feedback is required'
        if now - self.input_time > self.control['leader_timeout_sec']:
            return False, 'Fresh leader feedback is required'
        if max(abs(self.hardware[n] - v) for n, v in self.lower.items()) > self.control['lower_enable_tolerance_rad']:
            return False, 'Move the lower body to the standalone READY pose first'
        contacts = self.check(self.hardware)
        if contacts:
            return False, 'Current measured posture has a nonadjacent mesh intersection'
        if any(n in self.hardware and not low <= self.hardware[n] <= high
               for n, (low, high) in self.absolute_limits.items()):
            return False, 'Measured arm is outside configured Wizard limits'
        self.output = dict(self.hardware)
        self.anchor_arm = dict(self.hardware)
        self.anchor_leader = dict(self.input)
        self.enabled = True
        self.last_tick = now
        self.drift_since = None
        self.blocked_contacts = []
        if self.gripper:
            self.trigger.pressed = False
        self.detail = 'TELEOP'
        return True, 'Aligned to current measured arm and leader; READY lower body held'

    def disable(self, detail='DISABLED'):
        self.enabled = False
        self.detail = detail

    def tick(self, now):
        if not self.enabled:
            return None
        if now - self.hardware_time > self.control['hardware_timeout_sec']:
            self.disable('HARDWARE_STALE: retaining last motor goal; re-enable required')
            return None
        if now - self.input_time > self.control['leader_timeout_sec']:
            self.disable('LEADER_STALE: retaining last motor goal; re-enable required')
            return None
        drift = max(abs(self.hardware[n] - v) for n, v in self.lower.items())
        if drift > self.control['lower_drift_tolerance_rad']:
            if self.drift_since is None:
                self.drift_since = now
            elif now - self.drift_since >= self.control['lower_drift_duration_sec']:
                self.disable('LOWER_DRIFT: new arm motion stopped; re-enable required')
                return None
        else:
            self.drift_since = None
        # Do not catch up in one jump after a delayed timer callback.
        dt = max(0.0, min(now - self.last_tick, 1.0 / self.control['rate_hz']))
        self.last_tick = now
        candidate = dict(self.output)
        for n, goal in self.lower.items():
            step = self.control['lower_velocity_rad_s'] * dt
            candidate[n] += max(-step, min(step, goal - candidate[n]))
        for i, (source, target) in enumerate(zip(self.leader, self.arm)):
            relative = (self.input[source] - self.anchor_leader[source]) * self.mapping['direction'][i] * self.mapping['scale'][i]
            limit = self.mapping['max_delta_rad'][i]
            goal = self.anchor_arm[target] + max(-limit, min(limit, relative))
            if target == 'gripper_pitch' and self.gripper:
                fraction = (self.input[source]-self.gripper_leader_closed) / (
                    self.gripper_leader_open-self.gripper_leader_closed)
                fraction = max(0.0, min(1.0, fraction))
                if self.gripper.get('behavior') == 'press_to_close':
                    fraction = 0.0 if self.trigger.update(1.0-fraction) else 1.0
                goal = self.gripper_follower_closed + fraction * (
                    self.gripper_follower_open-self.gripper_follower_closed)
            if target in self.absolute_limits:
                low, high = self.absolute_limits[target]
                goal = max(low, min(high, goal))
            step = self.mapping['max_velocity_rad_s'][i] * dt
            candidate[target] += max(-step, min(step, goal - candidate[target]))
        # Check the complete step from both the last command and current actual
        # pose: lagging actuators need the second path, too.
        self.blocked_contacts = []
        for start in (self.output, self.hardware):
            count = max(1, math.ceil(max(abs(candidate[n] - start[n]) for n in self.names) / math.radians(0.5)))
            for index in range(1, count + 1):
                q = {n: start[n] + (candidate[n] - start[n]) * index / count for n in self.names}
                contacts = self.check(q)
                if contacts:
                    self.blocked_contacts = contacts
                    self.detail = 'COLLISION_BLOCKED'
                    # The CAD has no gripper geometry. An arm/body intersection
                    # must hold the arm but must not swallow the independent
                    # ID 26 -> ID 17 open/close command.
                    if 'gripper_pitch' in candidate:
                        self.output['gripper_pitch'] = candidate['gripper_pitch']
                    return dict(self.output)
        self.output = candidate
        self.detail = 'TELEOP'
        return dict(self.output)


class TeleopCollisionCheck:
    def __init__(self, share, library):
        from itertools import combinations
        from .mesh_interference import MeshInterference
        self.mesh = MeshInterference(share, library)
        self.pairs = [p for p in combinations(self.mesh.meshes, 2)
                      if frozenset(p) not in self.mesh.adjacent]

    def __call__(self, positions):
        return self.mesh.contacts(positions, pairs=self.pairs)

    def close(self):
        self.mesh.close()
