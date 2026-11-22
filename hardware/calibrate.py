#!/usr/bin/env python3
"""Read-only inspection by default; interactive hold and small raw-axis tests."""
import argparse
import fcntl
import json
import math
from pathlib import Path
import time

import yaml

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / 'config' / 'hardware.yaml'
SUPPORTED = {1020: 'XM430-W350', 1030: 'XM430-W210', 1060: 'XL430-W250',
             1200: 'XL330-M288'}


class Bus:
    def __init__(self, device, baudrate):
        from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler
        self.success = COMM_SUCCESS
        self.port = PortHandler(device)
        self.packet = PacketHandler(2.0)
        self.lock = open('/tmp/shift_ostrich_calibration.lock', 'w')
        fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        if not self.port.openPort():
            raise RuntimeError(f'Cannot open {device}')
        if not self.port.setBaudRate(baudrate):
            self.port.closePort()
            raise RuntimeError('Cannot set baudrate')
        time.sleep(0.3)

    def check(self, result, error):
        if result != self.success or error:
            raise RuntimeError(self.packet.getTxRxResult(result) if result != self.success
                               else self.packet.getRxPacketError(error))

    def read(self, mid, address, size):
        value, result, error = getattr(self.packet, f'read{size}ByteTxRx')(self.port, mid, address)
        self.check(result, error)
        return value

    def write(self, mid, address, size, value):
        result, error = getattr(self.packet, f'write{size}ByteTxRx')(self.port, mid, address, value)
        self.check(result, error)

    def inspect(self, motors):
        states = {}
        for motor in motors:
            mid = motor['id']
            try:
                model, result, error = self.packet.ping(self.port, mid)
                self.check(result, error)
                if model not in SUPPORTED:
                    raise RuntimeError(f'Unverified model {model}; register access blocked')
                states[mid] = dict(model=model, mode=self.read(mid, 11, 1),
                                   drive_mode=self.read(mid, 10, 1),
                                   torque=self.read(mid, 64, 1), error=self.read(mid, 70, 1),
                                   position=self.read(mid, 132, 4),
                                   voltage=self.read(mid, 144, 2) / 10,
                                   temperature=self.read(mid, 146, 1))
                print(f'{mid:2} {motor["joint_name"]:18} {states[mid]}', flush=True)
            except RuntimeError as exc:
                print(f'{mid:2} {motor["joint_name"]:18} FAILED: {exc}', flush=True)
        return states


def enable_hold(bus, motors, states, enabled):
    # Preflight all position joints before writing anything. Wheels remain untouched.
    joints = [m for m in motors if m['control_mode'] == 'position']
    for motor in joints:
        mid = motor['id']
        state = states.get(mid)
        if state is None or state['mode'] != 3 or state['error'] or state['drive_mode'] & 8:
            raise RuntimeError(f'ID {mid}: requires healthy position mode 3, '
                               'and torque-on-by-goal disabled. No automatic mode changes.')
        if not ((3.7 <= state['voltage'] <= 6) if state['model'] == 1200
                else (10 <= state['voltage'] <= 14.8)):
            raise RuntimeError(f'ID {mid}: voltage outside supported model range')
        if state['torque']:
            raise RuntimeError(f'ID {mid}: already torque-enabled; inspect existing controller first')
    for motor in joints:
        mid = motor['id']
        position = bus.read(mid, 132, 4) % 4096
        low, high = bus.read(mid, 52, 4), bus.read(mid, 48, 4)
        if not low <= position <= high:
            raise RuntimeError(f'ID {mid}: present position outside configured limits')
    for motor in joints:
        mid = motor['id']
        # Position mode resets the encoder to single-turn on torque enable.
        position = bus.read(mid, 132, 4) % 4096
        bus.write(mid, 116, 4, position)
        enabled.add(mid)  # Include this ID in cleanup even if the acknowledgement is lost.
        bus.write(mid, 64, 1, 1)
        if bus.read(mid, 64, 1) != 1:
            raise RuntimeError(f'ID {mid}: torque did not enable')
    print('Current-position hold ON. Wheels 6/12 unchanged. No posture or zero command sent.')


def jog(bus, mid, degrees, enabled, baseline):
    if mid not in enabled or mid in (6, 12):
        raise ValueError('Hold must be ON; wheel tests are excluded')
    if not math.isfinite(degrees) or not 0 < abs(degrees) <= 2:
        raise ValueError('Use a raw motor step between -2 and +2 degrees')
    if bus.read(mid, 10, 1) & 4:
        raise ValueError('Time-based profile: configure and verify velocity profile before jogging')
    current = bus.read(mid, 132, 4)
    if abs(current - baseline[mid]) > 57:
        raise ValueError('More than 5 degrees from hold position; stop and inspect')
    target = current + round(degrees * 4096 / 360)
    if abs(target - baseline[mid]) > 57:
        raise ValueError('Total movement limited to 5 degrees from hold position')
    low, high = bus.read(mid, 52, 4), bus.read(mid, 48, 4)
    if not low <= target <= high:
        raise ValueError('Target outside motor position limits')
    # Stream small goals at <= 2 deg/s, without changing stored profile/gain settings.
    steps = max(1, abs(target - current))
    for index in range(1, steps + 1):
        if bus.read(mid, 70, 1):
            raise RuntimeError(f'ID {mid}: hardware error')
        bus.write(mid, 116, 4, round(current + (target - current) * index / steps))
        time.sleep(abs(degrees) / (2 * steps))
    print(f'ID {mid}: raw {degrees:+g} degrees, target tick {target}; observe URDF direction')


def prepare(bus, motors):
    states = bus.inspect(motors)
    if len(states) != len(motors) or any(s['torque'] or s['error'] for s in states.values()):
        raise RuntimeError('All motors must respond, be healthy, and have torque OFF')
    backup = ROOT / 'hardware' / 'logs' / f'preparation_{time.time_ns()}.json'
    backup.parent.mkdir(exist_ok=True)
    backup.write_text(json.dumps(states, indent=2))
    for motor in motors:
        mid = motor['id']
        expected = 1 if motor['control_mode'] == 'velocity' else 3
        if states[mid]['mode'] != expected:
            bus.write(mid, 11, 1, expected)
            if bus.read(mid, 11, 1) != expected:
                raise RuntimeError(f'ID {mid}: operating mode readback mismatch')
    print(f'Modes aligned; torque remains OFF. Previous state: {backup}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port')
    parser.add_argument('--interactive', action='store_true')
    args = parser.parse_args()
    config = yaml.safe_load(CONFIG.read_text())
    bus = Bus(args.port or config['bus']['device'], config['bus']['baudrate'])
    enabled = set()
    try:
        states = bus.inspect(config['joints'])
        if not args.interactive:
            return
        print('Support the robot. Commands: status | prepare | hold | hold-body | jog ID DEG | direction ID +1/-1 | off | quit')
        print('jog uses raw encoder sign, NOT calibrated URDF sign. direction saves software mapping only.')
        baseline = {}
        while True:
            try:
                command = input('hardware> ').split()
                if not command:
                    continue
                if command == ['quit']:
                    break
                if command == ['status']:
                    states = bus.inspect(config['joints'])
                elif command == ['prepare']:
                    prepare(bus, config['joints'])
                elif command in (['hold'], ['hold-body']):
                    states = bus.inspect(config['joints'])
                    motors = [m for m in config['joints'] if command == ['hold'] or m['id'] != 17]
                    enable_hold(bus, motors, states, enabled)
                    baseline = {mid: bus.read(mid, 132, 4) for mid in enabled}
                elif command == ['off']:
                    for mid in list(enabled):
                        bus.write(mid, 64, 1, 0)
                        enabled.remove(mid)
                elif len(command) == 3 and command[0] == 'jog':
                    jog(bus, int(command[1]), float(command[2]), enabled, baseline)
                elif len(command) == 3 and command[0] == 'direction':
                    mid, direction = int(command[1]), int(command[2])
                    if direction not in (-1, 1):
                        raise ValueError('Direction must be +1 or -1')
                    motor = next(m for m in config['joints'] if m['id'] == mid)
                    if mid not in enabled:
                        raise ValueError('Confirm a held position joint after observing its jog')
                    motor.update(direction=direction, direction_checked=True)
                    CONFIG.write_text(yaml.safe_dump(config, allow_unicode=True, sort_keys=False))
                    print(f'Saved ID {mid} direction={direction}; zero remains uncalibrated')
                else:
                    print('Unknown command')
            except (ValueError, StopIteration) as exc:
                print(f'Rejected: {exc}')
    finally:
        # SDK can leave its in-process busy flag set after a serial I/O exception.
        bus.port.is_using = False
        for mid in sorted(enabled):
            try:
                bus.write(mid, 64, 1, 0)
            except Exception as exc:
                print(f'TORQUE OFF FAILED ID {mid}: {exc}', flush=True)
                bus.port.is_using = False
        bus.port.closePort()
        bus.lock.close()


if __name__ == '__main__':
    try:
        main()
    except (KeyboardInterrupt, EOFError):
        print('\nExited; attempted torque OFF for motors enabled by this tool.')
    except Exception as exc:
        raise SystemExit(str(exc))
