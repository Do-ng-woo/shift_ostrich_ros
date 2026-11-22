"""OMX leader trigger: motor-internal current-limited position control.

Reference: ROBOTIS-contributed OMX leader in Hugging Face LeRobot,
src/lerobot/teleoperators/omx_leader/omx_leader.py (Apache-2.0).
Mode 5 and 100 mA follow that sample. P/I/D=400/0/400 are the
ROBOTIS XL330-M077 e-Manual defaults, not OMY spring-controller gains.
Preserve this robot's calibrated Drive Mode/Homing Offset and open tick.
"""
import math


class PressToClose:
    def __init__(self, press=0.25, release=0.12):
        if not 0 <= release < press <= 1:
            raise ValueError('Need 0 <= release < press <= 1')
        self.press, self.release = press, release
        self.pressed = False

    def update(self, closure):
        if not math.isfinite(closure):
            raise ValueError('Non-finite trigger input')
        if self.pressed and closure <= self.release:
            self.pressed = False
        elif not self.pressed and closure >= self.press:
            self.pressed = True
        return self.pressed


class DynamixelTriggerSpring:
    """Own ID 26 only; use firmware PD, then restore mode with torque OFF."""
    def __init__(self, port, packet, config, now):
        self.port, self.packet, self.cfg = port, packet, config
        self.id = int(config.get('id', 26))
        if self.id != 26:
            raise ValueError('Open-return trigger is restricted to leader ID 26')
        if config.get('controller') != 'omx_current_based_position':
            raise ValueError('Only the OMX firmware position trigger is supported')
        if config.get('max_current_ma') != 100:
            raise ValueError('OMX leader sample requires a 100 mA current cap')
        if (config.get('position_p_gain'), config.get('position_i_gain'),
                config.get('position_d_gain')) != (400, 0, 400):
            raise ValueError('Leader requires XL330-M077 documented P/I/D=400/0/400')
        if not 0 <= config['open_tick'] <= 4095 or config['ramp_sec'] <= 0:
            raise ValueError('Invalid open position or ramp')
        self.started = now
        self.saved = None
        self.active = False

    def read(self, address, size):
        value, comm, error = getattr(self.packet, f'read{size}ByteTxRx')(self.port, self.id, address)
        if comm != 0 or error:
            raise RuntimeError(f'Leader 26 read failed at {address}: comm={comm}, error={error}')
        return value

    def write(self, address, size, value):
        comm, error = getattr(self.packet, f'write{size}ByteTxRx')(
            self.port, self.id, address, int(value) & ((1 << (size*8))-1))
        if comm != 0 or error:
            raise RuntimeError(f'Leader 26 write failed at {address}: comm={comm}, error={error}')

    def start(self):
        model, comm, error = self.packet.ping(self.port, self.id)
        if comm != 0 or error or model != 1190:
            raise RuntimeError('OMX leader trigger profile requires XL330-M077 (1190)')
        if self.read(64, 1):
            raise RuntimeError('Leader 26 is already torque enabled; stop its existing controller first')
        if self.read(70, 1):
            raise RuntimeError('Leader 26 reports a hardware fault')
        mode = self.read(11, 1)
        limit = self.read(38, 2)
        if limit < self.cfg['max_current_ma']:
            raise RuntimeError(f'Leader spring current cap exceeds motor limit {limit} mA')
        # Snapshot only RAM settings we change, plus Operating Mode. Never alter
        # EEPROM calibration, Drive Mode, Current Limit, or other motor IDs.
        self.saved = {'mode': mode, 'ram': [(a, n, self.read(a, n)) for a, n in
                     ((98, 1), (80, 2), (82, 2), (84, 2), (88, 2), (90, 2),
                      (108, 4), (112, 4), (102, 2), (116, 4))]}
        try:
            self.write(98, 1, 0)
            if mode != 5:
                self.write(11, 1, 5)
            if self.read(11, 1) != 5:
                raise RuntimeError('Leader trigger mode-5 readback failed')
            if self.read(10, 1) & 4:
                raise RuntimeError('Leader time-based profile is unsupported; keep existing Drive Mode')
            lo, hi = self.read(52, 4), self.read(48, 4)
            if not lo <= self.cfg['open_tick'] <= hi:
                raise RuntimeError('Calibrated leader open tick is outside motor position limits')
            position = self.read(132, 4)
            if not lo <= position <= hi:
                raise RuntimeError('Leader current position is outside position limits')
            self.write(102, 2, 0)
            for address, value in ((80, 400), (82, 0), (84, 400), (88, 0), (90, 0),
                                   (108, 25), (112, 50)):
                self.write(address, 4 if address >= 108 else 2, value)
                if self.read(address, 4 if address >= 108 else 2) != value:
                    raise RuntimeError(f'Leader trigger setting readback failed at {address}')
            # Align before enabling, then request the calibrated open position.
            self.write(116, 4, position)
            self.write(98, 1, 15)  # 300 ms communication watchdog.
            self.write(64, 1, 1)
            if self.read(64, 1) != 1:
                raise RuntimeError('Leader trigger torque-enable readback failed')
            self.active = True
            self.write(116, 4, self.cfg['open_tick'])
        except Exception:
            self.close()
            raise

    def apply(self, position_tick, velocity_rad_s, now):
        if self.active:
            if not all(math.isfinite(v) for v in (position_tick, velocity_rad_s, now)):
                self.zero()
                raise RuntimeError('Non-finite leader sample')
            # Positive magnitude only: the motor firmware chooses the restoring
            # current direction from position error and closes its own PD loop.
            fraction = max(0.0, min(1.0, (now-self.started)/self.cfg['ramp_sec']))
            self.write(102, 2, round(self.cfg['max_current_ma']*fraction))

    def zero(self):
        if self.active:
            self.write(102, 2, 0)

    def close(self):
        if self.saved is None:
            return
        errors = []
        # Restore RAM after restoring the mode (mode switching resets gains).
        operations = [(98, 1, 0), (102, 2, 0), (64, 1, 0),
                      (11, 1, self.saved['mode'])]
        operations += [(a, n, v) for a, n, v in self.saved['ram'] if a != 98]
        operations += [(98, 1, dict((a, v) for a, _, v in self.saved['ram'])[98])]
        for address, size, value in operations:
            try:
                self.write(address, size, value)
            except Exception as exc:
                errors.append(str(exc))
        self.active = False
        self.saved = None
        if errors:
            raise RuntimeError('; '.join(errors))
