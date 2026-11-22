"""NumPy deployment of the frozen J146 270-input residual actor; no ROS."""

from collections import deque

import numpy as np

from .contract import DT, LOWER, MODEL_FINGERPRINT, READY

CONTRACT = "e8ec756f378b5cb0f76fca783c9401a08231ca93c0110ad2454750f0799cd511"
OBS_DIM = 270
READY = np.asarray(READY, np.float32)


def nominal_index(step):
  if step < 0:
    raise ValueError("Negative J146 clock")
  return step if step < 400 else 355 + (step - 400) % 45


def phase(t):
  t = np.float32(t)
  angle = np.float32(2 * np.pi) * max(np.float32(0), t - 1) / np.float32(0.9)
  return np.asarray([np.sin(angle), np.cos(angle), np.clip(t / 2, 0, 1)], np.float32)


class History:
  def __init__(self):
    self.buffers = [deque(maxlen=5) for _ in range(9)]

  def append(
    self, q, dq, gravity, gyro, residual, command, nominal, motion_phase, heading
  ):
    terms = (
      gyro,
      gravity,
      q - READY,
      dq,
      residual,
      command,
      nominal,
      motion_phase,
      heading,
    )
    for buf, term in zip(self.buffers, terms, strict=True):
      value = np.asarray(term, np.float32)
      if not buf:
        buf.extend(value.copy() for _ in range(5))
      else:
        buf.append(value.copy())
    obs = np.concatenate([np.concatenate(list(buf)) for buf in self.buffers])
    if obs.shape != (OBS_DIM,) or not np.isfinite(obs).all():
      raise ValueError("Invalid J146 observation")
    return obs


class Policy:
  label = "J146 PPO"
  log_prefix = "j146"

  def __init__(self, path):
    with np.load(path, allow_pickle=False) as b:
      for key, expected in (
        ("j146_contract", CONTRACT),
        ("model_fingerprint", MODEL_FINGERPRINT),
      ):
        if str(b[key]) != expected:
          raise ValueError(f"J146 deployment mismatch: {key}")
      if tuple(b["actor_joint_names"]) != LOWER or not np.isclose(b["policy_dt"], DT):
        raise ValueError("J146 joint order or period mismatch")
      np.testing.assert_allclose(b["ready"], READY, atol=1e-7)
      self.budget = float(b["residual_budget"])
      self.progress = float(b["dr_strength"])
      self.nominal = b["nominal"].copy()
      self.negative = b["negative_scale"].reshape(10).copy()
      self.positive = b["positive_scale"].reshape(10).copy()
      self.mean = b["obs_mean"].reshape(OBS_DIM).copy()
      self.std = b["obs_std"].reshape(OBS_DIM).copy()
      self.weights = [b[f"weight_{i}"].copy() for i in range(int(b["layer_count"]))]
      self.biases = [b[f"bias_{i}"].copy() for i in range(len(self.weights))]
    self.width, self.stride, self.speed = 0.0, 1.0, 0.03
    if not 0 < self.budget <= 0.2 or not 0 <= self.progress <= 1:
      raise ValueError("Invalid J146 correction budget or DR metadata")
    if self.nominal.shape != (400, 10) or np.max(np.abs(self.nominal)) > 1.00001:
      raise ValueError("Invalid frozen J146 nominal table")
    if not self.weights or self.weights[0].shape[1] != OBS_DIM:
      raise ValueError("J146 requires a 270-input actor")
    previous = OBS_DIM
    for w, bias in zip(self.weights, self.biases, strict=True):
      if w.ndim != 2 or w.shape[1] != previous or bias.shape != (w.shape[0],):
        raise ValueError("Invalid J146 actor layer shapes")
      previous = w.shape[0]
    if previous != 10:
      raise ValueError("J146 requires 10 outputs")
    values = [
      self.nominal,
      self.negative,
      self.positive,
      self.mean,
      self.std,
      *self.weights,
      *self.biases,
    ]
    if not all(np.isfinite(x).all() for x in values) or np.any(self.std < 0):
      raise ValueError("Non-finite J146 policy")
    if np.any(self.negative <= 0) or np.any(self.positive <= 0):
      raise ValueError("Invalid J146 action scales")

  def normalized(self, obs):
    return (obs - self.mean) / (self.std + 0.01)

  def __call__(self, obs):
    value = self.normalized(obs)
    for i, (w, bias) in enumerate(zip(self.weights, self.biases, strict=True)):
      value = value @ w.T + bias
      if i + 1 < len(self.weights):
        negative = value < 0
        value[negative] = np.expm1(value[negative])
    if not np.isfinite(value).all():
      raise ValueError("Non-finite J146 output")
    return value.astype(np.float32)

  def target(self, nominal, residual):
    combined = np.clip(nominal + self.budget * np.clip(residual, -1, 1), -1, 1)
    # Motor transport, gains and response are the real plant. Do not apply the
    # simulation's fitted motor response again on the hardware command stream.
    return READY + combined * np.where(combined < 0, self.negative, self.positive)
