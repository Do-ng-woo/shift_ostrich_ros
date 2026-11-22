"""Export J146 checkpoints with their exact nominal table and correction budget."""

import argparse
import sys
from pathlib import Path

import numpy as np
import torch

# Permit direct execution under the separate MJLab uv environment.
if not __package__:
  sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
  __package__ = "core.policy"

from mjlab.tasks.shift_ostrich.free_walking.j146_rl.action import (
  CONTRACT as SOURCE_CONTRACT,
)
from mjlab.tasks.shift_ostrich.free_walking.j146_rl.action import REFERENCE, ROOT

from .contract import DT, LOWER, MODEL_FINGERPRINT, READY
from .policy import CONTRACT, OBS_DIM, Policy


def export(checkpoint, output):
  loaded = torch.load(checkpoint, map_location="cpu", weights_only=False)
  info = loaded.get("infos") or {}
  if SOURCE_CONTRACT != CONTRACT or info.get("j146_contract") != CONTRACT:
    raise ValueError("Not the frozen J146 deployment contract")
  state = loaded["actor_state_dict"]
  keys = sorted(
    (k for k in state if k.startswith("mlp.") and k.endswith(".weight")),
    key=lambda k: int(k.split(".")[1]),
  )
  weights = [state[k].numpy().astype(np.float32) for k in keys]
  biases = [state[k.replace("weight", "bias")].numpy().astype(np.float32) for k in keys]
  with np.load(ROOT / "trajectory.npz", allow_pickle=False) as archive:
    nominal = archive["action"][:, 0].copy()
  payload = dict(
    j146_contract=np.asarray(CONTRACT),
    model_fingerprint=np.asarray(MODEL_FINGERPRINT),
    actor_joint_names=np.asarray(LOWER),
    policy_dt=np.asarray(DT),
    ready=np.asarray(READY, np.float32),
    nominal=nominal,
    negative_scale=np.asarray(REFERENCE["negative_scale"], np.float32),
    positive_scale=np.asarray(REFERENCE["positive_scale"], np.float32),
    residual_budget=np.asarray(info["residual_budget"]),
    dr_strength=np.asarray(info["dr_strength"]),
    obs_mean=state["obs_normalizer._mean"].numpy(),
    obs_std=state["obs_normalizer._std"].numpy(),
    layer_count=np.asarray(len(weights)),
  )
  for i, (w, b) in enumerate(zip(weights, biases, strict=True)):
    payload[f"weight_{i}"], payload[f"bias_{i}"] = w, b
  output.parent.mkdir(parents=True, exist_ok=True)
  temporary = output.with_name("." + output.name + ".tmp.npz")
  np.savez(temporary, **payload)
  try:
    policy = Policy(temporary)
    rng = np.random.default_rng(146)
    obs = policy.mean + policy.std * rng.normal(size=(256, OBS_DIM)).astype(np.float32)
    value = (torch.from_numpy(obs) - state["obs_normalizer._mean"]) / (
      state["obs_normalizer._std"] + 0.01
    )
    for i, (w, b) in enumerate(zip(weights, biases, strict=True)):
      value = torch.nn.functional.linear(
        value, torch.from_numpy(w), torch.from_numpy(b)
      )
      if i + 1 < len(weights):
        value = torch.nn.functional.elu(value)
    error = float(np.max(np.abs(np.stack([policy(x) for x in obs]) - value.numpy())))
    if not np.isfinite(error) or error > 5e-4:
      raise ValueError(f"J146 actor parity failed: {error}")
    temporary.replace(output)
  finally:
    temporary.unlink(missing_ok=True)
  print(
    f"J146 export: 270 -> 10; budget={policy.budget}; DR={policy.progress}; "
    f"max action error={error:.3g}; {output}"
  )


if __name__ == "__main__":
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--checkpoint", type=Path, required=True)
  parser.add_argument("--output", type=Path, required=True)
  args = parser.parse_args()
  export(args.checkpoint, args.output)
