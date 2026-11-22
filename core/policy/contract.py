"""Deployed joint order, READY pose, timing and robot fingerprint."""

import math

LOWER = tuple(
  f"{side}_{joint}"
  for side in ("L", "R")
  for joint in ("hip_yaw", "hip_roll", "hip_pitch", "knee_pitch", "ankle_pitch")
)
ARMS = ("Arm_yaw", "Arm_shoulder", "Arm_elbow", "Arm_waist")
READY = (0.0, 0.0, -math.pi / 9, 2 * math.pi / 9, -math.pi / 9) * 2
DT = 0.02
MODEL_FINGERPRINT = "b1c011bcf577af9f0345bca6526be5bdcd7096b0712aca054fe26d7643b6c0d6"
