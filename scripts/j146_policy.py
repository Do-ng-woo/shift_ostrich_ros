#!/usr/bin/env python3
"""Run the current J146 policy through the ROS hardware bridge."""

import sys
from pathlib import Path

folder = Path(__file__).resolve().parent
root = folder.parent if (folder.parent / "core").is_dir() else folder
sys.path.insert(0, str(root))
from core.policy.node import main  # noqa: E402

if __name__ == "__main__":
  main()
