#!/usr/bin/env python3
"""Gazebo or hardware: execute the saved 20-second rearward-support bike-to-standing motion."""
from pathlib import Path
import sys

source_root = Path(__file__).resolve().parents[1]
if (source_root / 'core').is_dir():
    sys.path.insert(0, str(source_root))
from core.motion_runner import main

if __name__ == '__main__':
    main('revert')
