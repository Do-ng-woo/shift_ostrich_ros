#!/usr/bin/env python3
"""Shift Ostrich leader reader entry point."""
from pathlib import Path
import sys

source_root = Path(__file__).resolve().parents[1]
if (source_root / 'core').is_dir():
    sys.path.insert(0, str(source_root))
from core.leader_reader import main

if __name__ == '__main__':
    main()
