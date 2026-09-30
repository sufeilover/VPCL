"""VS Code Run entry point for the current 60+90-frame game ZOH validation."""

from __future__ import annotations

import runpy
import sys
from pathlib import Path


target = Path(__file__).with_name("exp1_ai_validation.py")
sys.argv = [str(target), "--baseline-mode", "zoh", *sys.argv[1:]]
runpy.run_path(str(target), run_name="__main__")
