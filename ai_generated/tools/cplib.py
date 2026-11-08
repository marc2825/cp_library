#!/usr/bin/env python3
"""Run the contest helper from any working directory, without installation."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tools.cp_tools.contest import main


if __name__ == "__main__":
    raise SystemExit(main())
