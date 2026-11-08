from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from tools.cp_tools.random_data import Random

rng = Random(int(sys.argv[1]))
print(8)
print(*(rng.integer(0, 1000) for _ in range(8)))
