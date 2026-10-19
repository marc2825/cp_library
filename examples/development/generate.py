"""Run from repository root: python3 examples/development/generate.py 42."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from tools.cp_tools.random_data import Random, forest, undirected_graph

rng = Random(int(sys.argv[1]))
n = rng.integer(1, 20)
graph = (forest(rng, n, rng.integer(1, n)) if rng.below(2) else
         undirected_graph(rng, n, rng.integer(0, 40), multi=True, loops=True))
print(graph.text(), end="")
