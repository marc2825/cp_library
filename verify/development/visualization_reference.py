"""JSON bridge for the development visualization's deterministic generators."""
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from tools.cp_tools.random_data import Random, tree, forest, dag, simple_polygon

results = []
for case in json.load(sys.stdin):
    rng, n = Random(int(case["seed"])), case["n"]
    kind = case["kind"]
    if kind == "polygon":
        results.append({"points": simple_polygon(rng, n, case["bound"])})
    else:
        graph = (tree(rng, n) if kind == "tree" else forest(rng, n, case["components"])
                 if kind == "forest" else dag(rng, n, case["m"]))
        results.append({"n": graph.n, "edges": graph.edges, "directed": graph.directed})
print(json.dumps(results))
