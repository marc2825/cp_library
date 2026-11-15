#!/usr/bin/env python3
"""Reproducible wall-time and peak traced-memory measurements; no thresholds."""
import argparse
import gc
import json
from pathlib import Path
import platform
import sys
import time
import tracemalloc

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from tools.cp_tools.generators import Random, random_array, random_string, random_sizes
from tools.cp_tools.structures import random_graph, random_queries
from tools.cp_tools.generation import generate_text


def measure(name, function):
    gc.collect()
    start = time.perf_counter()
    value = function()
    seconds = time.perf_counter() - start
    del value
    gc.collect()
    tracemalloc.start()
    value = function()
    _, peak = tracemalloc.get_traced_memory()
    tracemalloc.stop()
    del value
    return {"name":name, "seconds":round(seconds,6), "peak_traced_mib":round(peak/2**20,3)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path)
    args = parser.parse_args()
    spec = {"seed":42,"cases":{"count":10000,"aggregate":{"N":{"sum_max":200000}}},
            "variables":[{"name":"N","type":"int","min":1,"max":100},
                         {"name":"A","type":"int_array","length_ref":"N","min":0,"max":1000}],
            "output":[{"line":["N"]},{"line":["A"]}]}
    jobs = [
        ("array_200000", lambda: random_array(Random(42),200000,-10**9,10**9)),
        ("string_200000", lambda: random_string(Random(42),200000)),
        ("sparse_graph_200000_vertices_200000_edges", lambda: random_graph(Random(42),200000,200000,connected=True)),
        ("set_queries_200000", lambda: random_queries(Random(42),200000,200000,kind="set")),
        ("sizes_10000", lambda: random_sizes(Random(42),10000,1,100,sum_max=200000)),
        ("json_10000_cases_with_formatting", lambda: generate_text(spec)),
    ]
    rows = []
    for name, function in jobs:
        row = measure(name, function)
        rows.append(row)
        print(json.dumps(row), flush=True)
    report = {"python":platform.python_version(),"platform":platform.platform(),"seed":42,
              "method":"One untraced wall-time run and one separate tracemalloc peak run per workload; no file I/O",
              "results":rows}
    if args.out:
        args.out.parent.mkdir(parents=True,exist_ok=True)
        args.out.write_text(json.dumps(report,ensure_ascii=False,indent=2)+"\n")


if __name__ == "__main__":
    main()
