"""Run from ai_generated: python3 -m examples.development.generation.demo 13."""
import argparse
from pathlib import Path
import json

from tools.cp_tools.generators import (Random, random_int_tuple, random_interval,
    random_ordered_tuple, random_array, random_array_with_sum, random_string,
    random_brackets, random_sizes)
from tools.cp_tools.structures import (random_grid, random_tree, random_graph,
    random_dag, weighted_edges, random_intervals, random_queries)
from tools.cp_tools.formatting import format_case, format_cases
from tools.cp_tools.generation import generate_text


def example(number, seed=42):
    rng = Random(seed)
    if number in {1, 2}:
        values = random_int_tuple(rng, [(1, 100)] * (2 if number == 1 else 3))
        return format_case({"header": values}, [{"line": ["header"]}])
    if number == 3:
        l, r = random_interval(rng, 1, 100)
        return format_case({"N": 100, "L": l, "R": r}, [{"line": ["N", "L", "R"]}])
    if number == 4:
        return format_case({"v": random_ordered_tuple(rng, 3, 1, 1000)}, [{"line": ["v"]}])
    if number in {5, 6, 7}:
        a = random_array_with_sum(rng, 10, 0, 100, 300) if number == 7 else random_array(rng, 10, 0, 100, unique=number == 6)
        return format_case({"N": 10, "A": a}, [{"line": ["N"]}, {"line": ["A"]}])
    if number in {8, 9, 10}:
        value = random_brackets(rng, 10) if number == 10 else random_string(rng, 20, "abc", mode="palindrome" if number == 9 else "uniform")
        return format_case({"N": len(value), "s": value}, [{"line": ["N"]}, {"line": ["s"]}])
    if number == 11:
        return format_case({"h": 5, "w": 8, "grid": random_grid(rng, 5, 8, walls=10, connectivity="reachable")},
                           [{"line": ["h", "w"]}, {"rows": "grid"}])
    if number in {12, 13, 14, 15}:
        graph = random_tree(rng, 10) if number == 12 else random_dag(rng, 10, 15) if number == 14 else random_graph(rng, 10, 15, connected=True)
        edges = weighted_edges(rng, graph, -10, 100) if number == 15 else graph
        return format_case({"n": graph.n, "m": len(graph.edges), "g": edges},
                           [{"line": ["n", "m"]}, {"edges": "g", "one_based": True}])
    if number in {16, 17}:
        queries = random_intervals(rng, 10, 0, 20, half_open=True) if number == 16 else random_queries(rng, 10, 20, kind="set")
        return format_case({"n": 20, "q": 10, "queries": queries}, [{"line": ["n", "q"]}, {"rows": "queries"}])
    if number in {18, 19}:
        sizes = [5] * 3 if number == 18 else random_sizes(rng, 3, 1, 10, sum_max=20)
        cases = [{"n": n, "a": random_array(rng, n, 0, 100)} for n in sizes]
        return format_cases(cases, [{"line": ["n"]}, {"line": ["a"]}])
    if number == 20:
        return generate_text(json.loads(Path(__file__).with_name("arrays.json").read_text()), seed=seed)
    raise ValueError("choose example 1..20; stress/catalog commands are in README.md")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("number", type=int, choices=range(1, 21))
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()
    print(example(args.number, args.seed), end="")
