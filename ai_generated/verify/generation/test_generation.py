"""Independent invariant, feasibility, reproducibility and CLI integration tests."""
from collections import Counter, deque
import copy
import hashlib
from decimal import Decimal
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

from tools.cp_tools import generators as g, structures as s
from tools.cp_tools import generation as config
from tools.cp_tools import feature_catalog as catalog
from tools.cp_tools import stress
from tools.cp_tools.formatting import format_case, format_cases
from tools.cp_tools.random_data import Random, Graph
from tools.cp_tools.repository import ROOT
from examples.development.generation.demo import example


def component_count(graph):
    adj = [[] for _ in range(graph.n)]
    for u, v in graph.edges:
        if not 0 <= u < graph.n or not 0 <= v < graph.n:
            raise AssertionError("endpoint outside graph")
        adj[u].append(v)
        adj[v].append(u)
    seen, count = set(), 0
    for start in range(graph.n):
        if start in seen:
            continue
        count += 1
        seen.add(start)
        stack = [start]
        while stack:
            for v in adj[stack.pop()]:
                if v not in seen:
                    seen.add(v)
                    stack.append(v)
    return count


def grid_reached(rows):
    h, w = len(rows), len(rows[0])
    reached, pending = {(0, 0)}, [(0, 0)]
    for i, j in pending:
        for ni, nj in ((i - 1, j), (i + 1, j), (i, j - 1), (i, j + 1)):
            if 0 <= ni < h and 0 <= nj < w and rows[ni][nj] != "#" and (ni, nj) not in reached:
                reached.add((ni, nj))
                pending.append((ni, nj))
    return reached


class PrimitiveTests(unittest.TestCase):
    def test_big_integer_and_legacy_sequence(self):
        for seed in range(30):
            a, b = Random(seed), Random(seed)
            for n in (1, 2, 17, 2**64):
                self.assertEqual(a.integer(0, n - 1), b.below(n))
            for bits in (0, 1, 64, 65, 255, 512):
                value = a.bits(bits)
                self.assertTrue(0 <= value < 1 << bits)
            for _ in range(20):
                value = a.integer(-(10**100), 10**100)
                self.assertTrue(-10**100 <= value <= 10**100)
        with self.assertRaises(ValueError): Random(0).below(2**64 + 1)
        with self.assertRaises(ValueError): Random(0).randbelow(0)

    def test_integer_distributions(self):
        for mode in ("uniform", "minimum", "maximum", "boundary", "small", "large", "zero_heavy", "power2", "power10", ["minimum", "maximum"]):
            rng = Random(1)
            for _ in range(100):
                x = g.random_int(rng, -101, 101, mode=mode, multiple=3)
                self.assertTrue(-101 <= x <= 101 and x % 3 == 0)
        self.assertEqual(g.random_int(Random(1), -101, 101, mode="minimum", multiple=3), -99)
        self.assertEqual(g.random_int(Random(1), -101, 101, mode="maximum", multiple=3), 99)
        for low, high, multiple in ((1, 1, 2), (5, 4, 1), (0, 1, 0)):
            with self.assertRaises(ValueError): g.random_int(Random(0), low, high, multiple=multiple)

    def test_samples_without_population_materialization(self):
        for population in (0, 1, 10, 100, 10**100):
            count = min(population, 30)
            value = g.sample_indices(Random(4), population, count)
            self.assertEqual(len(set(value)), count)
            self.assertTrue(all(0 <= x < population for x in value))
        with self.assertRaises(ValueError): g.sample_indices(Random(0), 3, 4)

    def test_weighted_and_scalar_types(self):
        rng = Random(4)
        for _ in range(100):
            self.assertEqual(rng.weighted_choice("abc", [0, 5, 0]), "b")
            self.assertEqual(g.random_bool(rng, 1, 0), True)
            self.assertIn(g.random_char(rng, "aあ"), "aあ")
            self.assertTrue(-10 <= g.random_float(rng, -10, 30) <= 30)
            x = g.random_fixed(rng, "-1.234", "2.345", 2)
            self.assertTrue(Decimal("-1.234") <= x <= Decimal("2.345"))
            self.assertEqual(x.as_tuple().exponent, -2)
        self.assertEqual(g.random_fixed(rng, 10**80, 10**80, 3), Decimal(str(10**80) + ".000"))
        for weights in ([], [0, 0, 0], [-1, 1, 1], [1.2, 1, 1]):
            with self.assertRaises(ValueError): rng.weighted_choice("abc", weights)

    def test_tuples_and_intervals(self):
        rng = Random(1)
        for _ in range(100):
            a = g.random_int_tuple(rng, [(0, 100)] * 3, distinct=True)
            self.assertEqual(len(set(a)), 3)
            self.assertEqual(len(g.random_int_tuple(rng, [(0, 10)] * 2, predicate=lambda x: sum(x) <= 7)), 2)
            for strict in (False, True):
                a = g.random_ordered_tuple(rng, 3, -100, 100, strict=strict)
                self.assertTrue(all(x < y if strict else x <= y for x, y in zip(a, a[1:])))
            l, r = g.random_interval(rng, 1, 100)
            self.assertTrue(1 <= l <= r <= 100)
            l, r = g.random_interval(rng, 0, 100, half_open=True, length=7)
            self.assertEqual(r - l, 7)
        rows = g.random_records(rng, 10, [lambda r: r.integer(0, 2), lambda r: g.random_string(r, 3)])
        self.assertTrue(all(len(row) == 2 and len(row[1]) == 3 for row in rows))
        for call in (lambda: g.random_ordered_tuple(rng, 2, 1, 1), lambda: g.random_interval(rng, 1, 1, half_open=True),
                     lambda: g.random_int_tuple(rng, [(1, 1)] * 2, distinct=True),
                     lambda: g.random_int_tuple(rng, [(1, 1)], predicate=lambda x: False, attempts=2)):
            with self.assertRaises(ValueError): call()

    def test_arrays(self):
        for seed in range(30):
            rng = Random(seed)
            for mode in ("uniform", "sorted", "reversed", "almost_sorted", "all_equal", "few_distinct", "alternating", "zero_heavy", ["minimum", "maximum"]):
                a = g.random_array(rng, 100, -20, 20, mode=mode)
                self.assertEqual(len(a), 100)
                self.assertTrue(all(-20 <= x <= 20 for x in a))
                if mode == "sorted": self.assertEqual(a, sorted(a))
                if mode == "reversed": self.assertEqual(a, sorted(a, reverse=True))
                if mode == "all_equal": self.assertEqual(len(set(a)), 1)
                if mode == "alternating": self.assertTrue(all(a[i] == a[i % 2] for i in range(100)))
            for k in (1, 3, 10):
                self.assertEqual(len(set(g.random_array(rng, 20, 0, 20, distinct_count=k))), k)
            a = g.random_array(rng, 100, -(10**100), 10**100, unique=True)
            self.assertEqual(len(set(a)), 100)
        for opts in ({"unique": True}, {"distinct_count": 51}, {"mode": "bogus"}):
            with self.assertRaises(ValueError): g.random_array(Random(0), 100, 1, 50, **opts)
        with self.assertRaises(ValueError): g.random_array(Random(0), 3, 1, 50, distinct_count=3, mode="minimum")

    def test_sums_and_sizes(self):
        for seed in range(30):
            rng = Random(seed)
            for n in (0, 1, 20):
                for total in (n * -3, 0, n * 7):
                    a = g.random_array_with_sum(rng, n, -3, 7, total)
                    self.assertEqual(sum(a), total)
                    self.assertTrue(all(-3 <= x <= 7 for x in a))
            a = g.random_array_with_sum(rng, 100, -10, 10, sum_min=-50, sum_max=100)
            self.assertTrue(-50 <= sum(a) <= 100)
            sizes = g.random_sizes(rng, 100, 1, 100, sum_max=120, cost_max=200)
            self.assertTrue(sum(sizes) <= 120 and sum(x*x for x in sizes) <= 200)
        for call in (lambda: g.random_array_with_sum(Random(0), 2, 1, 3, 7),
                     lambda: g.random_array_with_sum(Random(0), 0, 0, 2, 1),
                     lambda: g.random_sizes(Random(0), 10, 2, 10, sum_max=19),
                     lambda: g.random_sizes(Random(0), 10, 2, 10, cost_max=39)):
            with self.assertRaises(ValueError): call()

    def test_strings(self):
        for seed in range(30):
            rng = Random(seed)
            for n in (0, 1, 2, 99, 100):
                for mode in ("uniform", "palindrome", "periodic", "no_adjacent", "all_equal", "alternating"):
                    value = g.random_string(rng, n, "aあc", mode=mode)
                    self.assertEqual(len(value), n)
                    self.assertTrue(set(value) <= set("aあc"))
                    if mode == "palindrome": self.assertEqual(value, value[::-1])
                    if mode == "no_adjacent": self.assertTrue(all(x != y for x, y in zip(value, value[1:])))
                if n > 1:
                    value = g.random_string(rng, n, "abc", mode="non_palindrome")
                    self.assertNotEqual(value, value[::-1])
            self.assertEqual(Counter(g.random_string(rng, 10, "abc", counts={"a": 3, "b": 7})), {"a": 3, "b": 7})
            a, b = g.random_anagram_pair(rng, 100, "abc")
            self.assertEqual(Counter(a), Counter(b))
            self.assertEqual(len(set(g.random_string(rng, 3, "abc", unique=True))), 3)
            self.assertEqual(len(set(g.random_string(rng, 10, "abc", distinct_count=2))), 2)
        for opts in ({"unique": True}, {"mode": "no_adjacent"}, {"mode": "non_palindrome"}):
            with self.assertRaises(ValueError): g.random_string(Random(0), 10, "a", **opts)

    def test_brackets(self):
        for seed in range(50):
            for mode in ("random", "nested", "alternating"):
                value = g.random_brackets(Random(seed), seed, mode=mode)
                balance = 0
                for c in value:
                    balance += 1 if c == "(" else -1
                    self.assertGreaterEqual(balance, 0)
                self.assertEqual((len(value), balance), (2 * seed, 0))


class StructureTests(unittest.TestCase):
    def test_matrix(self):
        rng = Random(0)
        for mode in ("symmetric", "identity", "diagonal", "upper", "lower", "permutation"):
            a = s.random_matrix(rng, 10, 10, 0, 1, mode=mode)
            self.assertEqual([len(row) for row in a], [10] * 10)
            for i in range(10):
                for j in range(10):
                    if mode == "symmetric": self.assertEqual(a[i][j], a[j][i])
                    if mode == "diagonal" and i != j: self.assertEqual(a[i][j], 0)
                    if mode == "upper" and i > j: self.assertEqual(a[i][j], 0)
                    if mode == "lower" and i < j: self.assertEqual(a[i][j], 0)
                    if mode == "identity": self.assertEqual(a[i][j], int(i == j))
            if mode == "permutation":
                self.assertEqual(list(map(sum, a)), [1] * 10)
                self.assertEqual(list(map(sum, zip(*a))), [1] * 10)
        self.assertEqual(sum(map(sum, s.random_matrix(rng, 5, 8, ones=17))), 17)
        self.assertEqual(list(map(sum, s.random_matrix(rng, 3, 5, -5, 5, row_sums=[-25, 0, 25]))), [-25, 0, 25])
        with self.assertRaises(ValueError): s.random_matrix(rng, 3, 4, mode="symmetric")
        with self.assertRaises(ValueError): s.random_matrix(rng, 3, 4, ones=13)

    def test_grid_reachability_and_unique_path(self):
        for seed in range(50):
            for h, w in ((1, 8), (8, 1), (5, 8)):
                for mode in ("reachable", "unreachable", "unique_path"):
                    walls = h*w - (h+w-1) if mode == "unique_path" else 1 if h == 1 or w == 1 else 5
                    if mode == "reachable" and (h == 1 or w == 1): walls = 0
                    grid = s.random_grid(Random(seed), h, w, walls=walls, connectivity=mode)
                    self.assertEqual(sum(row.count("#") for row in grid), walls)
                    self.assertEqual(grid[0][0], "S")
                    self.assertEqual(grid[-1][-1], "G")
                    reached = grid_reached(grid)
                    self.assertEqual((h-1, w-1) in reached, mode != "unreachable")
                    if mode == "unique_path":
                        edges = sum((i+1,j) in reached for i,j in reached) + sum((i,j+1) in reached for i,j in reached)
                        self.assertEqual(edges, len(reached)-1)
        for args in ((1, 1, 0, "reachable"), (1, 2, 0, "unreachable"), (2, 2, 2, "reachable")):
            h, w, walls, mode = args
            with self.assertRaises(ValueError): s.random_grid(Random(0), h, w, walls=walls, connectivity=mode)

    def test_trees(self):
        for seed in range(40):
            for mode in ("random", "uniform", "path", "star", "binary"):
                graph = s.random_tree(Random(seed), 1 + seed, mode=mode)
                self.assertEqual(len(graph.edges), graph.n - 1)
                self.assertEqual(component_count(graph), 1)
                parents = s.parent_array(graph)
                self.assertEqual(parents[0], -1)
                self.assertTrue(all(0 <= p < graph.n for p in parents[1:]))
                self.assertEqual(sum(map(len, s.adjacency_list(graph))), 2 * len(graph.edges))
        with self.assertRaises(ValueError): s.parent_array(Graph(3, ((0, 1), (1, 0))))

    def test_graphs_exhaustive_small_feasibility(self):
        for n in range(7):
            for directed in (False, True):
                for loops in (False, True):
                    capacity = n*(n-1) if directed else n*(n-1)//2
                    capacity += n if loops else 0
                    for m in range(capacity + 1):
                        graph = s.random_graph(Random(m+n), n, m, directed=directed, loops=loops)
                        self.assertEqual(len(graph.edges), m)
                        edges = graph.edges if directed else [tuple(sorted(e)) for e in graph.edges]
                        self.assertEqual(len(set(edges)), m)
                        self.assertTrue(all(0 <= u < n and 0 <= v < n and (loops or u != v) for u, v in edges))
                    with self.assertRaises(ValueError): s.random_graph(Random(0), n, capacity+1, directed=directed, loops=loops)
            for c in range(1, n+1):
                for m in range(n-c, (n-c+1)*(n-c)//2+1):
                    graph = s.random_graph(Random(m), n, m, components=c)
                    self.assertEqual(component_count(graph), c)
            if n:
                for directed in (False, True):
                    graph = s.random_graph(Random(n), n, n-1, directed=directed, connected=True)
                    self.assertEqual(component_count(graph), 1)
                    graph = s.random_graph(Random(n), n, 20, directed=directed, loops=True, multi=True)
                    self.assertEqual(len(graph.edges), 20)

    def test_dag_and_bipartite(self):
        for seed in range(50):
            n = seed % 20
            graph = s.random_dag(Random(seed), n, n*(n-1)//2)
            indegrees, adj = [0]*n, [[] for _ in range(n)]
            for u, v in graph.edges:
                adj[u].append(v)
                indegrees[v] += 1
            queue = [v for v in range(n) if not indegrees[v]]
            for u in queue:
                for v in adj[u]:
                    indegrees[v] -= 1
                    if not indegrees[v]: queue.append(v)
            self.assertEqual(len(queue), n)
            b = s.random_bipartite_graph(Random(seed), 7, 11, 50)
            self.assertEqual(len(set(b.edges)), 50)
            self.assertTrue(all(0 <= u < 7 <= v < 18 for u,v in b.edges))
            self.assertTrue(all(-10 <= w <= 20 for _, _, w in s.weighted_edges(Random(seed), b, -10, 20)))

    def test_stateful_queries_independent_replay(self):
        for kind in ("set", "multiset", "stack", "queue", "edges"):
            for seed in range(20):
                rows = s.random_queries(Random(seed), 500, 8, kind=kind)
                state = []
                for row in rows:
                    operation, *value = row
                    value = tuple(value)
                    if operation in {"push", "insert"}:
                        if kind in {"set", "edges"}: self.assertNotIn(value, state)
                        state.append(value)
                    else:
                        self.assertTrue(state)
                        if operation == "erase":
                            self.assertIn(value, state)
                            state.remove(value)
                        else:
                            state.pop(0 if kind == "queue" else -1)
        state = []
        rows = s.stateful_queries(Random(0), 10, state, lambda st: ["add"],
                                 lambda st, op, rng: (st.append(1) or (op, len(st))))
        self.assertEqual(rows[-1], ("add", 10))

    def test_stateless_queries_and_intervals(self):
        for kind in ("point", "range", "union_find"):
            rows = s.random_queries(Random(1), 500, 20, kind=kind)
            for row in rows:
                if kind == "range": self.assertTrue(0 <= row[1] < row[2] <= 20)
                else: self.assertTrue(0 <= row[1] < 20)
        for l,r in s.random_intervals(Random(0), 100, 1, 100, length=3):
            self.assertEqual(r-l+1, 3)


class ConfigurationTests(unittest.TestCase):
    def setUp(self):
        self.config = json.loads((ROOT / "examples/development/generation/arrays.json").read_text())

    def cli(self, *args):
        return subprocess.run([sys.executable, str(ROOT/"tools/cplib.py"), *map(str, args)], cwd="/tmp", capture_output=True, text=True)

    def test_formatting(self):
        values = {"N": 3, "M": 2, "A": [4,5,6], "G": Graph(3, ((0,1),(1,2))), "rows": [(1,"abc"),(False,3.5)]}
        self.assertEqual(format_case(values, [{"line":["N","M"]}, {"line":["A"]}, {"edges":"G","one_based":True}]), "3 2\n4 5 6\n1 2\n2 3\n")
        self.assertEqual(format_case(values, [{"lines":"A"}, {"rows":"rows"}, {"literal":"0 0"}]), "4\n5\n6\n1 abc\n0 3.5\n0 0\n")
        self.assertEqual(format_cases([values,values], [{"line":["N"]}]), "2\n3\n3\n")
        with self.assertRaises(ValueError): format_case({"s":"a b"}, [{"line":["s"]}])
        with self.assertRaises(ValueError): format_case({"a":[[1,2]]}, [{"line":["a"]}])

    def test_dependencies_and_reproduction(self):
        a = config.generate_cases(self.config)
        self.assertEqual(a, config.generate_cases(self.config))
        self.assertLessEqual(sum(c["N"] for c in a), 20)
        for i, case in enumerate(a):
            self.assertTrue(0 <= case["M"] <= case["N"])
            self.assertEqual(len(case["A"]), case["N"])
            self.assertEqual(config.generate_cases(self.config, case_index=i), [case])
        alternative = copy.deepcopy(self.config)
        alternative["output"] = [{"line":["N"]}]
        self.assertEqual(config.generate_cases(alternative), a)
        self.assertEqual(self.cli("gen", "--config", ROOT/"examples/development/generation/arrays.json").stdout, config.generate_text(self.config))

    def test_expressions_and_constraints(self):
        self.assertTrue(config.expression("1 <= L <= R <= N and gcd(L, R) == 1", {"L":1,"R":4,"N":5}))
        self.assertEqual(config.expression("N*(N-1)//2", {"N":10}), 45)
        self.assertEqual(config.expression("sum(A) + len(A)", {"A":[1,2]}), 5)
        for bad in ("__import__('os')", "N.real", "N[0]", "missing+1", "2 ** 100", "[x for x in N]"):
            with self.assertRaises(ValueError): config.expression(bad, {"N":3})
        spec = {"variables":[{"name":"A","type":"int","min":0,"max":5}, {"name":"B","type":"int","min":0,"max":5}],
                "constraints":["A != B", "A + B <= 5"], "output":[{"line":["A","B"]}]}
        for seed in range(20):
            values = config.generate_cases(spec, seed=seed)[0]
            self.assertTrue(values["A"] != values["B"] and values["A"]+values["B"] <= 5)
        spec["constraints"] = ["A < 0"]
        spec["attempts"] = 3
        with self.assertRaisesRegex(ValueError, "attempt budget"): config.generate_cases(spec)

    def test_schema_errors_and_impossible_constraints(self):
        bad_specs = [
            {"variables":[{"name":"N","type":"int","min":3,"max":1}],"output":[]},
            {"variables":[{"name":"M","type":"int","max_ref":"N"}],"output":[]},
            {"variables":[{"name":"N","type":"int","mni":1}],"output":[]},
            {"variables":[{"name":"A","type":"int_array","length":100,"min":1,"max":50,"unique":True}],"output":[]},
            {"variables":[],"output":[{"line":["missing"]}]},
            {"variables":[],"output":[],"cases":{"count":-1}},
        ]
        for spec in bad_specs:
            with self.subTest(spec=spec):
                with self.assertRaises(ValueError): config.generate_cases(spec)
        self.config["cases"]["aggregate"]["N"]["sum_max"] = 2
        with self.assertRaisesRegex(ValueError,"aggregate"): config.generate_cases(self.config)

    def test_json_types(self):
        examples = [
            ("tuple", {"ranges":[[1,10],[20,30]]}), ("ordered_tuple", {"length":3,"min":0,"max":10}),
            ("interval", {"min":1,"max":10}), ("intervals", {"count":3,"min":1,"max":10}),
            ("string", {"length":10,"mode":"palindrome"}), ("brackets", {"pairs":10}),
            ("matrix", {"height":3,"width":3,"min":0,"max":1,"ones":4}),
            ("grid", {"height":5,"width":8,"walls":5,"connectivity":"reachable"}),
            ("tree", {"n":10}), ("forest", {"n":10,"components":3}),
            ("graph", {"n":10,"m":20,"weight_min":-5,"weight_max":10}), ("dag", {"n":10,"m":10}),
            ("bipartite", {"left":4,"right":5,"m":10}), ("queries", {"length":10,"n":5,"kind":"set"}),
            ("polygon", {"n":5}), ("choice", {"values":[1,2],"weights":[0,1]}),
            ("float", {}), ("fixed", {}), ("bool", {}), ("char", {}), ("const", {"value":7}),
        ]
        for kind, options in examples:
            spec = {"variables":[{"name":"X","type":kind,**options}],"output":[]}
            with self.subTest(kind=kind):
                self.assertEqual(config.generate_cases(spec), config.generate_cases(spec))
        literal = {"variables":[{"name":"X","type":"const","value":"=literal"}],"output":[{"line":["X"]}]}
        self.assertEqual(config.generate_text(literal), "1\n=literal\n")

    def test_count_range_and_square_budgets(self):
        self.config["cases"]["count"] = 100
        self.assertEqual(len(config.generate_cases(self.config, count=3)),3)
        self.config["cases"]["count"] = {"min":1,"max":5}
        self.config["cases"]["aggregate"]["N"] = {"sum_max":20,"cost_max":50}
        for seed in range(30):
            values = config.generate_cases(self.config, seed=seed)
            self.assertTrue(1 <= len(values) <= 5)
            self.assertLessEqual(sum(c["N"]**2 for c in values), 50)
            self.assertEqual(config.generate_cases(self.config, seed=seed, case_index=0), values[:1])

    def test_cli_files_errors_and_no_data_on_failure(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            spec = root/"config.json"
            spec.write_text(json.dumps(self.config))
            result = self.cli("gen", "--config", spec, "--validate")
            self.assertEqual((result.returncode,result.stdout), (0,""))
            result = self.cli("gen", "--config", spec, "--out-dir", root/"cases")
            self.assertEqual(result.returncode,0,result.stderr)
            for i,case in enumerate(config.generate_cases(self.config)):
                self.assertEqual((root/f"cases/case-{i:06d}.txt").read_text(), format_case(case,self.config["output"]))
            result = self.cli("gen", "--config", spec, "--out-dir", root/"cases")
            self.assertNotEqual(result.returncode,0)
            result = self.cli("gen", "--config", spec, "--case-index", 1)
            self.assertEqual(result.stdout, (root/"cases/case-000001.txt").read_text())
            result = self.cli("gen", "--config", spec, "--out", spec, "--force")
            self.assertNotEqual(result.returncode,0)
            spec.write_text('{"variables":null}')
            result = self.cli("gen", "--config", spec)
            self.assertNotEqual(result.returncode,0)
            self.assertEqual(result.stdout,"")
            self.assertNotIn("Traceback",result.stderr)

    def test_all_examples(self):
        for i in range(1,21):
            self.assertEqual(example(i),example(i))
            self.assertTrue(example(i).endswith("\n"))

    def test_stress_json_generator_failure_save_and_replay(self):
        spec = ROOT/"examples/development/generation/graph.json"
        oracle = [sys.executable,str(ROOT/"examples/development/naive.py")]
        settings = {"cwd":str(ROOT),"timeout":2,"compare":"tokens",
                    "generator":[sys.executable,str(ROOT/"tools/cplib.py"),"gen","--config",str(spec),"--seed","{seed}"],
                    "fast":oracle,"naive":oracle}
        status, data, results = stress.run_case(settings,72)
        self.assertEqual(status,"ok")
        self.assertEqual(data.decode(),config.generate_text(json.loads(spec.read_text()),seed=72))
        broken = {**settings,"fast":[sys.executable,"-c","print('wrong')"]}
        status,data,results = stress.run_case(broken,72)
        self.assertEqual(status,"mismatch")
        with tempfile.TemporaryDirectory() as folder:
            saved = stress.save_failure(Path(folder),broken,72,status,data,results)
            self.assertEqual((saved/"input.txt").read_bytes(),data)
            # Replay bypasses generation and compares the exact saved input.
            broken["generator"] = ["missing-generator"]
            self.assertEqual(stress.run_case(broken,72,(saved/"input.txt").read_bytes())[0],"mismatch")


class CatalogTests(unittest.TestCase):
    def fixture(self, repo):
        paths = {"README.md":"[index](ai_generated/FEATURES.md)\n",
                 "ai_generated/README.md":"[index](FEATURES.md)\n",
                 "ai_generated/docs/README.md":"[index](../FEATURES.md)\n",
                 "ai_generated/catalog/README.md":"[index](../FEATURES.md)\n",
                 "ai_generated/demo/all.hpp":"#pragma once\n",
                 "ai_generated/demo/a.hpp":"#pragma once\n",
                 "ai_generated/docs/demo/a.md":"---\ntitle: 例\ndocumentation_of: //demo/a.hpp\n---\n\n説明。\n",
                 "ai_generated/tools/cp_tools/generators.py":"__all__ = ['hello']\ndef hello():\n    return 1\n"}
        for relative,text in paths.items():
            path = repo/relative
            path.parent.mkdir(parents=True,exist_ok=True)
            path.write_text(text)
        return paths

    def save(self, repo):
        data = catalog.collect_features(repo)
        (repo/"ai_generated/catalog/features.json").write_text(json.dumps(data))
        (repo/"ai_generated/FEATURES.md").write_text(catalog.render_markdown(data))
        return data

    def test_stale_catalog_missing_sources_and_orphan_docs(self):
        with tempfile.TemporaryDirectory() as folder:
            repo = Path(folder)
            self.fixture(repo)
            self.save(repo)
            # The generated index links to standard documentation navigation.
            for relative in ("docs/development/generation.md","docs/development_records/README.md"):
                path = repo/"ai_generated"/relative
                path.parent.mkdir(parents=True,exist_ok=True)
                path.write_text("# Example\n")
            self.save(repo)
            catalog.check_catalog(repo)
            data = self.save(repo)
            data["features"].append(data["features"][0])
            (repo/"ai_generated/catalog/features.json").write_text(json.dumps(data))
            with self.assertRaises(ValueError): catalog.check_catalog(repo)
            self.save(repo)
            (repo/"ai_generated/demo/a.hpp").unlink()
            with self.assertRaises(ValueError): catalog.check_catalog(repo)
            self.save(repo)
            with self.assertRaisesRegex(ValueError,"orphan"): catalog.check_catalog(repo)

    def test_new_python_exports_and_stale_evidence(self):
        with tempfile.TemporaryDirectory() as folder:
            repo = Path(folder)
            self.fixture(repo)
            source = repo/"ai_generated/tools/cp_tools/generators.py"
            receipt = {"schema_version":1,"status":"passed","feature_ids":["generator.hello"],
                       "sources":{"ai_generated/tools/cp_tools/generators.py":hashlib.sha256(source.read_bytes()).hexdigest()}}
            (repo/"ai_generated/catalog/test_evidence.json").write_text(json.dumps(receipt))
            data = self.save(repo)
            self.assertEqual(next(f["verification"] for f in data["features"] if f["id"]=="generator.hello"),"partially_tested")
            source.write_text(source.read_text()+"\ndef extra():\n    pass\n")
            data = catalog.collect_features(repo)
            self.assertEqual(data["evidence"],"stale")
            self.assertFalse(any(f["id"]=="generator.extra" for f in data["features"]))
            source.write_text(source.read_text().replace("['hello']","['hello', 'extra']"))
            self.assertTrue(any(f["id"]=="generator.extra" for f in catalog.collect_features(repo)["features"]))

    def test_inventory_ids_paths_and_determinism(self):
        data = catalog.collect_features()
        self.assertEqual(data, catalog.collect_features())
        ids = [f["id"] for f in data["features"]]
        self.assertEqual(len(ids),len(set(ids)))
        sources = {f["source"] for f in data["features"]}
        for feature in data["features"]:
            self.assertTrue((ROOT.parent/feature["source"]).is_file())
            if feature["documentation"]: self.assertTrue((ROOT.parent/feature["documentation"]).is_file())
        from tools.cp_tools.contest import public_headers
        self.assertTrue({"ai_generated/"+h for h in public_headers()} <= sources)
        self.assertTrue({p.relative_to(ROOT.parent).as_posix() for p in (ROOT.parent/"depricated").rglob("*.hpp")} <= sources)
        self.assertEqual(catalog.render_markdown(data),catalog.render_markdown(catalog.collect_features()))

    def test_search_aliases_filters_and_cli(self):
        data = catalog.collect_features()
        for query in ("DSU", "Union-Find", "素集合", "Fenwick", "BIT", "セグ木", "shortest-path", "最短路", "random", "文字列"):
            self.assertTrue(catalog.search_features(data,query),query)
        self.assertTrue(all(f["category"]=="test_generation" for f in catalog.search_features(data,"",category="test_generation")))
        self.assertTrue(all(f["status"]=="deprecated" for f in catalog.search_features(data,"",status="deprecated")))
        command = [sys.executable,str(ROOT/"tools/cplib.py"),"search","文字列","--category","test_generation","--json"]
        result = subprocess.run(command,cwd="/tmp",capture_output=True,text=True)
        self.assertEqual(result.returncode,0,result.stderr)
        self.assertTrue(json.loads(result.stdout))


if __name__ == "__main__":
    unittest.main()
