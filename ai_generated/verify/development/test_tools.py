import contextlib
import io
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from tools.cp_tools import random_data as rd
from tools.cp_tools import stress
from tools.cp_tools.bundle import bundle, BundleError


def components(graph):
    adj = [[] for _ in range(graph.n)]
    for u, v in graph.edges:
        assert 0 <= u < graph.n and 0 <= v < graph.n
        adj[u].append(v)
        adj[v].append(u)
    seen, count = set(), 0
    for i in range(graph.n):
        if i in seen:
            continue
        count += 1
        seen.add(i)
        stack = [i]
        while stack:
            for v in adj[stack.pop()]:
                if v not in seen:
                    seen.add(v)
                    stack.append(v)
    return count


class Generators(unittest.TestCase):
    def test_seed_and_bounds(self):
        rng = rd.Random(0)
        self.assertEqual([rng.next_u64() for _ in range(3)],
                         [0xe220a8397b1dcdaf, 0x6e789e6aa1b965f4, 0x06c45d188009454f])
        self.assertEqual(rd.tree(rd.Random(-1), 40), rd.tree(rd.Random((1 << 64) - 1), 40))
        for bound in [1, 2, 3, 17, (1 << 63) + 1, 1 << 64]:
            self.assertTrue(all(0 <= rng.below(bound) < bound for _ in range(100)))
        for bound in [0, -1, (1 << 64) + 1]:
            with self.assertRaises(ValueError): rng.below(bound)
        self.assertEqual(rd.tree(rng, 0).text(), "0 0\n")
        self.assertEqual(rd.forest(rng, 0, 0).edges, ())
        for call in [lambda: rd.tree(rng, -1), lambda: rd.forest(rng, 2, 0),
                     lambda: rd.undirected_graph(rng, 0, 1, multi=True),
                     lambda: rd.undirected_graph(rng, 3, 0, components=1),
                     lambda: rd.undirected_graph(rng, 3, 4), lambda: rd.dag(rng, 2, 2),
                     lambda: rd.simple_polygon(rng, 2), lambda: rd.simple_polygon(rng, 5, 1)]:
            with self.assertRaises(ValueError): call()

    def test_graph_invariants(self):
        for seed in range(120):
            rng = rd.Random(seed)
            n = rng.integer(1, 18)
            tree = rd.tree(rng, n)
            self.assertEqual((components(tree), len(tree.edges)), (1, n - 1))
            c = rng.integer(1, n)
            forest = rd.forest(rng, n, c)
            self.assertEqual((components(forest), len(forest.edges)), (c, n - c))
            m = rng.integer(n - c, (n - c + 1) * (n - c) // 2)
            g = rd.undirected_graph(rng, n, m, components=c)
            self.assertEqual(components(g), c)
            self.assertEqual(len(g.edges), m)
            self.assertEqual(len(set(tuple(sorted(e)) for e in g.edges)), m)
            self.assertTrue(all(u != v for u, v in g.edges))
            g = rd.undirected_graph(rng, n, 50, components=c, multi=True, loops=True)
            self.assertEqual((components(g), len(g.edges)), (c, 50))
            g = rd.dag(rng, n, rng.integer(0, n * (n - 1) // 2))
            self.assertTrue(g.directed)
            self.assertEqual(len(set(g.edges)), len(g.edges))
            remaining = set(range(n))
            while remaining:
                zero = {v for v in remaining if not any(b == v and a in remaining for a, b in g.edges)}
                self.assertTrue(zero)
                remaining -= zero
        for n in range(1, 7):
            for c in range(1, n + 1):
                for m in range(n - c, (n - c + 1) * (n - c) // 2 + 1):
                    g = rd.undirected_graph(rd.Random(42), n, m, components=c)
                    self.assertEqual((components(g), len(g.edges)), (c, m))

    def test_polygon_geometry(self):
        def cross(a, b, c):
            return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
        for seed in range(100):
            p = rd.simple_polygon(rd.Random(seed), 3 + seed % 20, 20)
            self.assertEqual(len(set(p)), len(p))
            for i in range(len(p)):
                a, b = p[i], p[(i + 1) % len(p)]
                self.assertTrue(all(cross(a, b, c) > 0 for c in p if c != a and c != b))


class Stress(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="cp stress spaces ")
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.config = {"cwd": str(self.root), "timeout": 1.0, "compare": "tokens",
                       "generator": [sys.executable, "-c", "import sys; print(sys.argv[1])", "{seed}"],
                       "fast": [sys.executable, "-c", "import sys; print(sys.stdin.read().strip())"],
                       "naive": [sys.executable, "-c", "import sys; print(sys.stdin.read().strip())"]}

    def test_compare_and_failures(self):
        self.assertTrue(stress.compare(b"1\t2\r\n", b"1 2", "tokens"))
        self.assertFalse(stress.compare(b"1\n", b"1", "exact"))
        self.assertFalse(stress.compare(b"1.0", b"1", "tokens"))
        self.assertEqual(stress.run_case(self.config, 42)[0], "ok")
        for key in ("generator", "fast", "naive"):
            config = {**self.config, key: [sys.executable, "-c", "import sys; print('bad'); sys.exit(7)"]}
            status, _, results = stress.run_case(config, 0)
            self.assertEqual(status, key + "_failed")
            self.assertEqual(results[key].returncode, 7)
        result = stress.execute([str(self.root / "missing")], b"", self.root, 1)
        self.assertIsNone(result.returncode)
        self.assertFalse(result.ok)
        literal = "$(touch unwanted); `pwd`"
        result = stress.execute([sys.executable, "-c", "import sys; print(sys.argv[1])", literal], b"", self.root, 1)
        self.assertEqual(result.stdout.decode().strip(), literal)
        self.assertFalse((self.root / "unwanted").exists())

    def test_timeout_and_partial_output(self):
        result = stress.execute([sys.executable, "-c", "import time; print('partial',flush=True); time.sleep(10)"], b"", self.root, .15)
        self.assertTrue(result.timeout)
        self.assertEqual(result.stdout, b"partial\n")
        self.assertLess(result.elapsed, 3)

    @unittest.skipUnless(os.name == "posix", "process group cleanup is POSIX")
    def test_descendant_cleanup(self):
        code = "import subprocess,sys,time; p=subprocess.Popen([sys.executable,'-c','import time; time.sleep(60)']); print(p.pid,flush=True); time.sleep(60)"
        result = stress.execute([sys.executable, "-c", code], b"", self.root, .2)
        self.assertTrue(result.timeout)
        pid = int(result.stdout)
        status = Path(f"/proc/{pid}/stat")
        if status.exists():
            self.assertIn(status.read_text().split()[2], ["Z", "X"])

    def test_save_and_replay_saved_input(self):
        config = {**self.config, "fast": [sys.executable, "-c", "print('wrong')"]}
        status, data, results = stress.run_case(config, 41)
        self.assertEqual(status, "mismatch")
        path = stress.save_failure(self.root, config, 41, status, data, results)
        self.assertEqual((path / "input.txt").read_bytes(), b"41\n")
        self.assertEqual((path / "fast.stdout").read_bytes(), b"wrong\n")
        record = json.loads((path / "case.json").read_text())
        record["config"]["generator"] = ["missing-generator"]
        (path / "case.json").write_text(json.dumps(record))
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(stress.main(["--replay", str(path)]), 1)
            record["config"]["fast"] = record["config"]["naive"]
            (path / "case.json").write_text(json.dumps(record))
            self.assertEqual(stress.main(["--replay", str(path)]), 0)
        self.assertNotEqual(path, stress.save_failure(self.root, config, 41, status, data, results))

    def test_configuration_and_cli(self):
        path = self.root / "config.json"
        path.write_text(json.dumps({**self.config, "cwd": "."}))
        self.assertEqual(stress.load_config(path)["cwd"], str(self.root))
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(stress.main(["--config", str(path), "--cases", "3"]), 0)
        for update in [{"timeout": 0}, {"timeout": float("nan")}, {"generator": "shell command"}, {"compare": "float"}]:
            path.write_text(json.dumps({**self.config, **update}))
            with self.assertRaises(ValueError): stress.load_config(path)


class Bundle(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="cp bundle ")
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)

    def put(self, name, source):
        path = self.root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(source)
        return path

    def compile_run(self, path, expected, flags=()):
        flat = self.put("flattened.cpp", bundle(path, [self.root]))
        executable = self.root / "flat"
        subprocess.run([os.environ.get("CXX", "g++"), "-std=c++17", "-Wall", "-Wextra", "-Werror",
                        *flags, str(flat), "-o", str(executable)], check=True, capture_output=True)
        self.assertEqual(subprocess.check_output([str(executable)]).decode(), expected)

    def test_diamond_and_cycles(self):
        self.put("common.hpp", '#pragma once\nstruct Value { int x = 7; };\n')
        self.put("a.hpp", '#pragma once\n#include "common.hpp"\n#include "b.hpp"\n')
        self.put("b.hpp", '#pragma once\n#include "a.hpp"\n#include <common.hpp>\n')
        source = self.put("main.cpp", '#include <iostream>\n#include "a.hpp"\n#include "b.hpp"\nint main(){std::cout<<Value{}.x;}\n')
        self.assertEqual(bundle(source, [self.root]).count("struct Value"), 1)
        self.compile_run(source, "7")

    def test_inactive_first_include_and_raw_strings(self):
        self.put("value.hpp", '#pragma once\nconstexpr int value = 7;\n')
        source = self.put("main.cpp", '#include <iostream>\n#if ENABLED\n#include "value.hpp"\n#endif\n#include "value.hpp"\n/*\n#include "missing.hpp"\n*/\nconst char* s = R"tag(\n#include "missing.hpp"\n)tag";\nconstexpr int n=1\'000;\nint main(){std::cout<<value;}\n')
        self.compile_run(source, "7", ["-DENABLED=0"])
        self.compile_run(source, "7", ["-DENABLED=1"])

    def test_guard_and_unguarded_xmacro(self):
        self.put("guard.hpp", '#ifndef VALUE_GUARD\n#define VALUE_GUARD\n#include "guard.hpp"\nstruct Value {};\n#endif\n')
        self.put("repeat.hpp", 'ITEM(3)\n')
        source = self.put("main.cpp", '#include <iostream>\n#include "guard.hpp"\n#include "guard.hpp"\nint main(){int x=0;\n#define ITEM(a) x+=a;\n#include "repeat.hpp"\n#include "repeat.hpp"\nstd::cout<<x;}\n')
        self.compile_run(source, "6")

    def test_errors_and_splicing(self):
        self.put("a.hpp", '#include "a.hpp"\n')
        for source in ['#include "missing.hpp"', '#include HEADER', '#include "a.hpp"']:
            with self.assertRaises(BundleError): bundle(self.put("main.cpp", source), [self.root])
        self.put("a.hpp", '#pragma once\nint value=7;\n')
        path = self.put("main.cpp", '#include \\\n"a.hpp"\n#include <iostream>\nint main(){std::cout<<value;}')
        self.compile_run(path, "7")

    def test_partial_guard_keeps_trailing_code(self):
        self.put("partial.hpp", '#ifndef PARTIAL\n#define PARTIAL\nx+=1;\n#endif\nx+=2;\n')
        source = self.put("main.cpp", '#include <iostream>\nint main(){int x=0;\n#include "partial.hpp"\n#include "partial.hpp"\nstd::cout<<x;}\n')
        self.compile_run(source, "5")

    def test_cli_check_preserves_output_on_error(self):
        source = self.put("main.cpp", 'syntax error\n')
        output = self.put("saved.cpp", 'original\n')
        command = [sys.executable, "-m", "tools.cp_tools.bundle", str(source), "--check", "-o", str(output)]
        self.assertNotEqual(subprocess.run(command, cwd=ROOT, capture_output=True).returncode, 0)
        self.assertEqual(output.read_text(), 'original\n')
        source.write_text('int main(){}\n')
        command[-1] = str(source)
        self.assertNotEqual(subprocess.run(command, cwd=ROOT, capture_output=True).returncode, 0)


if __name__ == "__main__":
    unittest.main()
