import base64
import contextlib
import io
import json
import math
import os
from pathlib import Path
import sys
import tempfile
from threading import Event, Timer
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from tools.cp_tools import benchmark, interactive
from tools.cp_tools.process import execute


def python(code, *args):
    return [sys.executable, "-c", code, *map(str, args)]


class Workspace(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="cp runtime spaces ")
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)

    def config_file(self, config):
        path = self.root / "config.json"
        path.write_text(json.dumps(config))
        return path


@unittest.skipUnless(os.name == "posix", "interactive harness uses POSIX pipes")
class Interactive(Workspace):
    def config(self, solver, judge, **extra):
        return interactive.load_config(self.config_file(dict(solver=solver, judge=judge, cwd=".",
            timeout=extra.get("timeout", 3), idle_timeout=extra.get("idle_timeout", 1), max_bytes=extra.get("max_bytes", 1 << 20))))

    def session(self, config):
        return interactive.run_session(config, self.root / "run")

    def test_bidirectional_and_transcript(self):
        solver = python("import sys; n=int(input()); print(n*2,flush=True); assert input()=='ok'")
        judge = python("import sys; print(21,flush=True); assert input()=='42'; print('ok',flush=True); print('accepted',file=sys.stderr)")
        result = self.session(self.config(solver, judge))
        self.assertEqual(result["status"], "ok", result)
        self.assertEqual(result["natural_returncodes"], {"solver": 0, "judge": 0})
        self.assertEqual((self.root / "run/solver.stdout").read_bytes(), b"42\n")
        events = [json.loads(line) for line in (self.root / "run/transcript.jsonl").read_text().splitlines()]
        for name in ("solver", "judge"):
            for kind in ("stdout", "stderr"):
                data = b"".join(base64.b64decode(e["data_base64"]) for e in events if e["from"] == name and e["stream"] == kind)
                self.assertEqual(data, (self.root / f"run/{name}.{kind}").read_bytes())

    def test_binary_partial_writes_and_backpressure(self):
        code = "import sys; a=int(sys.argv[1]); b=int(sys.argv[2]); sys.stdout.buffer.write(bytes([a])*200000); sys.stdout.buffer.flush(); data=sys.stdin.buffer.read(200000); assert data==bytes([b])*200000"
        result = self.session(self.config(python(code, 0, 255), python(code, 255, 0), timeout=5, idle_timeout=3))
        self.assertEqual(result["status"], "ok", result)
        self.assertEqual(result["bytes"], 400000)

    def test_eof_is_forwarded(self):
        result = self.session(self.config(python("import sys; assert sys.stdin.read()=='hello\\n'"), python("print('hello')")))
        self.assertEqual(result["status"], "ok", result)

    def test_no_flush_deadlock(self):
        result = self.session(self.config(python("import sys; sys.stdout.write('?'); sys.stdin.readline()"),
            python("input()"), idle_timeout=.2))
        self.assertEqual(result["status"], "idle_timeout", result)
        self.assertTrue(all(code is not None for code in result["returncodes"].values()))

    def test_wall_timeout_despite_activity(self):
        result = self.session(self.config(python("import sys,time\nwhile True: print('.',file=sys.stderr,flush=True); time.sleep(.01)"),
            python("import time; time.sleep(10)"), timeout=.3, idle_timeout=1))
        self.assertEqual(result["status"], "timeout", result)

    def test_output_limit(self):
        result = self.session(self.config(python("import sys; sys.stderr.write('x'*10000); sys.stderr.flush()"),
            python("import time; time.sleep(2)"), max_bytes=100))
        self.assertEqual(result["status"], "output_limit", result)
        self.assertEqual(result["bytes"], 100)

    def test_exit_and_launch_failures(self):
        result = self.session(self.config(python("import sys; print('failure',file=sys.stderr); sys.exit(7)"),
            python("import time; time.sleep(10)")))
        self.assertEqual(result["status"], "solver_exit", result)
        self.assertIn(b"failure", (self.root / "run/solver.stderr").read_bytes())
        config = self.config(python("import time; time.sleep(10)"), ["/no/such/local-judge"])
        result = interactive.run_session(config, self.root / "failed-launch")
        self.assertEqual(result["status"], "launch_or_io_error")
        self.assertIsNotNone(result["returncodes"]["solver"])

    def test_existing_output_and_invalid_config(self):
        config = self.config(python("pass"), python("pass"))
        with self.assertRaises(FileExistsError): interactive.run_session(config, self.root)
        for update in [{"solver": "shell"}, {"timeout": 0}, {"max_bytes": -1}, {"idle_timeout": math.nan}]:
            with self.assertRaises(ValueError): interactive.load_config(self.config_file({**config, **update}))


class Benchmark(Workspace):
    def config(self, **extra):
        config = dict(cwd=".", timeout=2, goal="maximize",
            generator=python("import sys; print(sys.argv[1])", "{seed}"),
            configurations=[{"name": "base", "solver": python("print(int(input())+10)")},
                            {"name": "better", "solver": python("print(int(input())+20)")}],
            scorer=python("from pathlib import Path; import sys; print('Score = '+Path(sys.argv[1]).read_text().strip())", "{output}"))
        return benchmark.load_config(self.config_file({**config, **extra}))

    def test_batch_comparison_and_replay(self):
        config = self.config()
        serial = benchmark.run_batch(config, [0, 2, -3], 1, self.root / "serial")
        parallel = benchmark.run_batch(config, [0, 2, -3], 3, self.root / "parallel")
        self.assertEqual(serial, parallel)
        self.assertEqual(parallel["best"], "better")
        self.assertEqual(parallel["configurations"][0]["mean"], 29 / 3)
        for seed in [0, 2, -3]:
            base = self.root / f"parallel/cases/base/{seed}"
            better = self.root / f"parallel/cases/better/{seed}"
            self.assertEqual((base / "input.txt").read_bytes(), (better / "input.txt").read_bytes())
        case = self.root / "parallel/cases/better/2"
        metadata = json.loads((case / "case.json").read_text())
        metadata["config"]["generator"] = ["/generator-must-not-be-rerun"]
        (case / "case.json").write_text(json.dumps(metadata))
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(benchmark.main(["--replay", str(case), "--output", str(self.root / "replay")]), 0)
        self.assertEqual(json.loads((self.root / "replay/case.json").read_text())["score"], 22)
        config["goal"] = "minimize"
        self.assertEqual(benchmark.run_batch(config, [1], 2, self.root / "min")["best"], "base")

    def test_actual_parallel_execution(self):
        # Both solver processes must reach the barrier before either may finish.
        code = "from pathlib import Path; import sys,time; Path('ready-'+sys.argv[1]).touch()\nwhile len(list(Path('.').glob('ready-*')))<2: time.sleep(.01)\nprint(7)"
        config = self.config(configurations=[{"name": "base", "solver": python(code, "{seed}")}])
        result = benchmark.run_batch(config, [4, 5], 2, self.root / "barrier")
        self.assertEqual(result["configurations"][0]["success"], 2)

    def test_failure_stages_and_exclusion(self):
        for stage in ("generator", "solver", "scorer"):
            config = self.config()
            command = python("import sys; print('failure',file=sys.stderr); sys.exit(4)")
            if stage == "solver": config["configurations"][1]["solver"] = command
            else: config[stage] = command
            summary = benchmark.run_batch(config, [0], 2, self.root / stage)
            self.assertEqual(summary["best"], "base" if stage == "solver" else None)
            record = json.loads((self.root / stage / "cases/better/0/case.json").read_text())
            self.assertEqual(record["status"], stage + "_failed")
            self.assertIsNone(record["score"])

    def test_invalid_scores_timeout_and_score_stream(self):
        config = self.config(score_stream="stderr", scorer=python("import sys; print('Score = -2.5',file=sys.stderr)"))
        result = benchmark.run_case(config, "base", 0, b"0\n", self.root / "stderr")
        self.assertEqual(result["score"], -2.5)
        for index, text in enumerate(["no score", "Score = 1e999", "Score = 1\nScore = 2"]):
            config = self.config(scorer=python("import sys; print(sys.argv[1])", text))
            result = benchmark.run_case(config, "base", 0, b"0\n", self.root / f"bad{index}")
            self.assertEqual(result["status"], "invalid_score")
        config = self.config(timeout=.15)
        config["configurations"][0]["solver"] = python("import time; time.sleep(10)")
        result = benchmark.run_case(config, "base", 0, b"0\n", self.root / "timeout")
        self.assertEqual(result["status"], "solver_failed")
        self.assertTrue(result["processes"]["solver"]["timeout"])

    def test_summary_boundaries_and_ties(self):
        for scores in [[sys.float_info.max, sys.float_info.max], [-sys.float_info.max, sys.float_info.max], [5e-324, 5e-324]]:
            rows = [dict(name=n, seed=i, status="ok", score=score) for n in ["a", "b"] for i, score in enumerate(scores)]
            result = benchmark.summarize(rows, ["a", "b"], [0, 1], "maximize")
            self.assertEqual(result["best"], "a")
            self.assertTrue(math.isfinite(result["configurations"][0]["median"]))
            self.assertEqual(result["configurations"][0]["mean"], result["configurations"][0]["median"])
        with self.assertRaises(ValueError): benchmark.summarize([], ["a"], [0], "maximize")

    def test_config_and_output_validation(self):
        config = self.config()
        for update in [{"timeout": math.inf}, {"goal": "unknown"}, {"score_regex": "Score"},
                       {"score_regex": "("}, {"generator": "shell"},
                       {"configurations": [{"name": "../escape", "solver": python("pass")}]}]:
            with self.assertRaises((ValueError, benchmark.re.error)): benchmark.load_config(self.config_file({**config, **update}))
        with self.assertRaises(ValueError): benchmark.run_batch(config, [1, 1], 2, self.root / "duplicate")
        with self.assertRaises(FileExistsError): benchmark.run_batch(config, [1], 1, self.root)

    def test_cancellation_cleans_process(self):
        cancel = Event()
        timer = Timer(.15, cancel.set)
        timer.start()
        try:
            result = execute(python("import time; time.sleep(10)"), b"", self.root, 20, cancel=cancel)
        finally:
            timer.join()
        self.assertTrue(result.cancelled)
        self.assertFalse(result.ok)
        self.assertLess(result.elapsed, 3)
        self.assertTrue(execute(python("pass"), b"", self.root, 20, cancel=cancel).cancelled)


if __name__ == "__main__":
    unittest.main()
