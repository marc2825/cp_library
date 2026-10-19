"""Parallel multi-seed external solver/scorer evaluation and saved-case replay."""
import argparse
from concurrent.futures import ThreadPoolExecutor
from dataclasses import asdict
from fractions import Fraction
import json
import math
from pathlib import Path
import re
import tempfile
from threading import Event

from .process import execute


def load_config(path: Path) -> dict:
    config = json.loads(path.read_text())
    config["cwd"] = str((path.resolve().parent / config.get("cwd", ".")).resolve())
    config["timeout"] = float(config.get("timeout", 2))
    if not math.isfinite(config["timeout"]) or config["timeout"] <= 0:
        raise ValueError("timeout must be finite and positive")
    configs = config.get("configurations")
    if not isinstance(configs, list) or not configs:
        raise ValueError("configurations must be a nonempty list")
    names = [c["name"] for c in configs]
    if any(not isinstance(n, str) or not re.fullmatch(r"[A-Za-z0-9_-]+", n) for n in names) or len(set(names)) != len(names):
        raise ValueError("configuration names must be distinct simple identifiers")
    for command in [config.get("generator"), config.get("scorer"), *(c.get("solver") for c in configs)]:
        if not isinstance(command, list) or not command or not all(isinstance(s, str) and s for s in command):
            raise ValueError("commands must be nonempty argv arrays")
    config["goal"] = config.get("goal", "maximize")
    config["score_stream"] = config.get("score_stream", "stdout")
    config["score_regex"] = config.get("score_regex", r"Score\s*=\s*([-+0-9.eE]+)")
    if config["goal"] not in ("maximize", "minimize") or config["score_stream"] not in ("stdout", "stderr"):
        raise ValueError("invalid goal or score_stream")
    if re.compile(config["score_regex"]).groups != 1:
        raise ValueError("score_regex must have exactly one capture group")
    return config


def command_args(argv, **values):
    # Substitute only known placeholders, once, without shell expansion.
    return [re.sub(r"\{(seed|input|output|workdir)\}", lambda m: str(values[m[1]]), s) for s in argv]


def record_process(directory, name, result):
    record = asdict(result)
    for stream in ("stdout", "stderr"):
        (directory / f"{name}.{stream}").write_bytes(record.pop(stream))
    return record


def generate(config, seed, cancel=None):
    return execute(command_args(config["generator"], seed=seed), b"", Path(config["cwd"]), config["timeout"], cancel=cancel)


def run_case(config, name, seed, data, directory, generator_result=None, cancel=None):
    directory.mkdir(parents=True, exist_ok=False)
    directory = directory.resolve()
    (directory / "input.txt").write_bytes(data)
    result = {"version": 1, "name": name, "seed": seed, "config": config,
              "status": "ok", "score": None, "processes": {}}
    if generator_result is not None:
        result["processes"]["generator"] = record_process(directory, "generator", generator_result)
        if not generator_result.ok:
            result["status"] = "generator_failed"
    if result["status"] == "ok":
        chosen = next(c for c in config["configurations"] if c["name"] == name)
        values = dict(seed=seed, input=directory / "input.txt", output=directory / "solver.stdout", workdir=directory)
        for stage, argv, stdin in [("solver", chosen["solver"], data), ("scorer", config["scorer"], b"")]:
            process = execute(command_args(argv, **values), stdin, Path(config["cwd"]), config["timeout"], cancel=cancel)
            result["processes"][stage] = record_process(directory, stage, process)
            if not process.ok:
                result["status"] = stage + "_failed"
                break
            if stage == "scorer":
                matches = re.findall(config["score_regex"], getattr(process, config["score_stream"]).decode(errors="replace"))
                try:
                    if len(matches) != 1:
                        raise ValueError("scorer must produce exactly one score match")
                    score = float(matches[0])
                    if not math.isfinite(score):
                        raise ValueError("score must be finite")
                    result["score"] = score
                except ValueError as exc:
                    result["status"], result["detail"] = "invalid_score", str(exc)
    (directory / "case.json").write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    return result


def summarize(results, names, seeds, goal):
    if not names or not seeds or len(set(names)) != len(names) or len(set(seeds)) != len(seeds) or goal not in ("maximize", "minimize"):
        raise ValueError("need nonempty unique names/seeds and a valid goal")
    rows = []
    by_key = {(r["name"], r["seed"]): r for r in results}
    if len(by_key) != len(results) or set(by_key) != {(n, s) for n in names for s in seeds}:
        raise ValueError("need exactly one result per configuration and seed")
    for name in names:
        selected = [by_key[name, seed] for seed in seeds]
        scores = [r["score"] for r in selected if r["status"] == "ok"]
        if any(score is None or not math.isfinite(score) for score in scores):
            raise ValueError("successful scores must be finite")
        # Exact rational accumulation avoids overflow even for finite scores
        # near DBL_MAX; conversion to float happens only after division.
        ordered = sorted(scores)
        mean = float(sum(map(Fraction, scores), Fraction()) / len(scores)) if scores else None
        median = float((Fraction(ordered[(len(scores) - 1) // 2]) + Fraction(ordered[len(scores) // 2])) / 2) if scores else None
        row = {"name": name, "success": len(scores), "failed": len(seeds) - len(scores),
               "mean": mean, "median": median,
               "min": min(scores) if scores else None, "max": max(scores) if scores else None,
               "eligible": len(scores) == len(seeds)}
        rows.append(row)
    eligible = [r for r in rows if r["eligible"]]
    best = (min(eligible, key=lambda r: r["mean"]) if goal == "minimize" else
            max(eligible, key=lambda r: r["mean"])) if eligible else None
    return {"goal": goal, "seeds": seeds, "configurations": rows, "best": best["name"] if best else None}


def run_batch(config, seeds, jobs, directory):
    if not seeds or any(type(s) is not int for s in seeds) or len(set(seeds)) != len(seeds) or jobs < 1:
        raise ValueError("seeds must be distinct integers and jobs must be positive")
    directory.mkdir(parents=True, exist_ok=False)
    names = [c["name"] for c in config["configurations"]]
    (directory / "run.json").write_text(json.dumps({"version": 1, "config": config, "seeds": seeds, "jobs": jobs}, indent=2) + "\n")
    # Generation happens once per seed; every configuration receives identical bytes.
    cancel = Event()
    pool = ThreadPoolExecutor(max_workers=jobs)
    try:
        generated = dict(zip(seeds, pool.map(lambda seed: generate(config, seed, cancel), seeds)))
        tasks = [(name, seed) for name in names for seed in seeds]
        def evaluate(task):
            name, seed = task
            generator = generated[seed]
            return run_case(config, name, seed, generator.stdout,
                            directory / "cases" / name / str(seed), generator, cancel)
        results = list(pool.map(evaluate, tasks))
    finally:
        cancel.set()
        pool.shutdown(wait=True, cancel_futures=True)
    summary = summarize(results, names, seeds, config["goal"])
    (directory / "summary.json").write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n")
    return summary


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--config", type=Path)
    source.add_argument("--replay", type=Path, help="case directory; rerun with saved input")
    parser.add_argument("--seeds", type=int, nargs="+", default=list(range(10)))
    parser.add_argument("--jobs", type=int, default=1)
    parser.add_argument("--output", type=Path, help="new directory")
    args = parser.parse_args(argv)
    try:
        if args.output is None:
            parent = Path("build/benchmark")
            parent.mkdir(parents=True, exist_ok=True)
            args.output = Path(tempfile.mkdtemp(prefix="run-", dir=parent))
            args.output.rmdir()
        if args.replay:
            record = json.loads((args.replay / "case.json").read_text())
            if record["version"] != 1:
                raise ValueError("unsupported saved-case version")
            generator = generate(record["config"], record["seed"]) if record["status"] == "generator_failed" else None
            data = generator.stdout if generator is not None else (args.replay / "input.txt").read_bytes()
            result = run_case(record["config"], record["name"], record["seed"], data, args.output, generator)
            print(f"{result['status']}: score={result['score']}; {args.output}")
            return int(result["status"] != "ok")
        summary = run_batch(load_config(args.config), args.seeds, args.jobs, args.output)
        print(json.dumps(summary, ensure_ascii=False, allow_nan=False))
        print(f"saved to {args.output}")
        return int(any(row["failed"] for row in summary["configurations"]))
    except (OSError, ValueError, KeyError, TypeError, re.error) as exc:
        parser.exit(2, f"benchmark: {exc}\n")


if __name__ == "__main__":
    raise SystemExit(main())
