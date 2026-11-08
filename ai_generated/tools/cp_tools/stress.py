"""Generator / fast / naive differential runner; shell-free argv and replay."""
import argparse
from dataclasses import asdict
import json
import math
from pathlib import Path
import tempfile


from .process import Result, execute


def compare(a: bytes, b: bytes, mode: str) -> bool:
    if mode == "exact":
        return a == b
    if mode == "tokens":
        return a.split() == b.split()
    raise ValueError("compare must be exact or tokens")


def load_config(path: Path) -> dict:
    config = json.loads(path.read_text())
    for key in ("generator", "fast", "naive"):
        command = config.get(key)
        if not isinstance(command, list) or not command or not all(isinstance(x, str) and x for x in command):
            raise ValueError(f"{key} must be a nonempty string array")
    config["cwd"] = str((path.resolve().parent / config.get("cwd", ".")).resolve())
    config["timeout"] = float(config.get("timeout", 2))
    if not math.isfinite(config["timeout"]) or config["timeout"] <= 0:
        raise ValueError("timeout must be finite and positive")
    config["compare"] = config.get("compare", "tokens")
    compare(b"", b"", config["compare"])
    return config


def run_case(config: dict, seed: int, saved_input: bytes | None = None):
    results = {}
    if saved_input is None:
        argv = [part.replace("{seed}", str(seed)) for part in config["generator"]]
        results["generator"] = execute(argv, b"", Path(config["cwd"]), config["timeout"])
        saved_input = results["generator"].stdout
        if not results["generator"].ok:
            return "generator_failed", saved_input, results
    for key in ("fast", "naive"):
        results[key] = execute(config[key], saved_input, Path(config["cwd"]), config["timeout"])
    for key in ("fast", "naive"):
        if not results[key].ok:
            return f"{key}_failed", saved_input, results
    status = "ok" if compare(results["fast"].stdout, results["naive"].stdout, config["compare"]) else "mismatch"
    return status, saved_input, results


def save_failure(directory: Path, config: dict, seed: int, status: str, data: bytes, results: dict) -> Path:
    directory.mkdir(parents=True, exist_ok=True)
    target = Path(tempfile.mkdtemp(prefix=f"seed-{seed}-", dir=directory))
    (target / "input.txt").write_bytes(data)
    metadata = {"version": 1, "seed": seed, "status": status, "config": config, "results": {}}
    for key, result in results.items():
        record = asdict(result)
        for stream in ("stdout", "stderr"):
            (target / f"{key}.{stream}").write_bytes(record.pop(stream))
        metadata["results"][key] = record
    (target / "case.json").write_text(json.dumps(metadata, ensure_ascii=False, indent=2) + "\n")
    return target


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--config", type=Path)
    source.add_argument("--replay", type=Path, help="saved failure directory (executes recorded commands)")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--cases", type=int, default=100)
    parser.add_argument("--failures", type=Path, default=Path(".stress-failures"))
    args = parser.parse_args(argv)
    try:
        if args.replay:
            record = json.loads((args.replay / "case.json").read_text())
            if record["version"] != 1:
                raise ValueError("unsupported replay format")
            data = None if record["status"] == "generator_failed" else (args.replay / "input.txt").read_bytes()
            status, _, _ = run_case(record["config"], record["seed"], data)
            print(f"replay seed={record['seed']}: {status} (original: {record['status']})")
            return int(status != "ok")
        if args.cases < 1:
            raise ValueError("cases must be positive")
        config = load_config(args.config)
        for seed in range(args.seed, args.seed + args.cases):
            status, data, results = run_case(config, seed)
            if status != "ok":
                path = save_failure(args.failures, config, seed, status, data, results)
                print(f"{status}: seed={seed}; saved to {path}")
                return 1
        print(f"OK: {args.cases} cases, seeds {args.seed}..{args.seed + args.cases - 1}")
        return 0
    except (OSError, ValueError, KeyError, TypeError) as exc:
        parser.exit(2, f"stress: {exc}\n")


if __name__ == "__main__":
    raise SystemExit(main())
