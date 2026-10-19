"""POSIX interactive harness: solver <-> judge, binary transcript and timeouts."""
import argparse
import base64
import json
import math
import os
from pathlib import Path
import selectors
import subprocess
import tempfile
import time

from .process import stop_process


def load_config(path: Path) -> dict:
    config = json.loads(path.read_text())
    for name in ("solver", "judge"):
        command = config.get(name)
        if not isinstance(command, list) or not command or not all(isinstance(x, str) and x for x in command):
            raise ValueError(f"{name} must be a nonempty argv array")
    config["cwd"] = str((path.resolve().parent / config.get("cwd", ".")).resolve())
    for name, default in (("timeout", 10), ("idle_timeout", 2)):
        config[name] = float(config.get(name, default))
        if not math.isfinite(config[name]) or config[name] <= 0:
            raise ValueError(f"{name} must be finite and positive")
    config["max_bytes"] = config.get("max_bytes", 1 << 20)
    if type(config["max_bytes"]) is not int or config["max_bytes"] < 1:
        raise ValueError("max_bytes must be a positive integer")
    return config


def run_session(config: dict, directory: Path, seed: int = 0) -> dict:
    if os.name != "posix":
        raise ValueError("interactive pipes require POSIX")
    directory.mkdir(parents=True, exist_ok=False)
    commands = {name: [arg.replace("{seed}", str(seed)) for arg in config[name]] for name in ("solver", "judge")}
    (directory / "config.json").write_text(json.dumps(config, indent=2) + "\n")
    started = time.monotonic()
    last_activity = started
    processes, streams, reads, writes = {}, {}, {}, {}
    pending = {"solver": bytearray(), "judge": bytearray()}
    source_eof = {"solver": False, "judge": False}
    total, status, detail = 0, "ok", ""
    peer = {"solver": "judge", "judge": "solver"}
    with selectors.DefaultSelector() as selector, (directory / "transcript.jsonl").open("w") as transcript:
        def unregister(stream):
            try:
                selector.unregister(stream)
            except KeyError:
                pass

        def close_input(name):
            stream = processes[name].stdin
            if not stream.closed:
                unregister(stream)
                stream.close()
                writes.pop(name, None)

        try:
            for name in ("solver", "judge"):
                processes[name] = subprocess.Popen(commands[name], cwd=config["cwd"], stdin=subprocess.PIPE,
                    stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=0, start_new_session=True)
                process = processes[name]
                os.set_blocking(process.stdin.fileno(), False)
                for kind in ("stdout", "stderr"):
                    stream = getattr(process, kind)
                    os.set_blocking(stream.fileno(), False)
                    reads[stream] = (name, kind)
                    streams[name, kind] = (directory / f"{name}.{kind}").open("wb")
                    selector.register(stream, selectors.EVENT_READ, ("read", name, kind))
            while reads or any(p.poll() is None for p in processes.values()) or any(pending.values()):
                now = time.monotonic()
                for name, process in processes.items():
                    if process.poll() not in (None, 0):
                        status, detail = name + "_exit", str(process.returncode)
                        break
                if status != "ok":
                    # Drain available stderr/stdout below before detecting the
                    # failure again; pipes are bounded by max_bytes.
                    if not reads:
                        break
                if now - started >= config["timeout"]:
                    status, detail = "timeout", "wall-clock limit"
                    break
                if now - last_activity >= config["idle_timeout"]:
                    status, detail = "idle_timeout", "no bytes transferred"
                    break
                events = selector.select(min(.05, config["timeout"] - (now - started),
                                             config["idle_timeout"] - (now - last_activity)))
                for key, _ in events:
                    action, name, kind = key.data
                    if action == "read":
                        try:
                            chunk = os.read(key.fd, min(65536, config["max_bytes"] - total + 1))
                        except BlockingIOError:
                            continue
                        if not chunk:
                            unregister(key.fileobj)
                            reads.pop(key.fileobj)
                            key.fileobj.close()
                            if kind == "stdout":
                                source_eof[peer[name]] = True
                                if not pending[peer[name]]:
                                    close_input(peer[name])
                            continue
                        allowed = chunk[:config["max_bytes"] - total]
                        total += len(allowed)
                        streams[name, kind].write(allowed)
                        transcript.write(json.dumps({"seconds": time.monotonic() - started,
                            "from": name, "stream": kind, "data_base64": base64.b64encode(allowed).decode()}) + "\n")
                        last_activity = time.monotonic()
                        if len(allowed) != len(chunk):
                            status, detail = "output_limit", "combined stdout/stderr exceeded max_bytes"
                            break
                        if kind == "stdout":
                            dest = peer[name]
                            if processes[dest].stdin.closed:
                                status, detail = "broken_pipe", f"{dest} stdin already closed"
                                break
                            pending[dest].extend(chunk)
                            if dest not in writes:
                                selector.register(processes[dest].stdin, selectors.EVENT_WRITE, ("write", dest, "stdin"))
                                writes[dest] = processes[dest].stdin
                    else:
                        try:
                            count = os.write(key.fd, pending[name])
                        except BlockingIOError:
                            continue
                        except BrokenPipeError:
                            status, detail = "broken_pipe", f"{name} closed stdin"
                            break
                        del pending[name][:count]
                        last_activity = time.monotonic()
                        if not pending[name]:
                            unregister(key.fileobj)
                            writes.pop(name)
                            if source_eof[name]:
                                close_input(name)
                if status != "ok":
                    break
        except OSError as exc:
            status, detail = "launch_or_io_error", str(exc)
        finally:
            # Capture exit codes before killing still-running peers.
            natural_codes = {name: process.poll() for name, process in processes.items()}
            for process in processes.values():
                stop_process(process)
                for stream in (process.stdin, process.stdout, process.stderr):
                    if stream and not stream.closed:
                        stream.close()
            for stream in streams.values():
                stream.close()
        if status == "ok":
            for name, code in natural_codes.items():
                if code != 0:
                    status, detail = name + "_exit", str(code)
                    break
        result = {"version": 1, "seed": seed, "status": status, "detail": detail,
                  "elapsed": time.monotonic() - started, "bytes": total, "commands": commands,
                  "returncodes": {name: process.returncode for name, process in processes.items()},
                  "natural_returncodes": natural_codes}
        (directory / "result.json").write_text(json.dumps(result, indent=2) + "\n")
        return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--output", type=Path, help="new directory; existing directories are not overwritten")
    args = parser.parse_args(argv)
    try:
        config = load_config(args.config)
        if args.output is None:
            parent = Path("build/interactive")
            parent.mkdir(parents=True, exist_ok=True)
            args.output = Path(tempfile.mkdtemp(prefix="run-", dir=parent))
            args.output.rmdir()
        result = run_session(config, args.output, args.seed)
        print(f"{result['status']}: {args.output}")
        return int(result["status"] != "ok")
    except (OSError, ValueError, KeyError, TypeError) as exc:
        parser.exit(2, f"interactive: {exc}\n")


if __name__ == "__main__":
    raise SystemExit(main())
