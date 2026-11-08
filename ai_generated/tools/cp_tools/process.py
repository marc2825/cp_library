"""Shared finite-process execution and process-group cleanup."""
from dataclasses import dataclass
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time


def stop_process(proc):
    """Kill the process group (POSIX) and reap the direct child."""
    try:
        if os.name == "posix":
            os.killpg(proc.pid, signal.SIGKILL)
        elif proc.poll() is None:
            proc.kill()
    except ProcessLookupError:
        pass
    proc.wait()


@dataclass
class Result:
    argv: list[str]
    returncode: int | None
    timeout: bool
    elapsed: float
    stdout: bytes
    stderr: bytes
    cancelled: bool = False

    @property
    def ok(self):
        return self.returncode == 0 and not self.timeout and not self.cancelled


def execute(argv: list[str], data: bytes, cwd: Path, timeout: float, *, cancel=None) -> Result:
    if not argv or not all(isinstance(x, str) for x in argv):
        raise ValueError("command must be a nonempty string array")
    if not math.isfinite(timeout) or timeout <= 0:
        raise ValueError("timeout must be finite and positive")
    start = time.monotonic()
    if cancel is not None and cancel.is_set():
        return Result(argv, None, False, 0, b"", b"", True)
    with tempfile.TemporaryFile() as inp, tempfile.TemporaryFile() as out, tempfile.TemporaryFile() as err:
        inp.write(data)
        inp.seek(0)
        try:
            proc = subprocess.Popen(argv, cwd=cwd, stdin=inp, stdout=out, stderr=err,
                                    start_new_session=(os.name == "posix"))
        except OSError as exc:
            return Result(argv, None, False, time.monotonic() - start, b"", str(exc).encode())
        timed_out = cancelled = False
        try:
            while proc.poll() is None:
                if cancel is not None and cancel.is_set():
                    cancelled = True
                    break
                remaining = timeout - (time.monotonic() - start)
                if remaining <= 0:
                    timed_out = True
                    break
                try:
                    proc.wait(timeout=min(.05, remaining) if cancel is not None else remaining)
                except subprocess.TimeoutExpired:
                    continue
        finally:
            stop_process(proc)
        out.seek(0)
        err.seek(0)
        return Result(argv, proc.returncode, timed_out, time.monotonic() - start, out.read(), err.read(), cancelled)
