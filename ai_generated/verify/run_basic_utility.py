#!/usr/bin/env python3
"""basic_utility: standalone headers and randomized comparisons with brute force."""
import argparse
import os
from pathlib import Path
import shlex
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def run(command, **kwargs):
    result = subprocess.run(list(map(str, command)), capture_output=True, text=True, timeout=600, **kwargs)
    if result.returncode:
        raise RuntimeError(f'{shlex.join(map(str, command))}\n{result.stdout}{result.stderr}')
    return result.stdout


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--compiler', default=os.environ.get('CXX', 'g++'))
    parser.add_argument('--sanitize', action='store_true')
    args = parser.parse_args()
    flags = shlex.split(args.compiler) + ['-std=c++17', '-Wall', '-Wextra', '-Wpedantic', '-Werror', '-I', ROOT]
    runtime = (['-O1', '-g', '-fsanitize=address,undefined', '-fno-sanitize-recover=all', '-fno-omit-frame-pointer']
               if args.sanitize else ['-O2'])
    headers = sorted(p.relative_to(ROOT).as_posix() for p in (ROOT / 'basic_utility').glob('*.hpp'))
    for header in headers:
        run(flags + ['-x', 'c++', '-fsyntax-only', '-'], input=f'#include "{header}"\n#include "{header}"\nint main(){{}}\n')
    print(f'{len(headers)} standalone/repeated headers passed', flush=True)
    with tempfile.TemporaryDirectory(prefix='cp-basic-utility-') as folder:
        for extra in [[], ['-DNDEBUG']]:
            binary = Path(folder) / 'random_test'
            run(flags + runtime + extra + [ROOT / 'verify/basic_utility/random_test.cpp', '-o', binary])
            print(run([binary]).strip() + (' (NDEBUG)' if extra else ''), flush=True)


if __name__ == '__main__':
    main()
