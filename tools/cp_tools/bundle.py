"""Expand literal local C++ includes, retaining system includes and conditions."""
import argparse
from bisect import bisect_right
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys
import tempfile


class BundleError(ValueError):
    pass


# Ignore directive-looking text in comments, ordinary strings and raw strings.
TOKENS = re.compile(r'//[^\n]*|/\*[\s\S]*?\*/|(?:u8|u|U|L)?R"([^ ()\\\t\r\n]{0,16})\([\s\S]*?\)\1"|"(?:\\[\s\S]|[^"\\])*"|(?<![0-9A-Za-z_])\'(?:\\[^\n]|[^\'\\\n])*\'')
DIRECTIVE = re.compile(r'^[ \t]*#[ \t]*(\w+)\b([^\n]*)', re.MULTILINE)


def directives(source: str):
    spans = [(m.start(), m.end()) for m in TOKENS.finditer(source)]
    starts = [a for a, _ in spans]
    for match in DIRECTIVE.finditer(source):
        pos = source.index("#", match.start(), match.end())
        i = bisect_right(starts, pos) - 1
        if i >= 0 and pos < spans[i][1]:
            continue
        yield match


class Bundler:
    def __init__(self, include_dirs=()):
        self.include_dirs = [Path(p).resolve() for p in include_dirs]
        self.unconditional = set()
        self.active = set()
        self.dependencies = set()

    def find(self, name: str, parent: Path, quoted: bool) -> Path | None:
        for directory in ([parent] if quoted else []) + self.include_dirs:
            candidate = (directory / name).resolve()
            if candidate.is_file():
                return candidate
        return None

    def expand(self, path: Path, unconditional=True) -> str:
        path = path.resolve()
        source = path.read_text(encoding="utf-8")
        # Phase-2 line splicing precedes comment recognition. Do it before lexing.
        source = re.sub(r'\\\r?\n', '', source)
        matches = list(directives(source))
        once = any(m[1] == "pragma" and m[2].strip() == "once" for m in matches)
        # Conventional whole-file include guards are retained verbatim.
        clean = re.sub(r'//[^\n]*|/\*[\s\S]*?\*/', '', source)
        classic = re.match(r'\s*#\s*ifndef\s+(\w+)\s*\n\s*#\s*define\s+\1\b', clean)
        whole_guard = False
        if classic:
            nesting = 0
            for match in matches:
                if match[1] in ("if", "ifdef", "ifndef"):
                    nesting += 1
                elif match[1] == "endif":
                    nesting -= 1
                    if nesting == 0:
                        suffix = re.sub(r'//[^\n]*|/\*[\s\S]*?\*/', '', source[match.end():])
                        whole_guard = not suffix.strip()
                        break
        guarded = once or whole_guard
        self.dependencies.add(path)
        if guarded and path in self.unconditional:
            return f"// already included: {path.name}\n"
        if path in self.active:
            if guarded:
                return f"// guarded cycle: {path.name}\n"
            raise BundleError(f"unguarded include cycle: {path}")
        if guarded and unconditional:
            self.unconditional.add(path)
        self.active.add(path)
        guard = "CP_BUNDLE_" + hashlib.sha256(str(path).encode()).hexdigest().upper()
        output = [f"// begin: {path.name}\n"]
        if once:
            output.append(f"#ifndef {guard}\n#define {guard}\n")
        cursor, depth = 0, 0
        for match in matches:
            key, argument = match[1], match[2].strip()
            replacement = None
            if key in ("if", "ifdef", "ifndef"):
                depth += 1
            elif key == "endif":
                depth -= 1
            elif key == "pragma" and argument == "once":
                replacement = "// pragma once replaced by bundle guard"
            elif key in ("include", "include_next"):
                include = re.fullmatch(r'([<"])([^>"\n]+)[>"][ \t]*(?://[^\n]*|/\*.*?\*/[ \t]*)?', argument)
                if key != "include" or not include:
                    raise BundleError(f"only literal #include is supported: {path}: {argument}")
                local = self.find(include[2], path.parent, include[1] == '"')
                if local is not None:
                    # Conditional occurrences get guards but do not suppress a
                    # later unconditional expansion (the first may be #if 0).
                    replacement = self.expand(local, unconditional and depth == 0)
                elif include[1] == '"':
                    raise BundleError(f"missing quoted include {include[2]!r} in {path}")
            if replacement is not None:
                output.append(source[cursor:match.start()])
                output.append(replacement)
                cursor = match.end()
        output.append(source[cursor:])
        if once:
            output.append(f"\n#endif // {guard}\n")
        output.append(f"\n// end: {path.name}\n")
        self.active.remove(path)
        return "".join(output)


def bundle(source: Path, include_dirs=()) -> str:
    return Bundler(include_dirs).expand(source)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("-I", "--include", action="append", type=Path, default=[])
    parser.add_argument("-o", "--output", type=Path)
    parser.add_argument("--check", action="store_true", help="compile C++17 before writing output")
    parser.add_argument("--compiler", default="g++")
    parser.add_argument("--flag", action="append", default=[])
    parser.add_argument("--dependencies", type=Path, help="write expanded dependency paths as JSON")
    args = parser.parse_args(argv)
    try:
        bundler = Bundler([Path.cwd(), *args.include])
        text = bundler.expand(args.source)
        if args.output and args.output.resolve() in bundler.dependencies:
            raise BundleError("output must not overwrite an input dependency")
        if args.dependencies and (args.dependencies.resolve() in bundler.dependencies or
                                  args.output and args.dependencies.resolve() == args.output.resolve()):
            raise BundleError("dependency report must use a separate file")
        if args.check:
            with tempfile.TemporaryDirectory(prefix="cp-bundle-") as temporary:
                flat = Path(temporary) / "main.cpp"
                flat.write_text(text)
                subprocess.run([args.compiler, "-std=c++17", "-O2", *args.flag, str(flat),
                                "-o", str(Path(temporary) / "main")], check=True)
        if args.output:
            args.output.write_text(text)
        else:
            sys.stdout.write(text)
        if args.dependencies:
            args.dependencies.write_text(json.dumps(sorted(map(str, bundler.dependencies)), indent=2) + "\n")
        return 0
    except (OSError, ValueError, subprocess.CalledProcessError) as exc:
        parser.exit(2, f"bundle: {exc}\n")


if __name__ == "__main__":
    raise SystemExit(main())
