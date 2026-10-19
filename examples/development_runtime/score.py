from pathlib import Path
import sys

values = list(map(int, Path(sys.argv[1]).read_text().split()))
try:
    index = int(Path(sys.argv[2]).read_text().strip())
    if len(values) != values[0] + 1 or not 0 <= index < values[0]:
        raise ValueError("invalid selected index")
except (ValueError, IndexError) as exc:
    print(exc, file=sys.stderr)
    raise SystemExit(1)
print(f"Score = {values[index + 1]}", file=sys.stderr)
