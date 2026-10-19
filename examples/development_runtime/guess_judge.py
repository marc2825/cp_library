"""A local number-guessing protocol, not an online judge submission."""
import sys

answer = int(sys.argv[1]) % 100 + 1
print(100, flush=True)
for _ in range(9):
    parts = sys.stdin.readline().split()
    if len(parts) != 2 or parts[0] not in ("?", "!"):
        break
    try:
        value = int(parts[1])
    except ValueError:
        break
    if parts[0] == "!":
        if value == answer:
            print("accepted", file=sys.stderr)
            raise SystemExit(0)
        break
    if not 1 <= value <= 100:
        break
    print((answer > value) - (answer < value), flush=True)
print("wrong answer or protocol error", file=sys.stderr)
raise SystemExit(1)
