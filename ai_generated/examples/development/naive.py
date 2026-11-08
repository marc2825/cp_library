import sys

values = iter(map(int, sys.stdin.buffer.read().split()))
n, m = next(values), next(values)
adj = [[] for _ in range(n)]
for _ in range(m):
    u, v = next(values), next(values)
    adj[u].append(v)
    adj[v].append(u)
seen = set()
components = 0
for start in range(n):
    if start in seen:
        continue
    components += 1
    seen.add(start)
    stack = [start]
    while stack:
        for v in adj[stack.pop()]:
            if v not in seen:
                seen.add(v)
                stack.append(v)
print(components)
