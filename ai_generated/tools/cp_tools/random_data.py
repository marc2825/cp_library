"""Reproducible small/medium test data (SplitMix64, explicit sampling order)."""
from dataclasses import dataclass

MASK = (1 << 64) - 1


class Random:
    def __init__(self, seed: int):
        self.state = seed & MASK

    def next_u64(self) -> int:
        self.state = (self.state + 0x9E3779B97F4A7C15) & MASK
        z = self.state
        z = ((z ^ (z >> 30)) * 0xBF58476D1CE4E5B9) & MASK
        z = ((z ^ (z >> 27)) * 0x94D049BB133111EB) & MASK
        return z ^ (z >> 31)

    def below(self, n: int) -> int:
        if not 1 <= n <= 1 << 64:
            raise ValueError("below requires 1 <= n <= 2**64")
        limit = (1 << 64) - (1 << 64) % n
        while True:
            value = self.next_u64()
            if value < limit:
                return value % n

    def integer(self, low: int, high: int) -> int:
        return low + self.below(high - low + 1)

    def shuffle(self, values: list) -> None:
        for i in range(len(values) - 1, 0, -1):
            j = self.below(i + 1)
            values[i], values[j] = values[j], values[i]

    def sample(self, values, count: int) -> list:
        values = list(values)
        if not 0 <= count <= len(values):
            raise ValueError("invalid sample size")
        for i in range(count):
            j = i + self.below(len(values) - i)
            values[i], values[j] = values[j], values[i]
        return values[:count]


@dataclass(frozen=True)
class Graph:
    n: int
    edges: tuple[tuple[int, int], ...]
    directed: bool = False

    def text(self, one_based: bool = False) -> str:
        shift = int(one_based)
        return f"{self.n} {len(self.edges)}\n" + "".join(
            f"{u + shift} {v + shift}\n" for u, v in self.edges)


def tree(rng: Random, n: int) -> Graph:
    if n < 0:
        raise ValueError("negative vertex count")
    labels = list(range(n))
    rng.shuffle(labels)
    edges = [(labels[i], labels[rng.below(i)]) for i in range(1, n)]
    rng.shuffle(edges)
    return Graph(n, tuple(edges))


def forest(rng: Random, n: int, components: int) -> Graph:
    if not (n == components == 0 or 1 <= components <= n):
        raise ValueError("invalid component count")
    return Graph(n, tuple(rng.sample(tree(rng, n).edges, n - components)))


def undirected_graph(rng: Random, n: int, m: int, *,
                     components: int | None = None, multi: bool = False,
                     loops: bool = False) -> Graph:
    """With components specified, generate exactly that many components.

    A large block and singleton blocks make every feasible m attainable.
    Candidate enumeration uses O(n**2) memory. Not a uniform graph sampler.
    """
    if n < 0 or m < 0:
        raise ValueError("negative size")
    if components is not None and not (n == components == 0 or 1 <= components <= n):
        raise ValueError("invalid component count")
    labels = list(range(n))
    rng.shuffle(labels)
    groups = [labels] if components is None else (
        [labels[:n - components + 1]] + [[x] for x in labels[n - components + 1:]] if n else [])
    required = []
    if components is not None:
        for group in groups:
            required.extend(tuple(sorted((group[i], group[rng.below(i)])))
                            for i in range(1, len(group)))
    candidates = [tuple(sorted((group[i], group[j]))) for group in groups
                  for i in range(len(group)) for j in range(i if loops else i + 1, len(group))]
    if m < len(required) or (not multi and m > len(candidates)) or (m and not candidates):
        raise ValueError("infeasible edge count")
    if multi:
        edges = required + [candidates[rng.below(len(candidates))] for _ in range(m - len(required))]
    else:
        used = set(required)
        edges = required + rng.sample([e for e in candidates if e not in used], m - len(required))
    rng.shuffle(edges)
    return Graph(n, tuple(edges))


def dag(rng: Random, n: int, m: int) -> Graph:
    if n < 0 or not 0 <= m <= n * (n - 1) // 2:
        raise ValueError("infeasible DAG size")
    order = list(range(n))
    rng.shuffle(order)
    edges = rng.sample(((order[i], order[j]) for i in range(n) for j in range(i + 1, n)), m)
    return Graph(n, tuple(edges), True)


def simple_polygon(rng: Random, n: int, bound: int = 100) -> list[tuple[int, int]]:
    """A strictly convex (hence simple) integer polygon on y=x*x, CCW.

    Coordinates satisfy |x| <= bound, 0 <= y <= bound**2.
    """
    if bound < 1 or not 3 <= n <= 2 * bound + 1:
        raise ValueError("need 3 <= n <= 2*bound+1")
    xs = sorted(rng.sample(range(-bound, bound + 1), n))
    points = [(x, x * x) for x in xs]
    start = rng.below(n)
    return points[start:] + points[:start]
