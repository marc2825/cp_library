"""Matrices, reachable grids, sparse graphs and valid stateful query streams."""
from bisect import bisect_right
from collections import deque
import heapq
import math

from .generators import (_bounds, _size, random_array, random_array_with_sum,
                         random_interval, sample_indices)
from .random_data import Graph, tree

__all__ = ["random_matrix", "random_grid", "random_tree", "random_graph", "random_dag",
           "random_bipartite_graph", "weighted_edges", "parent_array", "adjacency_list",
           "random_intervals", "random_queries", "stateful_queries"]


def random_matrix(rng, height, width, low=0, high=1, *, mode="uniform", ones=None, row_sums=None):
    _size(height)
    _size(width)
    _bounds(low, high)
    if ones is not None:
        if mode != "uniform" or row_sums is not None or (low, high) != (0, 1):
            raise ValueError("ones requires an otherwise unconstrained binary matrix")
        selected = set(sample_indices(rng, height * width, ones))
        return [[int(i * width + j in selected) for j in range(width)] for i in range(height)]
    if row_sums is not None:
        if len(row_sums) != height or mode != "uniform":
            raise ValueError("row_sums requires one total per row, mode=uniform")
        return [random_array_with_sum(rng, width, low, high, s) for s in row_sums]
    if mode in {"symmetric", "identity", "diagonal", "upper", "lower", "permutation"}:
        if height != width:
            raise ValueError("structured matrices must be square")
        if mode != "symmetric" and not low <= 0 <= high:
            raise ValueError("structured matrix needs zero in value range")
        if mode in {"identity", "permutation"} and not low <= 1 <= high:
            raise ValueError("identity/permutation matrix needs one in value range")
        a = [[0] * width for _ in range(height)]
        if mode == "permutation":
            p = sample_indices(rng, width, width)
            for i, j in enumerate(p):
                a[i][j] = 1
            return a
        for i in range(height):
            for j in range(width):
                if mode == "identity":
                    a[i][j] = int(i == j)
                elif mode == "symmetric":
                    a[i][j] = a[j][i] if j < i else rng.integer(low, high)
                elif mode == "diagonal" and i == j or mode == "upper" and i <= j or mode == "lower" and i >= j:
                    a[i][j] = rng.integer(low, high)
        return a
    return [random_array(rng, width, low, high, mode=mode) for _ in range(height)]


def random_grid(rng, height, width, *, walls=0, connectivity="any", start_goal=True):
    """Exact wall count; S=(0,0), G=(h-1,w-1). Four-neighbour connectivity.

    reachable reserves a monotone path; unreachable walls off S. unique_path
    retains just a monotone path, so requires walls=h*w-(h+w-1).
    """
    if type(height) is not int or type(width) is not int or height < 1 or width < 1:
        raise ValueError("grid dimensions must be positive")
    _size(walls)
    if connectivity not in {"any", "reachable", "unreachable", "unique_path"}:
        raise ValueError("unknown connectivity")
    if not start_goal and connectivity != "any":
        raise ValueError("connectivity requires S/G")
    cells = height * width
    if start_goal and cells < 2:
        raise ValueError("S and G need distinct cells")
    reserved = {0, cells - 1} if start_goal else set()
    required = set()
    if connectivity in {"reachable", "unique_path"}:
        directions = [width] * (height - 1) + [1] * (width - 1)
        rng.shuffle(directions)
        pos = 0
        for step in directions:
            pos += step
            reserved.add(pos)
        if connectivity == "unique_path":
            if walls != cells - len(reserved):
                raise ValueError("unique_path requires all off-path cells to be walls")
    elif connectivity == "unreachable":
        required = ({width} if height > 1 else set()) | ({1} if width > 1 else set())
        if required & reserved:
            raise ValueError("adjacent endpoints cannot be separated")
    available = [i for i in range(cells) if i not in reserved and i not in required]
    if not len(required) <= walls <= len(required) + len(available):
        raise ValueError("wall count conflicts with connectivity/endpoints")
    blocked = required | {available[i] for i in sample_indices(rng, len(available), walls - len(required))}
    rows = [["#" if i * width + j in blocked else "." for j in range(width)] for i in range(height)]
    if start_goal:
        rows[0][0], rows[-1][-1] = "S", "G"
    return ["".join(row) for row in rows]


def random_tree(rng, n, *, mode="random"):
    """random reuses recursive tree(); uniform uses a uniform Prüfer sequence."""
    _size(n)
    if mode == "random":
        return tree(rng, n)
    if mode == "uniform":
        if n <= 1:
            return Graph(n, ())
        code = [rng.integer(0, n - 1) for _ in range(n - 2)]
        degrees = [1] * n
        for v in code:
            degrees[v] += 1
        leaves = [v for v, degree in enumerate(degrees) if degree == 1]
        heapq.heapify(leaves)
        edges = []
        for v in code:
            leaf = heapq.heappop(leaves)
            edges.append((leaf, v))
            degrees[v] -= 1
            if degrees[v] == 1:
                heapq.heappush(leaves, v)
        edges.append(tuple(leaves))
    elif mode in {"path", "star", "binary"}:
        labels = list(range(n))
        rng.shuffle(labels)
        edges = [(labels[i], labels[i - 1 if mode == "path" else 0 if mode == "star" else (i - 1) // 2])
                 for i in range(1, n)]
    else:
        raise ValueError("unknown tree mode")
    rng.shuffle(edges)
    return Graph(n, tuple(edges))


def _pair_rank(n, u, v):
    return u * (2 * n - u - 1) // 2 + v - u - 1


def _pair_unrank(n, rank):
    u = (2 * n - 1 - math.isqrt((2 * n - 1) ** 2 - 8 * rank)) // 2
    if u * (2 * n - u - 1) // 2 > rank:
        u -= 1
    return u, u + 1 + rank - u * (2 * n - u - 1) // 2


def _edge_sample(rng, capacity, m, required=(), multi=False):
    required = sorted(set(required))
    if m < len(required) or not multi and m > capacity or m and capacity == 0:
        raise ValueError("infeasible edge count")
    if multi:
        edges = required + [rng.integer(0, capacity - 1) for _ in range(m - len(required))]
    else:
        # Map the compact complement universe to original ranks in O(log m).
        holes = [value - i for i, value in enumerate(required)]
        edges = required + [k + bisect_right(holes, k)
                            for k in sample_indices(rng, capacity - len(required), m - len(required))]
    rng.shuffle(edges)
    return edges


def random_graph(rng, n, m, *, directed=False, connected=False, components=None, loops=False, multi=False):
    """O(n + m log(m+1)) time, O(n+m) space; never enumerates O(n²) candidates.

    directed connected means weak connectivity. Exact components is undirected;
    its construction uses one large block and singleton blocks.
    """
    _size(n)
    _size(m)
    if components is not None and (directed or connected):
        raise ValueError("components cannot combine with directed/connected")
    if components is not None and not (n == components == 0 or type(components) is int and 1 <= components <= n):
        raise ValueError("invalid component count")
    c = (1 if n else 0) if connected else components
    size = n if c is None else n - c + 1 if n else 0
    labels = list(range(n))
    rng.shuffle(labels)
    triangle = size * (size - 1) // 2
    if directed:
        capacity = n * n if loops else n * (n - 1)
        def encode(u, v):
            return u * n + v if loops else u * (n - 1) + v - (v > u)
        def decode(rank):
            u, v = divmod(rank, n if loops else n - 1)
            return u, v if loops else v + (v >= u)
    else:
        capacity = triangle + (n if loops else 0)
        def encode(u, v):
            return triangle + u if u == v else _pair_rank(size, min(u, v), max(u, v))
        def decode(rank):
            return (rank - triangle, rank - triangle) if rank >= triangle else _pair_unrank(size, rank)
    required = [encode(i, rng.below(i)) for i in range(1, size)] if c is not None else []
    ranks = _edge_sample(rng, capacity, m, required, multi)
    return Graph(n, tuple((labels[u], labels[v]) for u, v in map(decode, ranks)), directed)


def random_dag(rng, n, m):
    _size(n)
    _size(m)
    order = list(range(n))
    rng.shuffle(order)
    ranks = sample_indices(rng, n * (n - 1) // 2, m)
    return Graph(n, tuple((order[u], order[v]) for u, v in (_pair_unrank(n, k) for k in ranks)), True)


def random_bipartite_graph(rng, left, right, m):
    _size(left)
    _size(right)
    _size(m)
    ranks = sample_indices(rng, left * right, m)
    return Graph(left + right, tuple((k // right, left + k % right) for k in ranks))


def weighted_edges(rng, graph, low, high):
    _bounds(low, high)
    return [(u, v, rng.integer(low, high)) for u, v in graph.edges]


def adjacency_list(graph):
    adj = [[] for _ in range(graph.n)]
    for u, v in graph.edges:
        adj[u].append(v)
        if not graph.directed:
            adj[v].append(u)
    return adj


def parent_array(graph, root=0):
    """Root parent is -1; validate an undirected connected tree first."""
    if graph.directed or not 0 <= root < graph.n or len(graph.edges) != graph.n - 1:
        raise ValueError("parent array requires a nonempty tree and valid root")
    adj = adjacency_list(graph)
    parents = [-2] * graph.n
    parents[root] = -1
    queue = [root]
    for u in queue:
        for v in adj[u]:
            if parents[v] == -2:
                parents[v] = u
                queue.append(v)
    if len(queue) != graph.n:
        raise ValueError("graph is disconnected")
    return parents


def random_intervals(rng, count, low, high, *, half_open=False, length=None):
    _size(count)
    # Validate even for an empty result without consuming RNG.
    _bounds(low, high)
    if half_open and low == high or length is not None and (type(length) is not int or length < 1 or
            length - (not half_open) > high - low):
        raise ValueError("infeasible interval bounds/length")
    return [random_interval(rng, low, high, half_open=half_open, length=length) for _ in range(count)]


def random_queries(rng, count, n, *, kind="range", low=0, high=100):
    """0-indexed point/range query tuples; range endpoints are half-open."""
    _size(count)
    if type(n) is not int or n < 1:
        raise ValueError("query universe must be positive")
    _bounds(low, high)
    if kind not in {"point", "range", "union_find", "set", "multiset", "stack", "queue", "edges"}:
        raise ValueError("unknown query kind")
    if kind in {"set", "multiset", "stack", "queue", "edges"}:
        def available(state):
            if kind in {"stack", "queue"}:
                return ["push"] + (["pop"] if state else [])
            size = n * (n - 1) // 2 if kind == "edges" else n
            return (["insert"] if kind == "multiset" or len(state) < size else []) + (["erase"] if state else [])
        def transition(state, operation, engine):
            if operation == "push":
                value = engine.integer(low, high)
                state.append(value)
                return "push", value
            if operation == "pop":
                state.pop() if kind == "stack" else state.popleft()
                return ("pop",)
            if operation == "insert":
                size = n * (n - 1) // 2 if kind == "edges" else n
                if kind == "multiset":
                    value = engine.integer(0, n - 1)
                    state.append(value)
                else:
                    # Dense map with swap removal gives O(1) legal insert/erase.
                    value = state.choose_missing(engine, size)
                    state.add(value)
            else:
                value = state.remove_random(engine)
            return (operation, *_pair_unrank(n, value)) if kind == "edges" else (operation, value)
        state = deque() if kind in {"stack", "queue"} else _DynamicSet() if kind in {"set", "edges"} else _Bag()
        return stateful_queries(rng, count, state, available, transition)
    result = []
    for _ in range(count):
        if kind == "union_find":
            result.append((rng.below(2), rng.below(n), rng.below(n)))
        elif kind == "point":
            result.append((0, rng.below(n), rng.integer(low, high)) if rng.below(2) else (1, rng.below(n)))
        else:
            l, r = random_interval(rng, 0, n, half_open=True)
            result.append((0, l, r, rng.integer(low, high)) if rng.below(2) else (1, l, r))
    return result


class _Bag(list):
    def remove_random(self, rng):
        i = rng.below(len(self))
        self[i], self[-1] = self[-1], self[i]
        return self.pop()


class _DynamicSet:
    """A lazy permutation split into present prefix and absent suffix."""
    def __init__(self):
        self.positions, self.values, self.count = {}, {}, 0

    def __len__(self):
        return self.count

    def _swap(self, i, j):
        a, b = self.values.get(i, i), self.values.get(j, j)
        self.values[i], self.values[j] = b, a
        self.positions[a], self.positions[b] = j, i

    def choose_missing(self, rng, size):
        i = rng.integer(self.count, size - 1)
        return self.values.get(i, i)

    def add(self, value):
        self._swap(self.count, self.positions.get(value, value))
        self.count += 1

    def remove_random(self, rng):
        i = rng.below(self.count)
        value = self.values.get(i, i)
        self.count -= 1
        self._swap(i, self.count)
        return value


def stateful_queries(rng, count, state, available, transition):
    """Extension: available(state) -> operations; transition(state, op, rng) -> row.

    transition mutates the supplied state; callbacks own problem-specific validity.
    """
    _size(count)
    rows = []
    for _ in range(count):
        operations = available(state)
        if not operations:
            raise ValueError("no valid state transition")
        rows.append(transition(state, rng.choice(operations), rng))
    return rows
