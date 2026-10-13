#pragma once
#include <algorithm>
#include <cassert>
#include <vector>

namespace cp {
class DisjointSetUnion {
public:
    std::vector<int> par, siz;
    explicit DisjointSetUnion(int n) : par(n, -1), siz(n, 1) {}
    int root(int v) {
        assert(0 <= v && v < static_cast<int>(par.size()));
        if (par[v] == -1) return v;
        return par[v] = root(par[v]);
    }
    bool same(int u, int v) { return root(u) == root(v); }
    bool merge(int u, int v) {
        u = root(u); v = root(v);
        if (u == v) return false;
        if (siz[u] < siz[v]) std::swap(u, v);
        par[v] = u; siz[u] += siz[v];
        return true;
    }
    int size(int v) { return siz[root(v)]; }
    bool issame(int u, int v) { return same(u, v); }
    bool unite(int u, int v) { return merge(u, v); }
};
} // namespace cp
