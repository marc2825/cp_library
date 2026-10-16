#pragma once

// Union-Find: union by size + path compression。成分数は O(1) で取得。
// 詳細: docs/union_find/disjoint_set_union.md
#include <algorithm>
#include <cassert>
#include <vector>

namespace cp {

class DisjointSetUnion {
public:
    // 旧 API との互換用。直接変更せず、問い合わせには leader/size を使う。
    std::vector<int> par, siz;

    explicit DisjointSetUnion(int n = 0)
        : par(checked_size(n), -1), siz(n, 1), component_count_(n) {}

    int vertex_count() const { return static_cast<int>(par.size()); }
    int component_count() const { return component_count_; }

    int leader(int v) {
        assert(0 <= v && v < vertex_count());
        int representative = v;
        while (par[representative] != -1) representative = par[representative];
        while (v != representative) {
            int next = par[v];
            par[v] = representative;
            v = next;
        }
        return representative;
    }

    bool same(int u, int v) { return leader(u) == leader(v); }
    int size(int v) { return siz[leader(v)]; }

    // 異なる成分を併合したときだけ true。ACL の代表元を返す merge とは異なる。
    bool merge(int u, int v) {
        u = leader(u);
        v = leader(v);
        if (u == v) return false;
        if (siz[u] < siz[v]) std::swap(u, v);
        par[v] = u;
        siz[u] += siz[v];
        --component_count_;
        return true;
    }

    // 成分内は頂点番号順、成分同士は最小の頂点番号順。
    std::vector<std::vector<int>> groups() {
        std::vector<int> group_index(vertex_count(), -1);
        std::vector<std::vector<int>> result;
        result.reserve(component_count_);
        for (int v = 0; v < vertex_count(); ++v) {
            int representative = leader(v);
            if (group_index[representative] == -1) {
                group_index[representative] = static_cast<int>(result.size());
                result.push_back({});
            }
            result[group_index[representative]].push_back(v);
        }
        return result;
    }

    void reset(int n) {
        assert(n >= 0);
        par.assign(n, -1);
        siz.assign(n, 1);
        component_count_ = n;
    }

    // 従来のライブラリで使っていた名前。
    int root(int v) { return leader(v); }
    bool issame(int u, int v) { return same(u, v); }
    bool unite(int u, int v) { return merge(u, v); }

private:
    int component_count_;
    static int checked_size(int n) {
        assert(n >= 0);
        return n;
    }
};

} // namespace cp
