#pragma once
#include <algorithm>
#include <cstddef>
#include <numeric>
#include <stdexcept>
#include <utility>
#include <vector>

namespace cp {
struct GeneralMatchingResult {
    int cardinality = 0;
    std::vector<int> mate; // -1 means unmatched.
    std::vector<std::pair<int, int>> pairs; // Smaller endpoint first; ascending first endpoint.
};
// Unweighted maximum cardinality matching in an undirected graph, including odd cycles.
class GeneralMatching {
    std::vector<std::vector<int>> adjacency_;
public:
    explicit GeneralMatching(int n = 0) {
        if (n < 0) throw std::invalid_argument("negative matching size");
        adjacency_.resize(n);
    }
    int size() const { return static_cast<int>(adjacency_.size()); }
    void add_edge(int u, int v) {
        if (u < 0 || v < 0 || u >= size() || v >= size()) throw std::out_of_range("matching vertex");
        if (u != v) { adjacency_[u].push_back(v); adjacency_[v].push_back(u); }
    }
    GeneralMatchingResult maximum_matching() const {
        const int n = size();
        std::vector<std::vector<int>> graph(n);
        std::vector<int> stamp(n, -1);
        for (int v = 0; v < n; ++v) for (int u : adjacency_[v])
            if (stamp[u] != v) { stamp[u] = v; graph[v].push_back(u); }
        GeneralMatchingResult result;
        auto& mate = result.mate; mate.assign(n, -1);
        std::vector<int> parent(n), base(n), queue;
        std::vector<bool> outer(n), blossom(n), ancestor(n);
        queue.reserve(n);
        const auto lca = [&](int a, int b) {
            std::fill(ancestor.begin(), ancestor.end(), false);
            while (true) {
                a = base[a]; ancestor[a] = true;
                if (mate[a] == -1) break;
                a = parent[mate[a]];
            }
            while (true) {
                b = base[b];
                if (ancestor[b]) return b;
                b = parent[mate[b]];
            }
        };
        const auto mark_path = [&](int v, int common, int child) {
            while (base[v] != common) {
                blossom[base[v]] = blossom[base[mate[v]]] = true;
                parent[v] = child; child = mate[v]; v = parent[mate[v]];
            }
        };
        const auto search = [&](int root) {
            std::fill(parent.begin(), parent.end(), -1);
            std::iota(base.begin(), base.end(), 0);
            std::fill(outer.begin(), outer.end(), false);
            queue.assign(1, root); outer[root] = true;
            for (std::size_t head = 0; head < queue.size(); ++head) {
                const int v = queue[head];
                for (int u : graph[v]) {
                    if (base[v] == base[u] || mate[v] == u) continue;
                    if (u == root || (mate[u] != -1 && parent[mate[u]] != -1)) {
                        const int common = lca(v, u);
                        std::fill(blossom.begin(), blossom.end(), false);
                        mark_path(v, common, u); mark_path(u, common, v);
                        for (int i = 0; i < n; ++i) if (blossom[base[i]]) {
                            base[i] = common;
                            if (!outer[i]) { outer[i] = true; queue.push_back(i); }
                        }
                    } else if (parent[u] == -1) {
                        parent[u] = v;
                        if (mate[u] == -1) return u;
                        u = mate[u]; outer[u] = true; queue.push_back(u);
                    }
                }
            }
            return -1;
        };
        for (int root = 0; root < n; ++root) if (mate[root] == -1) {
            int v = search(root);
            if (v == -1) continue;
            ++result.cardinality;
            while (v != -1) {
                const int p = parent[v], next = p == -1 ? -1 : mate[p];
                mate[v] = p;
                if (p != -1) mate[p] = v;
                v = next;
            }
        }
        for (int v = 0; v < n; ++v) if (mate[v] > v) result.pairs.emplace_back(v, mate[v]);
        return result;
    }
};
} // namespace cp
