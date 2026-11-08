#pragma once
#include <algorithm>
#include <cassert>
#include <cstddef>
#include <limits>
#include <queue>
#include <type_traits>
#include <vector>

namespace cp {

// Integral capacities; iterative Dinic. See docs/graph_flow/dinic.md.
template <class Cap = long long>
class Dinic {
    static_assert(std::is_integral<Cap>::value && !std::is_same<Cap, bool>::value,
                  "Dinic requires an integral capacity type other than bool");
    struct Arc { int to; Cap residual; };
    std::vector<std::vector<int>> adjacency_;
    std::vector<Arc> arcs_;
    int source_ = -1, sink_ = -1;

public:
    struct Edge { int from, to; Cap capacity, flow; };

    explicit Dinic(int n = 0) {
        assert(n >= 0);
        adjacency_.resize(n);
    }
    int size() const { return static_cast<int>(adjacency_.size()); }
    int edge_count() const { return static_cast<int>(arcs_.size() / 2); }

    int add_edge(int from, int to, Cap capacity) {
        assert(0 <= from && from < size() && 0 <= to && to < size());
        assert(Cap{} <= capacity);
        assert(arcs_.size() <= static_cast<std::size_t>(std::numeric_limits<int>::max() - 2));
        int id = edge_count(), arc = static_cast<int>(arcs_.size());
        arcs_.push_back({to, capacity});
        arcs_.push_back({from, Cap{}});
        adjacency_[from].push_back(arc);
        adjacency_[to].push_back(arc ^ 1);
        return id;
    }

    Edge get_edge(int id) const {
        assert(0 <= id && id < edge_count());
        const auto &forward = arcs_[2 * id], &reverse = arcs_[2 * id + 1];
        return {reverse.to, forward.to, static_cast<Cap>(forward.residual + reverse.residual), reverse.residual};
    }
    std::vector<Edge> edges() const {
        std::vector<Edge> result;
        result.reserve(edge_count());
        for (int id = 0; id < edge_count(); ++id) result.push_back(get_edge(id));
        return result;
    }

    // Returns the additional flow. Repeated calls must use the same s,t until reset_flow().
    Cap max_flow(int s, int t, Cap limit = std::numeric_limits<Cap>::max()) {
        assert(0 <= s && s < size() && 0 <= t && t < size() && s != t);
        assert(Cap{} <= limit);
        assert(source_ == -1 || (source_ == s && sink_ == t));
        source_ = s; sink_ = t;
        Cap total{};
        std::vector<int> level(size()), next(size()), vertices, path;
        std::vector<Cap> bottleneck;
        vertices.reserve(size()); path.reserve(size()); bottleneck.reserve(size());
        while (total < limit) {
            std::fill(level.begin(), level.end(), -1);
            std::queue<int> queue;
            level[s] = 0; queue.push(s);
            while (!queue.empty()) {
                int v = queue.front(); queue.pop();
                for (int id : adjacency_[v]) {
                    const auto &e = arcs_[id];
                    if (e.residual > Cap{} && level[e.to] == -1) {
                        level[e.to] = level[v] + 1;
                        queue.push(e.to);
                    }
                }
            }
            if (level[t] == -1) break;
            std::fill(next.begin(), next.end(), 0);
            while (total < limit) {
                vertices.assign(1, s); path.clear(); bottleneck.assign(1, limit - total);
                Cap pushed{};
                while (!vertices.empty()) {
                    int v = vertices.back();
                    if (v == t) {
                        pushed = bottleneck.back();
                        for (int id : path) {
                            arcs_[id].residual -= pushed;
                            arcs_[id ^ 1].residual += pushed;
                        }
                        break;
                    }
                    int &cursor = next[v];
                    while (cursor < static_cast<int>(adjacency_[v].size())) {
                        const auto &e = arcs_[adjacency_[v][cursor]];
                        if (e.residual > Cap{} && level[e.to] == level[v] + 1) break;
                        ++cursor;
                    }
                    if (cursor == static_cast<int>(adjacency_[v].size())) {
                        level[v] = -1;
                        vertices.pop_back(); bottleneck.pop_back();
                        if (!path.empty()) {
                            path.pop_back();
                            ++next[vertices.back()];
                        }
                    } else {
                        int id = adjacency_[v][cursor];
                        path.push_back(id); vertices.push_back(arcs_[id].to);
                        bottleneck.push_back(std::min(bottleneck.back(), arcs_[id].residual));
                    }
                }
                if (pushed == Cap{}) break;
                total += pushed;
            }
        }
        return total;
    }

    // Residual reachability; a minimum cut only after exhausting s->t flow.
    std::vector<bool> min_cut(int s) const {
        assert(0 <= s && s < size());
        std::vector<bool> side(size(), false);
        std::queue<int> queue;
        side[s] = true; queue.push(s);
        while (!queue.empty()) {
            int v = queue.front(); queue.pop();
            for (int id : adjacency_[v]) {
                const auto &e = arcs_[id];
                if (e.residual > Cap{} && !side[e.to]) {
                    side[e.to] = true; queue.push(e.to);
                }
            }
        }
        return side;
    }

    void reset_flow() {
        for (int id = 0; id < static_cast<int>(arcs_.size()); id += 2) {
            arcs_[id].residual += arcs_[id ^ 1].residual;
            arcs_[id ^ 1].residual = Cap{};
        }
        source_ = sink_ = -1;
    }
};

} // namespace cp
