#pragma once

#include <cassert>
#include <vector>

namespace cp {

enum class GraphKind { Directed, Undirected };

template <class T = long long>
struct Edge {
    int from = -1, to = -1;
    T cost = T(1);
    int idx = -1;
};

// 元の辺は edges() に1回だけ保存する。無向辺の両方向は同じ idx を持つ。
// 辺番号は追加順の [0, edge_count())。構築後のアルゴリズムはグラフを変更しない。
template <class T = long long>
class Graph {
public:
    using cost_type = T;

    explicit Graph(int n = 0, GraphKind kind = GraphKind::Undirected)
        : adjacency_(checked_size(n)), kind_(kind) {}

    int size() const { return static_cast<int>(adjacency_.size()); }
    int edge_count() const { return static_cast<int>(edges_.size()); }
    GraphKind kind() const { return kind_; }
    bool directed() const { return kind_ == GraphKind::Directed; }

    int add_edge(int from, int to, T cost = T(1)) {
        assert(0 <= from && from < size());
        assert(0 <= to && to < size());
        int idx = edge_count();
        Edge<T> edge{from, to, cost, idx};
        edges_.push_back(edge);
        adjacency_[from].push_back(edge);
        if (!directed()) adjacency_[to].push_back({to, from, cost, idx});
        return idx;
    }

    const std::vector<Edge<T>>& operator[](int v) const {
        assert(0 <= v && v < size());
        return adjacency_[v];
    }
    const std::vector<Edge<T>>& edges() const { return edges_; }

private:
    static int checked_size(int n) { assert(n >= 0); return n; }
    std::vector<std::vector<Edge<T>>> adjacency_;
    std::vector<Edge<T>> edges_;
    GraphKind kind_;
};

} // namespace cp
