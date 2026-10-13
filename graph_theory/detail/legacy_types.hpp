#pragma once

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <utility>
#include <vector>
#include "../../graph/graph.hpp"

// 旧ヘッダ間で型を共有する。新しいコードでは cp::Graph / cp::Edge を使う。
template <class T = long long>
struct Edge {
    int from = -1, to = -1;
    T cost = T(1);
    int idx = -1;
    Edge() = default;
    // 旧単独 Dijkstra の隣接リスト用。
    Edge(int to_, T cost_ = T(1)) : to(to_), cost(cost_) {}
    // 旧 Graph/Tree 用。2引数は上の隣接リスト形式なので、cost を明示する。
    Edge(int from_, int to_, T cost_, int idx_ = -1)
        : from(from_), to(to_), cost(cost_), idx(idx_) {}
};
template <class T = long long> using Edges = std::vector<Edge<T>>;
template <class T = long long> using graph = std::vector<Edges<T>>;

template <class T = long long>
struct ShortestPath {
    std::vector<T> dist;
    std::vector<int> prev, Eid;
    ShortestPath() = default;
    explicit ShortestPath(int n) : dist(n), prev(n, -1), Eid(n, -1) {}
    ShortestPath(std::vector<T> distances, std::vector<int> parents, std::vector<int> edges)
        : dist(std::move(distances)), prev(std::move(parents)), Eid(std::move(edges)) {}
};
struct Cycle {
    int len = 0;
    std::vector<int> Vid, Eid;
    std::size_t size() const { return static_cast<std::size_t>(len); }
};
template <class T = long long>
struct LowLink {
    std::vector<int> ord, low, articulation;
    std::vector<std::pair<Edge<T>, long long>> bridge;
};

namespace cp_legacy_detail {

template <class T>
struct Snapshot {
    cp::Graph<T> graph;
    std::vector<int> edge_ids;
};

// 互換層だけで旧公開データを保持する。E は無向辺も1回ずつ。
template <class T>
class Storage {
public:
    ::graph<T> G;
    ::Edges<T> E;

    explicit Storage(int n = 0) : G(checked_size(n)) {}
    int size() const { return static_cast<int>(G.size()); }
    std::size_t e_size() const { return E.size(); }
    void add_edge(int from, int to, T cost = T(1), int idx = -1) {
        add(from, to, cost, idx, false);
    }
    void add_directed_edge(int from, int to, T cost = T(1), int idx = -1) {
        add(from, to, cost, idx, true);
    }
    void resize(int n) {
        assert(n >= 0);
        auto old = std::move(E); auto directions = std::move(directed_);
        G.assign(n, {}); E.clear(); directed_.clear();
        for (std::size_t i = 0; i < old.size(); ++i) {
            const auto& e = old[i];
            if (e.from < n && e.to < n) add(e.from, e.to, e.cost, e.idx, directions[i]);
        }
        ++revision_;
    }
    std::vector<Edge<T>>& operator[](int v) {
        assert(0 <= v && v < size()); ++revision_; return G[v];
    }
    const std::vector<Edge<T>>& operator[](int v) const {
        assert(0 <= v && v < size()); return G[v];
    }

protected:
    std::size_t revision_ = 0;
    Snapshot<T> snapshot(bool force_undirected = false, bool force_directed = false) const {
        bool directed = force_directed;
        if (!force_undirected && !force_directed) {
            directed = std::find(directed_.begin(), directed_.end(), true) != directed_.end();
            if (E.empty()) for (const auto& list : G) if (!list.empty()) directed = true;
        }
        Snapshot<T> result{cp::Graph<T>(size(), directed ? cp::GraphKind::Directed : cp::GraphKind::Undirected), {}};
        auto append = [&](int from, int to, T cost, int idx) {
            int generated = result.graph.add_edge(from, to, cost);
            result.edge_ids.push_back(idx == -1 ? generated : idx);
        };
        if (directed || E.empty()) {
            for (int v = 0; v < size(); ++v)
                for (const auto& e : G[v]) append(v, e.to, e.cost, e.idx);
        } else {
            for (const auto& e : E) append(e.from, e.to, e.cost, e.idx);
        }
        return result;
    }

private:
    std::vector<bool> directed_;
    static int checked_size(int n) { assert(n >= 0); return n; }
    void add(int from, int to, T cost, int idx, bool directed) {
        assert(0 <= from && from < size() && 0 <= to && to < size());
        if (idx == -1) idx = static_cast<int>(E.size());
        assert(idx >= 0);
        E.emplace_back(from, to, cost, idx); directed_.push_back(directed);
        G[from].emplace_back(from, to, cost, idx);
        if (!directed) G[to].emplace_back(to, from, cost, idx);
        ++revision_;
    }
};

} // namespace cp_legacy_detail
