#pragma once

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <utility>
#include <vector>
#include "legacy_tree.hpp"
#include "../../graph/all.hpp"
#include "../../data_structure/UnionFind.hpp"

template <class T = long long>
class Graph : public cp_legacy_detail::Storage<T> {
    using Base = cp_legacy_detail::Storage<T>;
    using Snapshot = cp_legacy_detail::Snapshot<T>;
public:
    using Base::G;
    using Base::E;
    const T inf = cp::distance_inf<T>();
    std::vector<int> In_Deg, Out_Deg;
    std::vector<std::vector<T>> AdjMat;
    std::vector<std::vector<int>> idxMat;
    LowLink<T> LL;

    explicit Graph(int n = 0) : Base(n) {}
    void input(int m, int offset = -1, bool weighted = false, bool directed = false) {
        for (int i = 0; i < m; ++i) {
            int from, to; T cost = T(1); std::cin >> from >> to;
            if (weighted) std::cin >> cost;
            if (directed) this->add_directed_edge(from + offset, to + offset, cost);
            else this->add_edge(from + offset, to + offset, cost);
        }
    }
    void input_adjmat() {
        int n = this->size();
        AdjMat.assign(n, std::vector<T>(n, inf)); idxMat.assign(n, std::vector<int>(n, -1));
        for (auto& row : AdjMat) for (auto& value : row) std::cin >> value;
        matrix_revision_ = this->revision_;
    }
    void Calc_Deg() {
        In_Deg.assign(this->size(), 0); Out_Deg.assign(this->size(), 0);
        for (int v = 0; v < this->size(); ++v)
            for (const auto& e : G[v]) { ++Out_Deg[v]; ++In_Deg[e.to]; }
    }
    void resize(int n) { Base::resize(n); AdjMat.clear(); idxMat.clear(); LL = {}; Calc_Deg(); }

    ShortestPath<T> dijkstra(int start = 0) const {
        auto data = this->snapshot(); return convert(cp::dijkstra(data.graph, start), data);
    }
    ShortestPath<T> bellman_ford(int start = 0) const {
        auto data = this->snapshot(); return convert(cp::bellman_ford(data.graph, start), data);
    }
    ShortestPath<T> bfs(int start = 0) const {
        auto data = this->snapshot(); return convert(cp::bfs(data.graph, start), data);
    }
    ShortestPath<T> bfs_01(int start = 0) const {
        auto data = this->snapshot(); return convert(cp::zero_one_bfs(data.graph, start), data);
    }
    ShortestPath<T> dfs(int start = 0) const {
        auto data = this->snapshot(); return convert(cp::depth_first_search(data.graph, start), data);
    }
    ShortestPath<T> SingleSource_ShortestPath(int start = 0) const {
        for (const auto& list : G) for (const auto& e : list)
            if (e.cost < T(0)) return bellman_ford(start);
        return dijkstra(start);
    }
    std::vector<ShortestPath<T>> floyd_warshall() {
        auto data = matrix_snapshot();
        std::vector<ShortestPath<T>> result;
        for (const auto& row : cp::floyd_warshall(data.graph)) result.push_back(convert(row, data));
        return result;
    }
    std::vector<ShortestPath<T>> Allpair_ShortestPath() {
        int n = this->size();
        for (const auto& list : G) for (const auto& e : list) if (e.cost < T(0)) return floyd_warshall();
        if (!AdjMat.empty() && matrix_revision_ == this->revision_) return floyd_warshall();
        if (static_cast<long double>(n) * n <= static_cast<long double>(E.size()) * std::log2(std::max(2, n)))
            return floyd_warshall();
        std::vector<ShortestPath<T>> result;
        for (int s = 0; s < n; ++s) result.push_back(dijkstra(s));
        return result;
    }
    std::vector<int> bipartite_grouping() const {
        auto result = cp::bipartite_coloring(this->snapshot(true).graph);
        return result ? *result : std::vector<int>{};
    }
    bool is_bipartite() const { return cp::bipartite_coloring(this->snapshot(true).graph).has_value(); }
    std::vector<int> topological_sort() const {
        auto result = cp::topological_sort(this->snapshot(false, true).graph);
        return result ? *result : std::vector<int>{};
    }
    bool is_DAG() const { return cp::topological_sort(this->snapshot(false, true).graph).has_value(); }
    std::pair<T, Edges<T>> kruskal() const {
        auto data = this->snapshot(); auto result = cp::kruskal(data.graph);
        return {result.cost, convert_edges(result.edges, data)};
    }
    std::pair<T, Edges<T>> prim() const {
        auto data = this->snapshot(); auto result = cp::prim(data.graph);
        return {result.cost, convert_edges(result.edges, data)};
    }
    Cycle find_cycle(int v = 0, bool withv = false) const {
        auto data = this->snapshot(); auto result = cp::find_cycle(data.graph, withv ? v : -1);
        if (!result) return {};
        Cycle cycle; cycle.len = result->size(); cycle.Vid = result->vertices;
        for (int idx : result->edges) cycle.Eid.push_back(data.edge_ids[idx]);
        return cycle;
    }
    std::vector<std::vector<int>> components() const {
        auto data = this->snapshot(true); unionfind dsu(this->size());
        for (const auto& e : data.graph.edges()) dsu.unite(e.from, e.to);
        std::vector<std::vector<int>> result(this->size());
        for (int v = 0; v < this->size(); ++v) result[dsu.root(v)].push_back(v);
        return result;
    }
    std::vector<int> Articulation() { build_low_link(); return LL.articulation; }
    std::vector<std::pair<Edge<T>, long long>> Bridge() { build_low_link(); return LL.bridge; }
    std::pair<std::vector<int>, Tree<T>> TwoEdgeConnectedComponents() const {
        auto data = this->snapshot(); auto result = cp::two_edge_connected_components(data.graph);
        Tree<T> forest(result.forest.size());
        for (const auto& e : result.forest.edges())
            forest.add_edge(e.from, e.to, e.cost, data.edge_ids[result.original_edge[e.idx]]);
        return {std::move(result.component), std::move(forest)};
    }
    std::vector<std::vector<int>> BiConnectedComponents() const {
        return cp::biconnected_components(this->snapshot().graph);
    }
    Tree<T> BlockCutTree() const {
        auto result = cp::block_cut_forest(this->snapshot().graph);
        Tree<T> forest(result.forest.size());
        for (const auto& e : result.forest.edges()) forest.add_edge(e.from, e.to, T(1));
        return forest;
    }

private:
    std::size_t matrix_revision_ = std::numeric_limits<std::size_t>::max();
    template <class U>
    static ShortestPath<T> convert(const cp::ShortestPathResult<U>& result, const Snapshot& data) {
        ShortestPath<T> legacy(static_cast<int>(result.dist.size()));
        legacy.prev = result.parent;
        for (int v = 0; v < static_cast<int>(result.dist.size()); ++v) {
            legacy.dist[v] = !result.reachable(v) ? cp::distance_inf<T>()
                : result.negative_cycle[v] ? -cp::distance_inf<T>() : static_cast<T>(result.dist[v]);
            if (result.parent_edge[v] != -1) legacy.Eid[v] = data.edge_ids[result.parent_edge[v]];
        }
        return legacy;
    }
    static Edges<T> convert_edges(const std::vector<cp::Edge<T>>& edges, const Snapshot& data) {
        Edges<T> result;
        for (const auto& e : edges) result.emplace_back(e.from, e.to, e.cost, data.edge_ids[e.idx]);
        return result;
    }
    Snapshot matrix_snapshot() {
        int n = this->size();
        if (matrix_revision_ != this->revision_ || static_cast<int>(AdjMat.size()) != n) {
            AdjMat.assign(n, std::vector<T>(n, inf)); idxMat.assign(n, std::vector<int>(n, -1));
            for (int v = 0; v < n; ++v) {
                AdjMat[v][v] = T(0);
                for (const auto& e : G[v]) if (e.cost < AdjMat[v][e.to]) {
                    AdjMat[v][e.to] = e.cost; idxMat[v][e.to] = e.idx;
                }
            }
            matrix_revision_ = this->revision_;
        }
        Snapshot result{cp::Graph<T>(n, cp::GraphKind::Directed), {}};
        for (int from = 0; from < n; ++from) for (int to = 0; to < n; ++to) {
            if (AdjMat[from][to] == inf) continue;
            result.graph.add_edge(from, to, AdjMat[from][to]); result.edge_ids.push_back(idxMat[from][to]);
        }
        return result;
    }
    void build_low_link() {
        auto data = this->snapshot(); auto result = cp::low_link(data.graph);
        LL.ord = result.ord; LL.low = result.low; LL.articulation = result.articulation; LL.bridge.clear();
        for (int idx : result.bridge_edges) {
            auto e = data.graph.edges()[idx];
            int child = result.parent_edge[e.to] == idx ? e.to : e.from;
            LL.bridge.emplace_back(Edge<T>(result.parent[child], child, e.cost, data.edge_ids[idx]),
                                   result.subtree_size[child]);
        }
    }
};
