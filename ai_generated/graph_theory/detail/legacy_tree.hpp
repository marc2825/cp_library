#pragma once

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <iostream>
#include <optional>
#include <utility>
#include <vector>
#include "legacy_types.hpp"
#include "../../tree/all.hpp"

// 旧 Tree の入口。新規コードでは cp::Graph と tree/ の各クラスを使う。
template <class T = long long>
class Tree : public cp_legacy_detail::Storage<T> {
    using Base = cp_legacy_detail::Storage<T>;
public:
    using Base::G;
    using Base::E;
    using Base::size;
    int root;
    std::vector<std::vector<int>> par_dbl;
    std::vector<int> depth, sz;
    std::vector<T> dist_weighted;
    std::vector<int> ET_V, ET_Vin, ET_Vout, ET_E, ET_Ein, ET_Eout;
    int ET_root_edge = -1;
    std::vector<int> HLD, HLD_pos, HLD_head;
    std::vector<bool> CD_removed;
    std::vector<int> CD_sz, CD_par;

    explicit Tree(int n = 0, int root_ = 0) : Base(n), root(root_) {}

    void input(int offset = -1, bool weighted = false, bool directed = false, int m = -1) {
        if (m == -1) m = std::max(0, Base::size() - 1);
        for (int i = 0; i < m; ++i) {
            int from, to; T cost = T(1); std::cin >> from >> to;
            if (weighted) std::cin >> cost;
            if (directed) this->add_directed_edge(from + offset, to + offset, cost);
            else this->add_edge(from + offset, to + offset, cost);
        }
    }
    void input_par(int offset = -1, bool weighted = false) {
        for (int v = 0; v < Base::size(); ++v) {
            if (v == root) continue;
            int p; T cost = T(1); std::cin >> p;
            if (weighted) std::cin >> cost;
            this->add_directed_edge(p + offset, v, cost);
        }
    }
    void build() {
        if (lca_ && built_revision_ == this->revision_ && built_root_ == root) return;
        clear_derived();
        auto data = this->snapshot(true);
        lca_.emplace(data.graph, root);
        built_revision_ = this->revision_; built_root_ = root;
        depth = lca_->tree.depth; sz = lca_->tree.subtree_size;
        dist_weighted = lca_->tree.distance_from_root;
        par_dbl.assign(Base::size(), std::vector<int>(lca_->up.size()));
        for (int v = 0; v < Base::size(); ++v)
            for (std::size_t k = 0; k < lca_->up.size(); ++k) {
                int ancestor = lca_->up[k][v];
                par_dbl[v][k] = ancestor == -1 ? lca_->tree.roots[lca_->tree.component[v]] : ancestor;
            }
    }
    void rebuild(int root_) { root = root_; lca_.reset(); build(); }
    int dep(int v) { build(); return lca_->tree.depth.at(v); }
    int par(int v) {
        build(); int p = lca_->tree.parent.at(v); return p == -1 ? v : p;
    }
    int size(int v) { build(); return lca_->tree.subtree_size.at(v); }
    int kth_ancestor(int v, int k) { build(); return lca_->kth_ancestor(v, k); }
    int LCA(int u, int v) { build(); return lca_->lca(u, v); }
    T dist(int u, int v) { build(); return lca_->distance(u, v); }
    bool is_on_path(int s, int t, int v) { build(); return lca_->is_on_path(s, t, v); }
    std::vector<int> path(int s, int t) { build(); return lca_->vertex_path(s, t); }
    int nxt(int s, int t) { build(); return s == t ? -1 : lca_->jump(s, t, 1); }
    std::pair<T, std::pair<int, int>> diameter() {
        auto result = cp::tree_diameter(this->snapshot(true).graph);
        return {result.length, result.endpoints};
    }

    void EulerTour_build() {
        build();
        auto data = this->snapshot(true);
        cp::EulerTour<T> tour(data.graph, root);
        ET_V = std::move(tour.walk); ET_Vin = std::move(tour.first); ET_Vout = std::move(tour.last);
        ET_root_edge = std::max(0, Base::size() - 1);
        for (int idx : data.edge_ids) ET_root_edge = std::max(ET_root_edge, idx + 1);
        ET_Ein.assign(ET_root_edge + 1, -1); ET_Eout.assign(ET_root_edge + 1, -1);
        ET_E.assign(1, ET_root_edge); ET_Ein[ET_root_edge] = 0;
        for (int idx : tour.edge_walk) ET_E.push_back(data.edge_ids[idx]);
        for (int i = 0; i < data.graph.edge_count(); ++i) {
            ET_Ein[data.edge_ids[i]] = tour.edge_in[i] + 1;
            ET_Eout[data.edge_ids[i]] = tour.edge_out[i] + 1;
        }
        ET_E.push_back(ET_root_edge);
        ET_Eout[ET_root_edge] = static_cast<int>(ET_E.size());
    }
    void HLD_build() {
        build(); hld_.emplace(this->snapshot(true).graph, root);
        HLD = hld_->vertex_at; HLD_pos = hld_->pos; HLD_head = hld_->head;
    }
    // 旧 API だけ閉区間 [l,r]。新 API path_segments は半開区間と方向を返す。
    std::vector<std::pair<int, int>> HLD_query(int u, int v, bool is_edge = false) {
        build(); if (!hld_) HLD_build();
        std::vector<std::pair<int, int>> result;
        for (const auto& segment : hld_->path_segments(u, v, is_edge))
            result.emplace_back(segment.l, segment.r - 1);
        return result;
    }

    std::vector<int> centroid(int root_ = -1, int excluded_parent = -1) {
        build(); if (root_ == -1) root_ = root;
        if (Base::size() == 0) return {};
        if (CD_removed.empty()) CD_removed.assign(Base::size(), false);
        if (CD_removed.at(root_)) return {};
        auto data = this->snapshot(true);
        std::vector<int> vertices{root_}, inv(Base::size(), -1);
        inv[root_] = 0;
        for (std::size_t i = 0; i < vertices.size(); ++i)
            for (const auto& e : data.graph[vertices[i]]) {
                if (e.to == excluded_parent || CD_removed[e.to] || inv[e.to] != -1) continue;
                inv[e.to] = static_cast<int>(vertices.size()); vertices.push_back(e.to);
            }
        cp::Graph<int> part(static_cast<int>(vertices.size()));
        for (const auto& e : data.graph.edges())
            if (inv[e.from] != -1 && inv[e.to] != -1) part.add_edge(inv[e.from], inv[e.to]);
        cp::RootedTree<int> rooted(part);
        CD_sz.resize(Base::size());
        for (int v : vertices) CD_sz[v] = rooted.subtree_size[inv[v]];
        std::vector<int> result;
        for (int v : cp::tree_centroids(part)) result.push_back(vertices[v]);
        return result;
    }
    void centroid_decomposition(int root_ = -1, int previous = -1) {
        // 部分領域への再帰呼び出しは新 API の parent をたどる形に移行する。
        assert((root_ == -1 || root_ == root) && previous == -1);
        (void)root_; (void)previous;
        build();
        cp::CentroidDecomposition<T> result(this->snapshot(true).graph);
        CD_par = std::move(result.parent); CD_sz = std::move(result.component_size);
        CD_removed.assign(Base::size(), true);
    }

private:
    std::optional<cp::DoublingLCA<T>> lca_;
    std::optional<cp::HeavyLightDecomposition<T>> hld_;
    std::size_t built_revision_ = 0;
    int built_root_ = -1;
    void clear_derived() {
        hld_.reset(); ET_V.clear(); ET_Vin.clear(); ET_Vout.clear();
        ET_E.clear(); ET_Ein.clear(); ET_Eout.clear(); ET_root_edge = -1;
        HLD.clear(); HLD_pos.clear(); HLD_head.clear();
        CD_removed.clear(); CD_sz.clear(); CD_par.clear();
    }
};
