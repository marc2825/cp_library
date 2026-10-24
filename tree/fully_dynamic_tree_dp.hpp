#pragma once
// 木構造も変更できるDP。説明: docs/tree/fully_dynamic_tree_dp.md
#include <array>
#include <optional>
#include <utility>
#include "dynamic_top_tree.hpp"

namespace cp {
// DynamicTopTreeのSpecにfinish(Path)->Resultを追加する。
template<class Spec> class FullyDynamicTreeDP : public DynamicTopTree<Spec> {
    using Base = DynamicTopTree<Spec>;
public:
    using Base::Base;
    using Result = decltype(Spec::finish(std::declval<typename Spec::Path>()));
    Result query(int root) { return Spec::finish(this->component_cluster(root)); }
    std::optional<Result> subtree_query(int root, int v) {
        auto p = this->subtree_cluster(root, v);
        if (!p) return std::nullopt;
        return Spec::finish(*p);
    }
};

// Σ_{独立集合I} ∏_{v∈I} weight[v]。加減乗算できる可換環、逆元不要。
template<class T> struct IndependentSetTreeDP {
    using Info = T;
    using Path = std::array<std::array<T, 2>, 2>;
    struct Point { T any, white; };
    static Path vertex(const Info& w) { return add_vertex({T(1), T(1)}, w); }
    static Path add_vertex(const Point& p, const Info& w) {
        return {{{p.any, p.any}, {w * p.white, T(0)}}};
    }
    static Point add_edge(const Path& p) { return {p[0][0] + p[1][0], p[0][0]}; }
    static Point rake(const Point& a, const Point& b) { return {a.any * b.any, a.white * b.white}; }
    static Path compress(const Path& a, const Path& b) {
        Path c{};
        for (int i = 0; i < 2; ++i) for (int j = 0; j < 2; ++j)
            c[i][j] = a[i][0] * b[0][j] + a[i][1] * b[1][j];
        return c;
    }
    static T finish(const Path& p) { return p[0][0] + p[1][0]; }
};
template<class T> using FullyDynamicIndependentSet = FullyDynamicTreeDP<IndependentSetTreeDP<T>>;

// 単位辺長の木。query(root)=Σ_v weight[v]*dist(root,v)。
template<class T> struct DistanceSumTreeDP {
    using Info = T;
    struct Point { T mass, distance; };
    struct Path { int length; T mass, top, bottom; };
    static Path vertex(const Info& w) { return {1, w, T(0), T(0)}; }
    static Path add_vertex(const Point& p, const Info& w) {
        return {1, p.mass + w, p.distance, p.distance};
    }
    static Point add_edge(const Path& p) { return {p.mass, p.top + p.mass}; }
    static Point rake(const Point& a, const Point& b) { return {a.mass + b.mass, a.distance + b.distance}; }
    static Path compress(const Path& a, const Path& b) {
        return {a.length + b.length, a.mass + b.mass,
            a.top + b.top + T(a.length) * b.mass, b.bottom + a.bottom + T(b.length) * a.mass};
    }
    static T finish(const Path& p) { return p.top; }
};
template<class T> using FullyDynamicDistanceSum = FullyDynamicTreeDP<DistanceSumTreeDP<T>>;
} // namespace cp
