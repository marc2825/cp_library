#pragma once
// パス/部分木/成分への独立したlazy作用。説明: docs/tree/lazy_top_tree.md
#include "dynamic_top_tree.hpp"

namespace cp {
// Specの追加条件: Lazy::id/propagate、Info/Point/Path::propagate、
// Path::propagate_light。spineとoff-spineへ作用を分配できること。
template<class Spec> class LazyTopTree : public detail::TopTreeFacade<Spec, true> {
    using Base = detail::TopTreeFacade<Spec, true>;
public:
    using Lazy = typename Spec::Lazy;
    using Base::Base;
    bool path_apply(int u, int v, const Lazy& f) {
        if (!this->connected(u, v)) return false;
        this->core_.set_propagate_path(this->node(u), this->node(v), f);
        return true;
    }
    bool subtree_apply(int root, int v, const Lazy& f) {
        if (!this->connected(root, v)) return false;
        this->core_.set_propagate_subtree(this->node(root), this->node(v), f);
        return true;
    }
    void component_apply(int v, const Lazy& f) { subtree_apply(v, v, f); }
};

// 既存ActedMonoid (S,F,op,e,mapping,composition,id) の共有アダプタ。
// opは可換。Sに長さ等を含める。mapping(f,e())==e()が必要。
template<class Action> struct TopTreeMonoidAction {
    using S = typename Action::S;
    using F = typename Action::F;
    struct Lazy {
        F value;
        static Lazy id() { return {Action::id()}; }
        void propagate(const Lazy& newer) { value = Action::composition(newer.value, value); }
    };
    struct Info {
        S value;
        void propagate(const Lazy& f) { value = Action::mapping(f.value, value); }
    };
    struct Point {
        S value;
        void propagate(const Lazy& f) { value = Action::mapping(f.value, value); }
    };
    struct Path {
        S spine, off;
        void propagate(const Lazy& f) { spine = Action::mapping(f.value, spine); }
        void propagate_light(const Lazy& f) { off = Action::mapping(f.value, off); }
        S all() const { return Action::op(spine, off); }
    };
    static Path vertex(const Info& x) { return {x.value, Action::e()}; }
    static Path add_vertex(const Point& p, const Info& x) { return {x.value, p.value}; }
    static Point add_edge(const Path& p) { return {p.all()}; }
    static Point rake(const Point& a, const Point& b) { return {Action::op(a.value, b.value)}; }
    static Path compress(const Path& a, const Path& b) {
        return {Action::op(a.spine, b.spine), Action::op(a.off, b.off)};
    }
};
} // namespace cp
