#pragma once
// link/cutと指定根の部分木prod/apply。説明: docs/tree/dynamic_subtree_aggregate.md
#include <optional>
#include <vector>
#include "lazy_top_tree.hpp"

namespace cp {
template<class Action> class DynamicSubtreeAggregate {
    using Spec = TopTreeMonoidAction<Action>;
    LazyTopTree<Spec> tree_;
    static std::vector<typename Spec::Info> wrap(const std::vector<typename Action::S>& values) {
        std::vector<typename Spec::Info> result;
        result.reserve(values.size());
        for (const auto& x : values) result.push_back({x});
        return result;
    }
public:
    using S = typename Action::S;
    using F = typename Action::F;
    explicit DynamicSubtreeAggregate(const std::vector<S>& values = {}) : tree_(wrap(values)) {}
    int size() const { return tree_.size(); }
    std::size_t allocated_nodes() const { return tree_.allocated_nodes(); }
    bool link(int u, int v) { return tree_.link(u, v); }
    bool cut(int u, int v) { return tree_.cut(u, v); }
    bool connected(int u, int v) { return tree_.connected(u, v); }
    void make_root(int v) { tree_.make_root(v); }
    void set(int v, const S& x) { tree_.set_vertex(v, {x}); }
    S get(int v) { return tree_.get_vertex(v).value; }
    std::optional<S> subtree_prod(int root, int v) {
        auto p = tree_.subtree_cluster(root, v);
        if (!p) return std::nullopt;
        return p->all();
    }
    bool subtree_apply(int root, int v, const F& f) { return tree_.subtree_apply(root, v, {f}); }
    S component_prod(int v) { return tree_.component_cluster(v).all(); }
    void component_apply(int v, const F& f) { tree_.component_apply(v, {f}); }
    std::optional<S> path_prod(int u, int v) {
        auto p = tree_.path_cluster(u, v);
        if (!p) return std::nullopt;
        return p->spine;
    }
    bool path_apply(int u, int v, const F& f) { return tree_.path_apply(u, v, {f}); }
};
} // namespace cp
