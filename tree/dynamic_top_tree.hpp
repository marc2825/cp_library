#pragma once
// link/cut を伴うrake/compress。説明: docs/tree/dynamic_top_tree.md
#include <limits>
#include <optional>
#include <stdexcept>
#include <vector>
#include "detail/top_tree_core.hpp"

namespace cp { namespace detail {
template<class Spec, bool LazyEnabled> class TopTreeFacade {
public:
    using Info = typename Spec::Info;
    using Path = typename Spec::Path;
protected:
    TopTreeCore<Spec, LazyEnabled> core_;
    using NP = typename TopTreeCore<Spec, LazyEnabled>::NP;
    std::vector<NP> vertices_;
    NP node(int v) const {
        if (v < 0 || v >= size()) throw std::out_of_range("top tree vertex");
        return vertices_[v];
    }
public:
    explicit TopTreeFacade(const std::vector<Info>& values = {}) {
        if (values.size() > static_cast<std::size_t>(std::numeric_limits<int>::max()))
            throw std::length_error("too many top tree vertices");
        vertices_ = core_.build(values);
    }
    TopTreeFacade(const TopTreeFacade&) = delete;
    TopTreeFacade& operator=(const TopTreeFacade&) = delete;
    TopTreeFacade(TopTreeFacade&&) noexcept = default;
    TopTreeFacade& operator=(TopTreeFacade&&) noexcept = default;
    int size() const { return static_cast<int>(vertices_.size()); }
    std::size_t allocated_nodes() const {
        return core_.storage.size() + core_.splay_tree.storage.size();
    }
    bool connected(int u, int v) { return core_.is_connected(node(u), node(v)); }
    void make_root(int v) { core_.evert(node(v)); }
    bool link(int u, int v) {
        auto a = node(u), b = node(v);
        if (core_.is_connected(a, b)) return false;
        core_.evert(a); core_.link(a, b);
        return true;
    }
    bool cut(int u, int v) {
        auto a = node(u), b = node(v);
        core_.evert(a); core_.expose(b);
        if (b->l != a || a->r) return false;
        core_.cut(b); return true;
    }
    void set_vertex(int v, const Info& value) { core_.set_key(node(v), value); }
    Info get_vertex(int v) {
        auto p = node(v); core_.expose(p); return p->info;
    }
    // spine u→v と、その脇の全clusterを含む。純粋なパス積とは異なる。
    std::optional<Path> path_cluster(int u, int v) {
        if (!connected(u, v)) return std::nullopt;
        return core_.query_path(node(u), node(v));
    }
    Path component_cluster(int root) { return core_.query(node(root)); }
    std::optional<Path> subtree_cluster(int root, int v) {
        if (!connected(root, v)) return std::nullopt;
        return core_.query_subtree(node(root), node(v));
    }
};
} // namespace detail

// Spec: Info/Path/Point と vertex/add_vertex/add_edge/rake/compress。
// rakeは可換。合法なcluster分解によらず同じ木DPを表すこと。
template<class Spec> class DynamicTopTree : public detail::TopTreeFacade<Spec, false> {
    using Base = detail::TopTreeFacade<Spec, false>;
public:
    using Base::Base;
};
} // namespace cp
