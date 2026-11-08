#pragma once
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>
#include "graph.hpp"

namespace cp {
template <class T = long long> struct CSRArc {
    int from = -1, to = -1;
    T cost = T(1);
    int idx = -1, rev = -1;
};

// Static adjacency in input order. Undirected self-loops have two distinct arcs.
template <class T = long long> class CSRGraph {
    GraphKind kind_;
    std::vector<Edge<T>> edges_;
    std::vector<CSRArc<T>> arcs_;
    std::vector<std::size_t> offsets_;
    void vertex(int v) const {
        if (v < 0 || v >= size()) throw std::out_of_range("CSRGraph vertex out of range");
    }
public:
    using cost_type = T;
    class ArcRange {
        using Iterator = typename std::vector<CSRArc<T>>::const_iterator;
        const std::vector<CSRArc<T>>* arcs_;
        std::size_t first_, last_;
        Iterator iterator(std::size_t i) const {
            return i == 0 ? arcs_->cbegin() : arcs_->cbegin() + static_cast<std::ptrdiff_t>(i);
        }
    public:
        ArcRange(const std::vector<CSRArc<T>>& arcs, std::size_t first, std::size_t last)
            : arcs_(&arcs), first_(first), last_(last) {}
        Iterator begin() const { return iterator(first_); }
        Iterator end() const { return iterator(last_); }
        std::size_t size() const { return last_ - first_; }
        bool empty() const { return first_ == last_; }
        const CSRArc<T>& operator[](std::size_t i) const {
            if (i >= size()) throw std::out_of_range("CSRGraph local arc out of range");
            return (*arcs_)[first_ + i];
        }
    };
    explicit CSRGraph(int n = 0, std::vector<Edge<T>> edges = {}, GraphKind kind = GraphKind::Undirected)
        : kind_(kind), edges_(std::move(edges)) {
        if (n < 0) throw std::invalid_argument("negative CSRGraph size");
        if (kind != GraphKind::Directed && kind != GraphKind::Undirected) throw std::invalid_argument("invalid graph kind");
        const std::size_t factor = directed() ? 1 : 2;
        if (edges_.size() > static_cast<std::size_t>(std::numeric_limits<int>::max()) / factor)
            throw std::length_error("CSRGraph arc count exceeds int");
        offsets_.assign(static_cast<std::size_t>(n) + 1, 0);
        for (std::size_t i = 0; i < edges_.size(); ++i) {
            auto& edge = edges_[i]; vertex(edge.from); vertex(edge.to);
            edge.idx = static_cast<int>(i);
            ++offsets_[edge.from + 1];
            if (!directed()) ++offsets_[edge.to + 1];
        }
        for (int v = 0; v < n; ++v) offsets_[v + 1] += offsets_[v];
        arcs_.resize(edges_.size() * factor);
        auto next = offsets_;
        for (const auto& edge : edges_) {
            const auto a = next[edge.from]++;
            if (directed()) arcs_[a] = {edge.from, edge.to, edge.cost, edge.idx, -1};
            else {
                const auto b = next[edge.to]++;
                arcs_[a] = {edge.from, edge.to, edge.cost, edge.idx, static_cast<int>(b)};
                arcs_[b] = {edge.to, edge.from, edge.cost, edge.idx, static_cast<int>(a)};
            }
        }
    }
    explicit CSRGraph(const Graph<T>& graph) : CSRGraph(graph.size(), graph.edges(), graph.kind()) {}
    // A moved-from graph is empty and can be assigned/reused.
    CSRGraph(const CSRGraph&) = default;
    CSRGraph(CSRGraph&& other) : CSRGraph() { swap(other); }
    CSRGraph& operator=(CSRGraph other) { swap(other); return *this; }
    void swap(CSRGraph& other) noexcept {
        std::swap(kind_, other.kind_); edges_.swap(other.edges_); arcs_.swap(other.arcs_); offsets_.swap(other.offsets_);
    }
    friend void swap(CSRGraph& a, CSRGraph& b) noexcept { a.swap(b); }
    int size() const { return static_cast<int>(offsets_.size() - 1); }
    int edge_count() const { return static_cast<int>(edges_.size()); }
    int arc_count() const { return static_cast<int>(arcs_.size()); }
    GraphKind kind() const { return kind_; }
    bool directed() const { return kind_ == GraphKind::Directed; }
    ArcRange operator[](int v) const { vertex(v); return ArcRange(arcs_, offsets_[v], offsets_[v + 1]); }
    const std::vector<Edge<T>>& edges() const { return edges_; }
    const std::vector<CSRArc<T>>& arcs() const { return arcs_; }
    const std::vector<std::size_t>& offsets() const { return offsets_; }
    const CSRArc<T>& arc(int i) const {
        if (i < 0 || i >= arc_count()) throw std::out_of_range("CSRGraph arc out of range");
        return arcs_[i];
    }
    Graph<T> to_graph() const {
        Graph<T> result(size(), kind_);
        for (const auto& edge : edges_) result.add_edge(edge.from, edge.to, edge.cost);
        return result;
    }
};
} // namespace cp
