#pragma once
#include <stdexcept>
#include <vector>
#include "negative_cycle.hpp"

namespace cp {
struct DifferenceConstraintsResult {
    std::vector<OptimizationWide> assignment;
    std::optional<NegativeCycle> contradiction;
    bool feasible() const { return !contradiction; }
};
// Constraint i is x[to]-x[from] <= bound; contradiction IDs refer to add_constraint.
template<class T=long long> class DifferenceConstraints {
    Graph<T> graph_;
    static int checked(int n) { if (n<0) throw std::invalid_argument("negative variable count"); return n; }
public:
    explicit DifferenceConstraints(int n):graph_(checked(n),GraphKind::Directed){}
    int size() const {return graph_.size();}
    int add_constraint(int from,int to,T bound) {
        if (from<0 || to<0 || from>=size() || to>=size()) throw std::out_of_range("constraint variable");
        return graph_.add_edge(from,to,bound);
    }
    const Graph<T>& graph() const {return graph_;}
    DifferenceConstraintsResult solve() const {
        auto witness=detail::bellman_ford_witness(graph_,-1);
        DifferenceConstraintsResult r; r.contradiction=std::move(witness.cycle);
        if (!r.contradiction) for (auto d:witness.distance) r.assignment.push_back(*d);
        return r;
    }
};
} // namespace cp
