#pragma once
#include <algorithm>
#include <cstddef>
#include <functional>
#include <limits>
#include <optional>
#include <queue>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <vector>
#include "detail/arithmetic.hpp"

namespace cp {
// Integral capacities and signed costs. Negative residual cycles in any component are rejected before the first positive flow.
template<class Cap = long long, class Cost = long long>
class MinCostFlow {
    static_assert(std::is_integral_v<Cap> && !std::is_same_v<Cap, bool> && sizeof(Cap) <= 8,
                  "MinCostFlow needs an integer capacity of at most 64 bits");
    static_assert(std::is_integral_v<Cost> && std::is_signed_v<Cost> && sizeof(Cost) <= 8,
                  "MinCostFlow needs a signed integer cost of at most 64 bits");
    using Wide = OptimizationWide;
    struct Arc { int to; Cap residual; Wide cost; };
    std::vector<std::vector<int>> adjacency_;
    std::vector<Arc> arcs_;
    std::vector<Wide> potential_;
    int source_ = -1, sink_ = -1;
    bool initialized_ = false;
    void vertex(int v) const {
        if (v < 0 || v >= size()) throw std::out_of_range("min-cost flow vertex");
    }
    void initialize() {
        using namespace optimization_detail;
        std::vector<Wide> next(size(), 0);
        bool negative = false;
        for (const auto& e : arcs_) if (e.residual > Cap{} && e.cost < 0) negative = true;
        if (negative) {
            // A virtual zero-cost source reaches every vertex, including disconnected components.
            for (int pass = 0; pass < size(); ++pass) {
                bool changed = false;
                for (int v = 0; v < size(); ++v) for (int id : adjacency_[v]) {
                    const auto& e = arcs_[id];
                    if (e.residual == Cap{}) continue;
                    const Wide candidate = add(next[v], e.cost);
                    if (candidate < next[e.to]) { next[e.to] = candidate; changed = true; }
                }
                if (!changed) break;
                if (pass == size() - 1) throw std::domain_error("negative residual cycle");
            }
        }
        potential_ = std::move(next); initialized_ = true;
    }
public:
    struct Result { Cap flow{}; Cost cost{}; };
    struct Edge { int from, to; Cap capacity, flow; Cost cost; };
private:
    template<class Emit> Result run(int source, int sink, Cap limit, Emit emit) {
        using namespace optimization_detail;
        vertex(source); vertex(sink);
        if (source == sink || limit < Cap{}) throw std::invalid_argument("invalid min-cost flow terminals or limit");
        if (source_ != -1 && (source_ != source || sink_ != sink))
            throw std::logic_error("reset_flow before changing flow terminals");
        source_ = source; sink_ = sink;
        Result result;
        if (limit == Cap{}) return result;
        if (!initialized_) initialize();
        while (result.flow < limit) {
            std::vector<std::optional<Wide>> distance(size());
            std::vector<int> previous(size(), -1);
            using State = std::pair<Wide, int>;
            std::priority_queue<State, std::vector<State>, std::greater<State>> queue;
            distance[source] = 0; queue.emplace(0, source);
            while (!queue.empty()) {
                const auto [value, v] = queue.top(); queue.pop();
                if (value != *distance[v]) continue;
                for (int id : adjacency_[v]) {
                    const auto& e = arcs_[id];
                    if (e.residual == Cap{}) continue;
                    const Wide reduced = add(e.cost, sub(potential_[v], potential_[e.to]));
                    if (reduced < 0) throw std::logic_error("invalid min-cost flow potential");
                    const Wide candidate = add(value, reduced);
                    if (!distance[e.to] || candidate < *distance[e.to]) {
                        distance[e.to] = candidate; previous[e.to] = id;
                        queue.emplace(candidate, e.to);
                    }
                }
            }
            if (!distance[sink]) break;
            // Prepare arithmetic before changing the residual network for this augmentation.
            std::vector<Wide> next = potential_;
            for (int v = 0; v < size(); ++v) if (distance[v]) next[v] = add(next[v], *distance[v]);
            Cap amount = limit - result.flow;
            for (int v = sink; v != source; v = arcs_[previous[v] ^ 1].to)
                amount = std::min(amount, arcs_[previous[v]].residual);
            const Wide unit_cost = sub(next[sink], next[source]);
            const Cost total = narrow<Cost>(add(Wide(result.cost), mul(Wide(amount), unit_cost)));
            potential_ = std::move(next);
            for (int v = sink; v != source; v = arcs_[previous[v] ^ 1].to) {
                const int id = previous[v];
                arcs_[id].residual -= amount; arcs_[id ^ 1].residual += amount;
            }
            result.flow += amount; result.cost = total;
            emit(result, unit_cost);
        }
        return result;
    }
public:
    explicit MinCostFlow(int n = 0) {
        if (n < 0) throw std::invalid_argument("negative min-cost flow size");
        adjacency_.resize(n);
    }
    int size() const { return static_cast<int>(adjacency_.size()); }
    int edge_count() const { return static_cast<int>(arcs_.size() / 2); }
    int add_edge(int from, int to, Cap capacity, Cost cost) {
        vertex(from); vertex(to);
        if (capacity < Cap{}) throw std::invalid_argument("negative flow capacity");
        if (source_ != -1) throw std::logic_error("reset_flow before adding edges");
        if (arcs_.size() > std::size_t(std::numeric_limits<int>::max() - 2))
            throw std::length_error("too many min-cost flow edges");
        const int id = edge_count(), arc = static_cast<int>(arcs_.size());
        arcs_.push_back({to, capacity, Wide(cost)});
        arcs_.push_back({from, Cap{}, optimization_detail::sub(0, Wide(cost))});
        adjacency_[from].push_back(arc); adjacency_[to].push_back(arc ^ 1);
        return id;
    }
    Edge get_edge(int id) const {
        if (id < 0 || id >= edge_count()) throw std::out_of_range("min-cost flow edge");
        const auto& a = arcs_[2 * id]; const auto& b = arcs_[2 * id + 1];
        return {b.to, a.to, Cap(a.residual + b.residual), b.residual,
                optimization_detail::narrow<Cost>(a.cost)};
    }
    std::vector<Edge> edges() const {
        std::vector<Edge> result; result.reserve(edge_count());
        for (int i = 0; i < edge_count(); ++i) result.push_back(get_edge(i));
        return result;
    }
    // Additional flow and additional cost; the same terminals are fixed until reset.
    Result flow(int source, int sink, Cap limit = std::numeric_limits<Cap>::max()) {
        return run(source, sink, limit, [](Result, Wide) {});
    }
    // Starts at (0,0) for this call; adjacent equal marginal costs are coalesced.
    std::vector<Result> slope(int source, int sink, Cap limit = std::numeric_limits<Cap>::max()) {
        std::vector<Result> points(1);
        std::optional<Wide> last_cost;
        run(source, sink, limit, [&](Result point, Wide cost) {
            if (last_cost && *last_cost == cost) points.pop_back();
            points.push_back(point); last_cost = cost;
        });
        return points;
    }
    void reset_flow() {
        for (std::size_t i = 0; i < arcs_.size(); i += 2) {
            arcs_[i].residual += arcs_[i + 1].residual; arcs_[i + 1].residual = Cap{};
        }
        source_ = sink_ = -1; initialized_ = false; potential_.clear();
    }
};
} // namespace cp
