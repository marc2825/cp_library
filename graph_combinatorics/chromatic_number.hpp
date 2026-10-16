#pragma once
#include <cstdint>
#include <stdexcept>
#include <vector>
#include "detail/bit_graph.hpp"

namespace cp {
struct GraphColoringResult {
    int chromatic_number = 0;
    std::vector<int> color;
    std::vector<std::vector<int>> groups;
};

// Exact subset DP, O(3^n) time / O(2^n) memory; n<=22. No modular zero tests.
template<class T> GraphColoringResult minimum_coloring(const Graph<T>& g) {
    using namespace graph_combinatorics_detail;
    if (g.size() > 22) throw std::length_error("subset coloring supports at most 22 vertices");
    const auto adjacency = adjacency_masks(g);
    const std::uint32_t states = std::uint32_t{1} << g.size();
    std::vector<unsigned char> independent(states), dp(states);
    std::vector<std::uint32_t> choice(states);
    independent[0] = 1;
    for (std::uint32_t mask = 1; mask < states; ++mask) {
        const int v = first_vertex(mask);
        const std::uint32_t bit = std::uint32_t{1} << v, rest = mask ^ bit;
        independent[mask] = independent[rest] && !(adjacency[v] & rest);
        if (independent[mask]) { dp[mask] = 1; choice[mask] = mask; continue; }
        dp[mask] = static_cast<unsigned char>(g.size() + 1);
        const std::uint32_t allowed = rest & ~static_cast<std::uint32_t>(adjacency[v]);
        for (std::uint32_t sub = allowed;; sub = (sub - 1) & allowed) {
            if (independent[sub] && dp[mask ^ (sub | bit)] + 1 < dp[mask]) {
                dp[mask] = static_cast<unsigned char>(dp[mask ^ (sub | bit)] + 1);
                choice[mask] = sub | bit;
            }
            if (dp[mask] == 2 || !sub) break;
        }
    }
    GraphColoringResult result;
    result.chromatic_number = dp.back(); result.color.assign(g.size(), -1);
    for (std::uint32_t mask = states - 1; mask;) {
        std::vector<int> group;
        for (Mask sub = choice[mask]; sub; sub &= sub - 1) {
            int v = first_vertex(sub); group.push_back(v);
            result.color[v] = static_cast<int>(result.groups.size());
        }
        result.groups.push_back(group); mask ^= choice[mask];
    }
    return result;
}
template<class T> int chromatic_number(const Graph<T>& g) { return minimum_coloring(g).chromatic_number; }
} // namespace cp
