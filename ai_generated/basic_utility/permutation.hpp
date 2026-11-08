#pragma once
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace cp {
inline bool is_permutation(const std::vector<int>& p) {
    if (p.size() > static_cast<std::size_t>(std::numeric_limits<int>::max())) return false;
    std::vector<bool> seen(p.size());
    for (int x : p) {
        if (x < 0 || static_cast<std::size_t>(x) >= p.size() || seen[x]) return false;
        seen[x] = true;
    }
    return true;
}
inline void validate_permutation(const std::vector<int>& p) {
    if (!is_permutation(p)) throw std::invalid_argument("expected permutation of [0,n)");
}
// Composition p after q: result[i] = p[q[i]].
inline std::vector<int> permutation_compose(const std::vector<int>& p, const std::vector<int>& q) {
    validate_permutation(p); validate_permutation(q);
    if (p.size() != q.size()) throw std::invalid_argument("permutation size mismatch");
    std::vector<int> result(p.size());
    for (std::size_t i = 0; i < p.size(); ++i) result[i] = p[q[i]];
    return result;
}
inline std::vector<int> permutation_inverse(const std::vector<int>& p) {
    validate_permutation(p); std::vector<int> result(p.size());
    for (std::size_t i = 0; i < p.size(); ++i) result[p[i]] = static_cast<int>(i);
    return result;
}
inline std::vector<std::vector<int>> permutation_cycles(const std::vector<int>& p) {
    validate_permutation(p);
    std::vector<bool> seen(p.size()); std::vector<std::vector<int>> cycles;
    for (int i = 0; i < static_cast<int>(p.size()); ++i) if (!seen[i]) {
        std::vector<int> cycle;
        for (int x = i; !seen[x]; x = p[x]) { seen[x] = true; cycle.push_back(x); }
        cycles.push_back(std::move(cycle));
    }
    return cycles;
}
inline int permutation_sign(const std::vector<int>& p) {
    return (p.size() - permutation_cycles(p).size()) % 2 ? -1 : 1;
}
inline std::vector<int> permutation_power(const std::vector<int>& p, std::int64_t exponent) {
    std::vector<int> result(p.size());
    for (const auto& cycle : permutation_cycles(p)) {
        const auto length = static_cast<std::int64_t>(cycle.size());
        auto shift = exponent % length; if (shift < 0) shift += length;
        for (std::size_t i = 0; i < cycle.size(); ++i) result[cycle[i]] = cycle[(i + static_cast<std::size_t>(shift)) % cycle.size()];
    }
    return result;
}
} // namespace cp
