#pragma once
#include "power_projection.hpp"
namespace cp {
// Truncated formal-series composition, g(0)=0. O(n log^2 n), O(n log n) retained plans.
template <int Mod = 998244353, int Root = 3>
std::vector<int> fps_compose_fast(const std::vector<int> &outer, const std::vector<int> &inner,
                                  std::size_t n) {
    if (!inner.empty() && fps_detail::Field<Mod>::norm(inner[0]))
        throw std::domain_error("formal composition needs inner constant zero");
    return projection_detail::Engine<Mod, Root>(inner, n, std::min(n, outer.size())).compose(outer);
}
} // namespace cp
