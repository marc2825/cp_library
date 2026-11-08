#pragma once
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <utility>
#include <vector>
#include "safe_integer.hpp"

namespace cp {
struct TwoPointersResult {
    // Valid nonempty intervals starting at l are exactly [l, r), l < r <= right[l].
    std::vector<std::size_t> right;
    std::pair<std::size_t, std::size_t> longest() const {
        std::pair<std::size_t, std::size_t> best{0, 0};
        for (std::size_t l = 0; l < right.size(); ++l)
            if (right[l] - l > best.second - best.first) best = {l, right[l]};
        return best;
    }
    std::uint64_t count() const {
        std::uint64_t total = 0;
        for (std::size_t l = 0; l < right.size(); ++l) {
            const auto next = checked_add(total, static_cast<std::uint64_t>(right[l] - l));
            if (!next) throw std::overflow_error("interval count overflow");
            total = *next;
        }
        return total;
    }
    // Lexicographic (l,r) order. Return false from visit to stop immediately.
    template<class Visit> bool for_each(Visit visit) const {
        for (std::size_t l = 0; l < right.size(); ++l)
            for (std::size_t r = l + 1; r <= right[l]; ++r)
                if (!visit(l, r)) return false;
        return true;
    }
};
// Validity must be closed under taking subintervals; empty intervals are valid.
// State starts empty. can_add(r) only observes; add/remove update the window.
template<class CanAdd, class Add, class Remove>
TwoPointersResult two_pointers(std::size_t n, CanAdd can_add, Add add, Remove remove) {
    TwoPointersResult result{std::vector<std::size_t>(n)};
    std::size_t r = 0;
    for (std::size_t l = 0; l < n; ++l) {
        while (r < n && can_add(r)) { add(r); ++r; }
        result.right[l] = r;
        if (l < r) remove(l);
        else ++r; // The rejected singleton was never added.
    }
    return result;
}
} // namespace cp
