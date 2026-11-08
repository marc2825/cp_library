#pragma once
#include "../basic_utility/big_integer.hpp"
#include <cstdint>
#include <map>
namespace cp::geometry::exact {
// Inserts [0,x] x [0,y], x,y >= 0. Skyline x increases, y strictly decreases.
class OriginRectangleUnion {
    std::map<std::int64_t, std::int64_t> corners_;
    BigInt area_;
    BigInt contribution(std::map<std::int64_t, std::int64_t>::const_iterator it) const {
        const auto left = it == corners_.begin() ? 0 : std::prev(it)->first;
        return (BigInt(it->first) - left) * it->second;
    }

  public:
    const BigInt &area() const { return area_; }
    const std::map<std::int64_t, std::int64_t> &skyline() const { return corners_; }
    BigInt insert(std::int64_t x, std::int64_t y) {
        if (x < 0 || y < 0)
            throw std::invalid_argument("origin rectangle needs nonnegative coordinates");
        if (!x || !y)
            return area_;
        auto next = corners_.lower_bound(x);
        if (next != corners_.end() && next->second >= y)
            return area_;
        // Remove the local contributions before altering predecessor coordinates.
        if (next != corners_.end())
            area_ -= contribution(next);
        if (next != corners_.end() && next->first == x) {
            auto after = std::next(next);
            if (after != corners_.end())
                area_ -= contribution(after);
            next = corners_.erase(next);
        }
        while (next != corners_.begin()) {
            auto previous = std::prev(next);
            if (previous->second > y)
                break;
            area_ -= contribution(previous);
            corners_.erase(previous);
        }
        auto current = corners_.emplace_hint(next, x, y);
        area_ += contribution(current);
        if (next != corners_.end())
            area_ += contribution(next);
        return area_;
    }
};
} // namespace cp::geometry::exact
