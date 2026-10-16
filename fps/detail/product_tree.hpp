#pragma once
#include <cstddef>
#include <stdexcept>
#include <vector>
#include "../division.hpp"

namespace cp {
namespace fps_detail {
template <int Mod, int Root> struct ProductTree {
    std::size_t count, base = 1;
    std::vector<std::vector<int>> nodes;
    explicit ProductTree(const std::vector<int>& xs) : count(xs.size()) {
        if (count >= max_size<Mod, Root>()) throw std::length_error("too many product-tree points");
        while (base < count) base *= 2;
        nodes.resize(2 * base);
        for (std::size_t i = 0; i < base; ++i)
            nodes[base + i] = i < count ? std::vector<int>{Field<Mod>::sub(0, Field<Mod>::norm(xs[i])), 1} : std::vector<int>{1};
        for (std::size_t i = base; --i > 0;) nodes[i] = multiply<Mod, Root>(nodes[2 * i], nodes[2 * i + 1]);
    }
    std::vector<int> evaluate(const std::vector<int>& a) const {
        if (!count) return {};
        std::vector<int> values(count);
        descend(1, polynomial_remainder<Mod, Root>(a, nodes[1]), values);
        return values;
    }
private:
    void descend(std::size_t index, const std::vector<int>& r, std::vector<int>& values) const {
        if (index >= base) {
            if (index - base < count) values[index - base] = r.empty() ? 0 : r[0];
            return;
        }
        descend(2 * index, polynomial_remainder<Mod, Root>(r, nodes[2 * index]), values);
        if (nodes[2 * index + 1].size() > 1)
            descend(2 * index + 1, polynomial_remainder<Mod, Root>(r, nodes[2 * index + 1]), values);
    }
};
} // namespace fps_detail
} // namespace cp
