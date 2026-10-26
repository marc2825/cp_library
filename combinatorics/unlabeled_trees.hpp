#pragma once
// 根付き/無根の非平面・ラベルなし木の同型類数。空木は両方 0。
// 説明: docs/combinatorics/unlabeled_trees.md
#include <cstddef>
#include <stdexcept>
#include <vector>
#include "../basic_utility/big_integer.hpp"
namespace cp {
struct UnlabeledTreeCounts {
    std::vector<BigInt> rooted, unrooted;
};
inline UnlabeledTreeCounts unlabeled_tree_counts(std::size_t n, std::size_t max_vertices = 2000) {
    if (n > max_vertices || n >= std::vector<BigInt>().max_size())
        throw std::length_error("unlabeled tree prefix exceeds budget");
    UnlabeledTreeCounts result{std::vector<BigInt>(n + 1), std::vector<BigInt>(n + 1)};
    std::vector<BigInt> divisor_sum(n + 1);
    for (std::size_t v = 1; v <= n; ++v) {
        auto& t = result.rooted[v];
        if (v == 1) t = 1;
        else {
            for (std::size_t k = 1; k < v; ++k) t += divisor_sum[k] * result.rooted[v - k];
            t /= BigInt(v - 1);
        }
        for (std::size_t k = v; k <= n; k += v) divisor_sum[k] += BigInt(v) * t;
        BigInt pairs = 0;
        for (std::size_t k = 1; k < v; ++k) pairs += result.rooted[k] * result.rooted[v - k];
        if (v % 2 == 0) pairs -= result.rooted[v / 2];
        result.unrooted[v] = t - pairs / BigInt(2);
    }
    return result;
}
} // namespace cp
