#pragma once
#include <algorithm>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>
#include "permutation.hpp"
#include "../range_query/lazy_segtree.hpp"

// Adapted from hitonanode/cplib-cpp other_algorithms/permutation_tree.hpp.
// https://hitonanode.github.io/cplib-cpp/other_algorithms/permutation_tree.hpp.html
// MIT License
// Copyright (c) 2019 Ryotaro Sato
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

namespace cp {
namespace permutation_tree_detail {
inline std::int64_t minimum(std::int64_t a, std::int64_t b) { return std::min(a, b); }
inline std::int64_t infinity() { return std::numeric_limits<std::int64_t>::max() / 4; }
inline std::int64_t add(std::int64_t a, std::int64_t b) { return a + b; }
inline std::int64_t zero() { return 0; }
using MinAdd = LazySegtree<std::int64_t, minimum, infinity, std::int64_t, add, add, zero>;
} // namespace permutation_tree_detail

// Strong common interval tree, O(n log n) construction, O(n) storage, no deep DFS.
class PermutationTree {
public:
    enum class Type { Leaf, Increasing, Decreasing, Prime };
    struct Node {
        Type type;
        int left, right, low, high, parent = -1; // Both position and value ranges are half-open.
        std::vector<int> children; // Position order, not node-ID order.
    };
private:
    std::vector<Node> nodes_;
    int root_ = -1;
    void append(int parent, int child) {
        auto& a = nodes_[parent]; const auto& b = nodes_[child];
        a.children.push_back(child);
        a.left = std::min(a.left, b.left); a.right = std::max(a.right, b.right);
        a.low = std::min(a.low, b.low); a.high = std::max(a.high, b.high);
    }
public:
    explicit PermutationTree(const std::vector<int>& p) {
        if (p.size() > 100000000) throw std::length_error("permutation tree too large");
        validate_permutation(p);
        const int n = static_cast<int>(p.size());
        if (!n) return;
        nodes_.reserve(2 * p.size() - 1);
        permutation_tree_detail::MinAdd gaps(std::vector<std::int64_t>(p.size(), 0));
        std::vector<int> maxima{-1}, minima{-1}, stack;
        for (int i = 0; i < n; ++i) {
            while (maxima.back() >= 0 && p[maxima.back()] < p[i]) {
                gaps.apply(maxima[maxima.size() - 2] + 1, maxima.back() + 1, p[i] - p[maxima.back()]);
                maxima.pop_back();
            }
            maxima.push_back(i);
            while (minima.back() >= 0 && p[i] < p[minima.back()]) {
                gaps.apply(minima[minima.size() - 2] + 1, minima.back() + 1, p[minima.back()] - p[i]);
                minima.pop_back();
            }
            minima.push_back(i);
            int current = static_cast<int>(nodes_.size());
            nodes_.push_back({Type::Leaf, i, i + 1, p[i], p[i] + 1, -1, {}});
            for (;;) {
                Type join = Type::Prime;
                if (!stack.empty()) {
                    if (nodes_[stack.back()].high == nodes_[current].low) join = Type::Increasing;
                    if (nodes_[current].high == nodes_[stack.back()].low) join = Type::Decreasing;
                }
                if (join != Type::Prime) {
                    int previous = stack.back(); stack.pop_back();
                    if (nodes_[previous].type != join) {
                        const auto old = nodes_[previous];
                        nodes_.push_back({join, old.left, old.right, old.low, old.high, -1, {previous}});
                        previous = static_cast<int>(nodes_.size()) - 1;
                    }
                    append(previous, current); current = previous;
                } else if (gaps.prod(0, nodes_[current].left) == 0) {
                    const auto old = nodes_[current];
                    nodes_.push_back({Type::Prime, old.left, old.right, old.low, old.high, -1, {current}});
                    current = static_cast<int>(nodes_.size()) - 1;
                    do {
                        append(current, stack.back()); stack.pop_back();
                    } while (nodes_[current].high - nodes_[current].low != nodes_[current].right - nodes_[current].left);
                    std::reverse(nodes_[current].children.begin(), nodes_[current].children.end());
                } else break;
            }
            stack.push_back(current);
            // At the next endpoint, gap[l] is max-min+1-length for [l,i+2).
            gaps.apply(0, i + 1, -1);
        }
        root_ = stack.front();
        for (int id = 0; id < static_cast<int>(nodes_.size()); ++id)
            for (int child : nodes_[id].children) nodes_[child].parent = id;
    }
    int root() const { return root_; }
    const std::vector<Node>& nodes() const { return nodes_; }
    std::uint64_t common_interval_count() const {
        std::uint64_t result = 0;
        for (const auto& node : nodes_) {
            const auto k = static_cast<std::uint64_t>(node.children.size());
            result += node.type == Type::Increasing || node.type == Type::Decreasing ? k * (k - 1) / 2 : 1;
        }
        return result;
    }
};
} // namespace cp
