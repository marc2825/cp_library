#pragma once
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>
#include "big_integer.hpp"

namespace cp {
inline bool is_balanced_parentheses(std::string_view text) {
    std::size_t depth = 0;
    for (char c : text) {
        if (c == '(') ++depth;
        else if (c == ')' && depth) --depth;
        else return false;
    }
    return depth == 0;
}
// The implicit root 0 is not enclosed in parentheses. One pair per tree edge.
inline std::vector<std::vector<int>> parentheses_to_tree(std::string_view text) {
    if (!is_balanced_parentheses(text)) throw std::invalid_argument("unbalanced parentheses");
    if (text.size() / 2 >= static_cast<std::size_t>(std::numeric_limits<int>::max())) throw std::length_error("tree too large");
    std::vector<std::vector<int>> children(1); std::vector<int> path{0};
    for (char c : text) {
        if (c == '(') {
            const int v = static_cast<int>(children.size());
            children.emplace_back(); children[path.back()].push_back(v); path.push_back(v);
        } else path.pop_back();
    }
    return children;
}
inline std::string tree_to_parentheses(const std::vector<std::vector<int>>& children) {
    if (children.empty()) throw std::invalid_argument("ordered tree needs root 0");
    std::vector<bool> seen(children.size()); seen[0] = true;
    std::vector<std::pair<std::size_t, std::size_t>> stack{{0, 0}};
    std::string result; std::size_t visited = 1;
    while (!stack.empty()) {
        auto& frame = stack.back();
        if (frame.second == children[frame.first].size()) {
            stack.pop_back(); if (!stack.empty()) result += ')';
        } else {
            const int v = children[frame.first][frame.second++];
            if (v < 0 || static_cast<std::size_t>(v) >= children.size() || seen[v]) throw std::invalid_argument("not an ordered rooted tree");
            seen[v] = true; ++visited; result += '('; stack.emplace_back(static_cast<std::size_t>(v), 0);
        }
    }
    if (visited != children.size()) throw std::invalid_argument("disconnected ordered tree");
    return result;
}
// O(n^2) BigInt additions/storage, then O(n) rank/unrank (arithmetic costs excluded).
class ParenthesesTable {
    int pairs_;
    std::vector<std::vector<BigInt>> ways_;
public:
    explicit ParenthesesTable(int pairs) : pairs_(pairs) {
        if (pairs < 0 || pairs > std::numeric_limits<int>::max() / 2) throw std::invalid_argument("invalid pair count");
        ways_.assign(static_cast<std::size_t>(pairs) + 1, std::vector<BigInt>(static_cast<std::size_t>(pairs) + 1));
        for (int close = 0; close <= pairs; ++close) ways_[0][close] = 1;
        for (int open = 1; open <= pairs; ++open) for (int close = open; close <= pairs; ++close)
            ways_[open][close] = ways_[open - 1][close] + (close > open ? ways_[open][close - 1] : BigInt(0));
    }
    int pairs() const { return pairs_; }
    const BigInt& count() const { return ways_[pairs_][pairs_]; }
    std::string unrank(BigInt rank) const {
        if (rank < 0 || rank >= count()) throw std::out_of_range("parentheses rank");
        std::string result; result.reserve(static_cast<std::size_t>(pairs_) * 2);
        int open = pairs_, close = pairs_;
        while (close) {
            const BigInt starts_open = open ? ways_[open - 1][close] : BigInt(0);
            if (rank < starts_open) { result += '('; --open; }
            else { rank -= starts_open; result += ')'; --close; }
        }
        return result;
    }
    BigInt rank(std::string_view text) const {
        if (text.size() != static_cast<std::size_t>(pairs_) * 2 || !is_balanced_parentheses(text)) throw std::invalid_argument("invalid parentheses word");
        BigInt result = 0; int open = pairs_, close = pairs_;
        for (char c : text) {
            if (c == '(') --open;
            else { if (open) result += ways_[open - 1][close]; --close; }
        }
        return result;
    }
};
} // namespace cp
