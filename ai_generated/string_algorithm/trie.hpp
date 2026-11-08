#pragma once
#include <algorithm>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <string_view>
#include <utility>
#include <vector>
#include "detail/common.hpp"

namespace cp {
// 任意のbyteを辺に持つTrieの多重集合。空文字列・重複に対応、eraseは1個だけ削除。
class Trie {
    using Edge = std::pair<unsigned char, int>;
    struct Node { std::vector<Edge> edges; std::size_t passed = 0, terminal = 0; };
    std::vector<Node> nodes_{1};
    int child(int v, unsigned char byte) const {
        const auto& edges = nodes_[v].edges;
        auto it = std::lower_bound(edges.begin(), edges.end(), byte, [](const Edge& edge, unsigned char c) { return edge.first < c; });
        return it != edges.end() && it->first == byte ? it->second : -1;
    }
    int find(std::string_view s) const {
        string_algorithm_detail::checked_size(s.size()); int v = 0;
        for (unsigned char byte : s) { v = child(v, byte); if (v == -1) return -1; }
        return v;
    }
public:
    Trie() = default;
    Trie(const Trie&) = default;
    Trie& operator=(const Trie&) = default;
    Trie(Trie&& other) : Trie() { nodes_.swap(other.nodes_); }
    Trie& operator=(Trie&& other) { if (this != &other) { Trie moved(std::move(other)); nodes_.swap(moved.nodes_); } return *this; }
    std::size_t size() const { return nodes_[0].passed; }
    bool empty() const { return !size(); }
    std::size_t node_count() const { return nodes_.size(); }
    void clear() { std::vector<Node> fresh(1); nodes_.swap(fresh); }
    void insert(std::string_view s) {
        string_algorithm_detail::checked_size(s.size());
        if (size() == std::numeric_limits<std::size_t>::max()) throw std::overflow_error("trie multiplicity overflow");
        std::size_t matched = 0; int v = 0;
        for (unsigned char byte : s) { const int next = child(v, byte); if (next == -1) break; v = next; ++matched; }
        if (s.size() - matched > static_cast<std::size_t>(std::numeric_limits<int>::max()) - nodes_.size()) throw std::length_error("trie node count exceeds int range");
        std::vector<int> path{0}; path.reserve(s.size() + 1); v = 0;
        for (unsigned char byte : s) {
            int next = child(v, byte);
            if (next == -1) {
                next = static_cast<int>(nodes_.size()); nodes_.emplace_back();
                auto& edges = nodes_[v].edges;
                auto it = std::lower_bound(edges.begin(), edges.end(), byte, [](const Edge& edge, unsigned char c) { return edge.first < c; });
                edges.insert(it, {byte, next});
            }
            v = next; path.push_back(v);
        }
        for (int node : path) ++nodes_[node].passed;
        ++nodes_[v].terminal;
    }
    std::size_t count(std::string_view s) const { const int v = find(s); return v == -1 ? 0 : nodes_[v].terminal; }
    std::size_t count_prefix(std::string_view s) const { const int v = find(s); return v == -1 ? 0 : nodes_[v].passed; }
    bool contains(std::string_view s) const { return count(s) != 0; }
    bool erase(std::string_view s) {
        string_algorithm_detail::checked_size(s.size()); std::vector<int> path{0}; int v = 0;
        for (unsigned char byte : s) { v = child(v, byte); if (v == -1) return false; path.push_back(v); }
        if (!nodes_[v].terminal) return false;
        --nodes_[v].terminal; for (int node : path) --nodes_[node].passed;
        return true;
    }
};
} // namespace cp
