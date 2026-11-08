#pragma once
#include "detail/common.hpp"
#include <cstdint>
#include <deque>
#include <map>
#include <string>
#include <vector>

namespace cp {
// Surface-recording deque eertree. Historical palindrome nodes remain allocated until clear().
class DoubleEndedEertree {
    struct DictionaryNode {
        int child[2] = {0, 0};
        int value = 0;
    };
    struct Node {
        int length = 0, link = 0, direct = 0, suffix_depth = 0, active_children = 0;
        std::uint64_t count = 0;
        bool active = false;
        std::map<unsigned char, int> next;
    };
    std::vector<DictionaryNode> dictionary_{1};
    std::vector<Node> nodes_{2};
    std::deque<unsigned char> text_;
    std::deque<int> prefix_surface_, suffix_surface_;
    int distinct_ = 0;
    std::uint64_t total_ = 0;
    int lookup(int root, unsigned char c) const {
        for (int bit = 7; bit >= 0; --bit)
            root = dictionary_[root].child[(c >> bit) & 1];
        return dictionary_[root].value;
    }
    int update(int root, unsigned char c, int value, int bit = 7) {
        DictionaryNode copy = dictionary_[root];
        const int id = static_cast<int>(dictionary_.size());
        dictionary_.push_back(copy);
        if (bit < 0)
            dictionary_[id].value = value;
        else {
            const int side = (c >> bit) & 1;
            const int child = update(copy.child[side], c, value, bit - 1);
            dictionary_[id].child[side] = child;
        }
        return id;
    }
    int extendable(int v, unsigned char c, bool front) const {
        const int p = front ? nodes_[v].length + 1 : size() - nodes_[v].length - 2;
        if (p >= 0 && p < size() && text_[p] == c)
            return v;
        return lookup(nodes_[v].direct, c);
    }
    int extend(int old, unsigned char c, bool front) {
        const int inner = extendable(old, c, front);
        auto it = nodes_[inner].next.find(c);
        int v;
        if (it != nodes_[inner].next.end())
            v = it->second;
        else {
            Node node;
            node.length = nodes_[inner].length + 2;
            if (node.length == 1)
                node.link = 1;
            else {
                const int candidate = extendable(nodes_[inner].link, c, front);
                node.link = nodes_[candidate].next.at(c);
            }
            node.suffix_depth = nodes_[node.link].suffix_depth + 1;
            const int p = front ? nodes_[node.link].length : size() - 1 - nodes_[node.link].length;
            node.direct = update(nodes_[node.link].direct, text_[p], node.link);
            v = static_cast<int>(nodes_.size());
            nodes_.push_back(std::move(node));
            nodes_[inner].next[c] = v;
        }
        if (!nodes_[v].active) {
            nodes_[v].active = true;
            ++distinct_;
            ++nodes_[nodes_[v].link].active_children;
        }
        ++nodes_[v].count;
        total_ += nodes_[v].suffix_depth;
        return v;
    }
    void remove_occurrence(int v) {
        if (nodes_[v].count == 1 && !nodes_[v].active_children) {
            nodes_[v].active = false;
            --distinct_;
            --nodes_[nodes_[v].link].active_children;
        }
        --nodes_[v].count;
        total_ -= nodes_[v].suffix_depth;
    }
    void reserve_operation() const {
        if (text_.size() >= static_cast<std::size_t>(std::numeric_limits<int>::max() - 2) ||
            dictionary_.size() > static_cast<std::size_t>(std::numeric_limits<int>::max() - 10))
            throw std::length_error("deque eertree indices exhausted");
    }

  public:
    DoubleEndedEertree() {
        nodes_[0].length = -1;
        nodes_[0].active = nodes_[1].active = true;
    }
    explicit DoubleEndedEertree(std::string_view text) : DoubleEndedEertree() {
        for (unsigned char c : text)
            push_back(c);
    }
    int size() const { return static_cast<int>(text_.size()); }
    bool empty() const { return text_.empty(); }
    int distinct_count() const { return distinct_; }
    std::uint64_t total_occurrences() const { return total_; }
    int longest_prefix() const { return empty() ? 0 : nodes_[prefix_surface_.front()].length; }
    int longest_suffix() const { return empty() ? 0 : nodes_[suffix_surface_.back()].length; }
    std::size_t allocated_nodes() const { return nodes_.size() - 2; }
    std::string text() const { return std::string(text_.begin(), text_.end()); }
    void clear() { *this = DoubleEndedEertree(); }
    void push_back(unsigned char c) {
        reserve_operation();
        const int old = empty() ? 1 : suffix_surface_.back();
        text_.push_back(c);
        prefix_surface_.push_back(1);
        suffix_surface_.push_back(1);
        const int v = extend(old, c, false), f = nodes_[v].link, start = size() - nodes_[v].length;
        prefix_surface_[start] = suffix_surface_.back() = v;
        if (nodes_[f].length > 0) {
            const int p = start + nodes_[f].length - 1;
            if (suffix_surface_[p] == f)
                suffix_surface_[p] = 1;
        }
    }
    void push_front(unsigned char c) {
        reserve_operation();
        const int old = empty() ? 1 : prefix_surface_.front();
        text_.push_front(c);
        prefix_surface_.push_front(1);
        suffix_surface_.push_front(1);
        const int v = extend(old, c, true), f = nodes_[v].link;
        prefix_surface_.front() = suffix_surface_[nodes_[v].length - 1] = v;
        if (nodes_[f].length > 0) {
            const int p = nodes_[v].length - nodes_[f].length;
            if (prefix_surface_[p] == f)
                prefix_surface_[p] = 1;
        }
    }
    unsigned char pop_back() {
        if (empty())
            throw std::out_of_range("pop from empty eertree");
        const int v = suffix_surface_.back(), f = nodes_[v].link, start = size() - nodes_[v].length;
        const int p = start + nodes_[f].length - 1;
        if (nodes_[f].length > 0 && nodes_[suffix_surface_[p]].length < nodes_[f].length)
            suffix_surface_[p] = prefix_surface_[start] = f;
        else
            prefix_surface_[start] = 1;
        remove_occurrence(v);
        auto c = text_.back();
        text_.pop_back();
        prefix_surface_.pop_back();
        suffix_surface_.pop_back();
        return c;
    }
    unsigned char pop_front() {
        if (empty())
            throw std::out_of_range("pop from empty eertree");
        const int v = prefix_surface_.front(), f = nodes_[v].link, end = nodes_[v].length - 1;
        const int p = nodes_[v].length - nodes_[f].length;
        if (nodes_[f].length > 0 && nodes_[prefix_surface_[p]].length < nodes_[f].length)
            prefix_surface_[p] = suffix_surface_[end] = f;
        else
            suffix_surface_[end] = 1;
        remove_occurrence(v);
        auto c = text_.front();
        text_.pop_front();
        prefix_surface_.pop_front();
        suffix_surface_.pop_front();
        return c;
    }
};
} // namespace cp
