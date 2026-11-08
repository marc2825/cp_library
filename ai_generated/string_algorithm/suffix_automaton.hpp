#pragma once
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <optional>
#include <stdexcept>
#include <string_view>
#include <utility>
#include <vector>
#include "detail/common.hpp"

namespace cp {
struct CommonSubstringResult {
    int first_left = 0, second_left = 0, length = 0;
    int first_right() const { return first_left + length; }
    int second_right() const { return second_left + length; }
};
// 任意byteのオンラインSAM。疎な遷移、空patternも可。
// docs/string_algorithm/suffix_automaton.md
class SuffixAutomaton {
public:
    struct State {
        std::map<unsigned char, int> next;
        int link = -1, length = 0, first_end = -1;
        bool clone = false;
    };
private:
    std::vector<State> states_{1};
    std::vector<int> prefixes_;
    int last_ = 0;
    mutable std::vector<std::uint64_t> counts_;
    int walk(std::string_view pattern) const {
        int v = 0;
        for (unsigned char c : pattern) {
            const auto it = states_[v].next.find(c);
            if (it == states_[v].next.end()) return -1;
            v = it->second;
        }
        return v;
    }
    void build_counts() const {
        if (counts_.size() == states_.size()) return;
        counts_.assign(states_.size(), 0);
        for (int v : prefixes_) ++counts_[v];
        std::vector<std::size_t> count(prefixes_.size() + 1), order(states_.size());
        for (const auto& s : states_) ++count[s.length];
        for (std::size_t i = 1; i < count.size(); ++i) count[i] += count[i - 1];
        for (std::size_t v = 0; v < states_.size(); ++v) order[--count[states_[v].length]] = v;
        for (auto i = order.size(); i-- > 0;) {
            const auto v = order[i];
            if (states_[v].link >= 0) counts_[states_[v].link] += counts_[v];
        }
    }
public:
    explicit SuffixAutomaton(std::string_view text = {}) { append(text); }
    int size() const { return static_cast<int>(prefixes_.size()); }
    std::size_t state_count() const { return states_.size(); }
    const std::vector<State>& states() const { return states_; }
    void clear() { states_.assign(1, State{}); prefixes_.clear(); counts_.clear(); last_ = 0; }
    void append(unsigned char c) {
        if (prefixes_.size() >= static_cast<std::size_t>((std::numeric_limits<int>::max() - 1) / 2))
            throw std::length_error("suffix automaton state indices exceed int range");
        counts_.clear();
        const int current = static_cast<int>(states_.size());
        states_.emplace_back(); states_[current].length = size() + 1; states_[current].first_end = size();
        int p = last_;
        while (p >= 0 && !states_[p].next.count(c)) { states_[p].next[c] = current; p = states_[p].link; }
        if (p < 0) states_[current].link = 0;
        else {
            const int q = states_[p].next.at(c);
            if (states_[p].length + 1 == states_[q].length) states_[current].link = q;
            else {
                const int clone = static_cast<int>(states_.size());
                State copied = states_[q]; copied.length = states_[p].length + 1; copied.clone = true;
                states_.push_back(std::move(copied));
                while (p >= 0) {
                    auto it = states_[p].next.find(c);
                    if (it == states_[p].next.end() || it->second != q) break;
                    it->second = clone; p = states_[p].link;
                }
                states_[q].link = states_[current].link = clone;
            }
        }
        last_ = current; prefixes_.push_back(current);
    }
    void append(std::string_view text) {
        if (text.size() > static_cast<std::size_t>((std::numeric_limits<int>::max() - 1) / 2) - prefixes_.size())
            throw std::length_error("suffix automaton input too long");
        for (unsigned char c : text) append(c);
    }
    bool contains(std::string_view pattern) const { return walk(pattern) >= 0; }
    std::optional<int> first_occurrence(std::string_view pattern) const {
        if (pattern.empty()) return 0;
        const int v = walk(pattern);
        if (v < 0) return std::nullopt;
        return states_[v].first_end + 1 - static_cast<int>(pattern.size());
    }
    std::uint64_t count_occurrences(std::string_view pattern) const {
        if (pattern.empty()) return prefixes_.size() + 1;
        const int v = walk(pattern); if (v < 0) return 0;
        build_counts(); return counts_[v];
    }
    std::uint64_t distinct_substrings() const {
        std::uint64_t count = 0;
        for (std::size_t v = 1; v < states_.size(); ++v) count += states_[v].length - states_[states_[v].link].length;
        return count;
    }
    // 同長ならsecond_left、次にfirst_leftが小さい解。空なら(0,0,0)。
    CommonSubstringResult longest_common_substring(std::string_view text) const {
        const int n = string_algorithm_detail::checked_size(text.size());
        int v = 0, length = 0; CommonSubstringResult result;
        for (int i = 0; i < n; ++i) {
            const auto c = static_cast<unsigned char>(text[i]);
            while (v && !states_[v].next.count(c)) { v = states_[v].link; length = states_[v].length; }
            const auto it = states_[v].next.find(c);
            if (it == states_[v].next.end()) { v = 0; length = 0; continue; }
            v = it->second; ++length;
            const int first = states_[v].first_end + 1 - length, second = i + 1 - length;
            if (length > result.length || (length == result.length && (second < result.second_left ||
                (second == result.second_left && first < result.first_left)))) result = {first, second, length};
        }
        return result;
    }
};
} // namespace cp
