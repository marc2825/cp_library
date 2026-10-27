#pragma once
#include "detail/common.hpp"
#include <algorithm>
#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace cp {
// Vertex k denotes the prefix of length k; parent is its longest proper border.
class BorderTree {
    static constexpr int levels = std::numeric_limits<int>::digits;
    std::string text_;
    std::vector<std::array<int, levels>> up_{1};

  public:
    explicit BorderTree(std::string_view text = {}) { append(text); }
    int size() const { return static_cast<int>(text_.size()); }
    void append(unsigned char c) {
        if (text_.size() >= static_cast<std::size_t>(std::numeric_limits<int>::max() - 1))
            throw std::length_error("border tree too long");
        int k = up_.back()[0];
        if (!text_.empty()) {
            while (k && static_cast<unsigned char>(text_[k]) != c)
                k = up_[k][0];
            if (static_cast<unsigned char>(text_[k]) == c)
                ++k;
        }
        std::array<int, levels> row{};
        row[0] = k;
        for (int j = 1; j < levels; ++j)
            row[j] = up_[row[j - 1]][j - 1];
        text_.push_back(static_cast<char>(c));
        up_.push_back(row);
    }
    void append(std::string_view text) {
        const std::string copy(text);
        for (unsigned char c : copy)
            append(c);
    }
    const std::string &text() const { return text_; }
    int parent(int length) const {
        string_algorithm_detail::range(size(), length, length);
        return up_[length][0];
    }
    bool is_border(int border, int prefix) const {
        string_algorithm_detail::range(size(), border, prefix);
        int v = prefix;
        for (int j = levels; j-- > 0;)
            if (up_[v][j] > border)
                v = up_[v][j];
        return v == border || up_[v][0] == border;
    }
    std::vector<int> borders(int prefix, bool include_self = false, bool include_empty = false) const {
        string_algorithm_detail::range(size(), prefix, prefix);
        std::vector<int> out;
        for (int v = include_self ? prefix : parent(prefix); v; v = parent(v))
            out.push_back(v);
        if (include_empty)
            out.push_back(0);
        std::reverse(out.begin(), out.end());
        return out;
    }
    std::vector<int> periods(int prefix) const {
        string_algorithm_detail::range(size(), prefix, prefix);
        std::vector<int> out;
        if (!prefix)
            return out;
        for (int v = parent(prefix); v; v = parent(v))
            out.push_back(prefix - v);
        out.push_back(prefix);
        return out;
    }
    std::vector<std::uint64_t> prefix_occurrences() const {
        std::vector<std::uint64_t> out(text_.size() + 1, 1);
        for (int i = size(); i > 0; --i)
            out[parent(i)] += out[i];
        return out;
    }
};
inline std::vector<int> string_periods(std::string_view text) {
    return BorderTree(text).periods(static_cast<int>(text.size()));
}
} // namespace cp
