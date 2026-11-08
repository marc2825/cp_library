#pragma once
#include "../static_query/detail/seaweed_rank.hpp"
#include "detail/common.hpp"
#include <numeric>
#include <string_view>

namespace cp {
namespace string_algorithm_detail {
inline void seaweed_row(std::vector<int> &bottom, int &left, unsigned char c, std::string_view b) {
    for (std::size_t j = 0; j < b.size(); ++j)
        if (c == static_cast<unsigned char>(b[j]) || left > bottom[j])
            std::swap(left, bottom[j]);
}
inline void lcs_lengths(std::size_t m, std::size_t n) {
    if (n > static_cast<std::size_t>(std::numeric_limits<int>::max()) ||
        m > static_cast<std::size_t>(std::numeric_limits<int>::max()) - n)
        throw std::length_error("semi-local LCS too large");
}
} // namespace string_algorithm_detail
// Compact seaweed permutation: all four semi-local LCS families, O(mn) cell updates.
class SemiLocalLCS {
    int m_, n_;
    std::vector<int> seaweed_;
    seaweed_detail::RankIndex rank_;

  public:
    SemiLocalLCS(std::string_view a, std::string_view b) : m_(0), n_(0) {
        string_algorithm_detail::lcs_lengths(a.size(), b.size());
        m_ = static_cast<int>(a.size());
        n_ = static_cast<int>(b.size());
        std::vector<int> bottom(n_);
        std::iota(bottom.begin(), bottom.end(), 0);
        seaweed_.resize(a.size() + b.size());
        for (int i = 0; i < m_; ++i) {
            int left = -i - 1;
            string_algorithm_detail::seaweed_row(bottom, left, static_cast<unsigned char>(a[i]), b);
            seaweed_[n_ + m_ - i - 1] = left;
        }
        std::copy(bottom.begin(), bottom.end(), seaweed_.begin());
        rank_ = seaweed_detail::RankIndex(seaweed_);
    }
    int first_size() const { return m_; }
    int second_size() const { return n_; }
    const std::vector<int> &seaweed() const { return seaweed_; }
    int string_substring(int l, int r) const {
        string_algorithm_detail::range(n_, l, r);
        return rank_.score(l, r);
    }
    int substring_string(int l, int r) const {
        string_algorithm_detail::range(m_, l, r);
        return n_ - rank_.count(n_ + m_ - r, -l);
    }
    int prefix_suffix(int first_prefix, int second_start) const {
        string_algorithm_detail::range(m_, first_prefix, first_prefix);
        string_algorithm_detail::range(n_, second_start, second_start);
        return n_ - second_start - rank_.count(n_ + m_ - first_prefix, second_start);
    }
    int suffix_prefix(int first_start, int second_prefix) const {
        string_algorithm_detail::range(m_, first_start, first_start);
        string_algorithm_detail::range(n_, second_prefix, second_prefix);
        return second_prefix - rank_.count(second_prefix, -first_start);
    }
};
// Retain one bottom-boundary kernel for each prefix of a. Explicit O((m+1)n) cell budget.
class PrefixSubstringLCS {
    int m_, n_;
    std::vector<seaweed_detail::RankIndex> prefixes_;

  public:
    PrefixSubstringLCS(std::string_view a, std::string_view b, std::size_t max_cells = 4000000)
        : m_(0), n_(0) {
        string_algorithm_detail::lcs_lengths(a.size(), b.size());
        if ((b.size() && a.size() + 1 > max_cells / b.size()) || a.size() + 1 > max_cells)
            throw std::length_error("prefix-substring LCS cell budget exceeded");
        m_ = static_cast<int>(a.size());
        n_ = static_cast<int>(b.size());
        std::vector<int> bottom(n_);
        std::iota(bottom.begin(), bottom.end(), 0);
        prefixes_.reserve(a.size() + 1);
        prefixes_.emplace_back(bottom);
        for (int i = 0; i < m_; ++i) {
            int left = -i - 1;
            string_algorithm_detail::seaweed_row(bottom, left, static_cast<unsigned char>(a[i]), b);
            prefixes_.emplace_back(bottom);
        }
    }
    int query(int prefix, int l, int r) const {
        string_algorithm_detail::range(m_, prefix, prefix);
        string_algorithm_detail::range(n_, l, r);
        return prefixes_[prefix].score(l, r);
    }
};
} // namespace cp
