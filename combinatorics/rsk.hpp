#pragma once
// 行挿入の RSK 対応と逆対応。説明: docs/combinatorics/rsk.md
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <stdexcept>
#include <utility>
#include <vector>
#include "young_tableaux.hpp"
namespace cp {
template<class Upper, class Lower> struct RSKPair {
    YoungTableau<Lower> insertion;
    YoungTableau<Upper> recording;
};
template<class Upper, class Lower, class UpperCompare = std::less<Upper>, class LowerCompare = std::less<Lower>>
RSKPair<Upper, Lower> rsk_biword(const std::vector<std::pair<Upper, Lower>>& biword,
                               UpperCompare upper_less = UpperCompare{}, LowerCompare lower_less = LowerCompare{}) {
    RSKPair<Upper, Lower> result;
    for (std::size_t i = 0; i < biword.size(); ++i) {
        if (i && (upper_less(biword[i].first, biword[i - 1].first) ||
            (!upper_less(biword[i - 1].first, biword[i].first) && lower_less(biword[i].second, biword[i - 1].second))))
            throw std::invalid_argument("RSK biword must be lexicographically sorted");
        auto value = biword[i].second;
        std::size_t r = 0;
        for (; r < result.insertion.size(); ++r) {
            auto& row = result.insertion[r];
            auto it = std::upper_bound(row.begin(), row.end(), value, lower_less);
            if (it == row.end()) { row.push_back(value); break; }
            std::swap(value, *it);
        }
        if (r == result.insertion.size()) { result.insertion.push_back({value}); result.recording.emplace_back(); }
        result.recording[r].push_back(biword[i].first);
    }
    return result;
}
template<class T, class Compare = std::less<T>>
RSKPair<std::size_t, T> rsk_word(const std::vector<T>& word, Compare less = Compare{}) {
    std::vector<std::pair<std::size_t, T>> biword;
    biword.reserve(word.size());
    for (std::size_t i = 0; i < word.size(); ++i) biword.emplace_back(i + 1, word[i]);
    return rsk_biword(biword, std::less<std::size_t>{}, less);
}
template<class Upper, class Lower, class UpperCompare = std::less<Upper>, class LowerCompare = std::less<Lower>>
std::vector<std::pair<Upper, Lower>> inverse_rsk_biword(RSKPair<Upper, Lower> pair,
                                  UpperCompare upper_less = UpperCompare{}, LowerCompare lower_less = LowerCompare{}) {
    if (!is_semistandard_young_tableau(pair.insertion, lower_less) ||
        !is_semistandard_young_tableau(pair.recording, upper_less) ||
        young_tableau_shape(pair.insertion) != young_tableau_shape(pair.recording))
        throw std::invalid_argument("RSK needs semistandard tableaux of equal shape");
    std::vector<std::pair<Upper, Lower>> result;
    while (!pair.recording.empty()) {
        std::size_t chosen = 0;
        for (std::size_t r = 1; r < pair.recording.size(); ++r) {
            const auto& a = pair.recording[chosen].back(); const auto& b = pair.recording[r].back();
            if (upper_less(a, b) || (!upper_less(b, a) && pair.recording[r].size() > pair.recording[chosen].size())) chosen = r;
        }
        auto label = pair.recording[chosen].back();
        auto value = pair.insertion[chosen].back();
        pair.recording[chosen].pop_back(); pair.insertion[chosen].pop_back();
        if (pair.recording[chosen].empty()) { pair.recording.pop_back(); pair.insertion.pop_back(); }
        for (std::size_t r = chosen; r; --r) {
            auto& row = pair.insertion[r - 1];
            auto it = std::lower_bound(row.begin(), row.end(), value, lower_less);
            if (it == row.begin()) throw std::invalid_argument("invalid reverse RSK bump");
            std::swap(value, *--it);
        }
        result.emplace_back(label, value);
    }
    std::reverse(result.begin(), result.end()); return result;
}
template<class T, class Compare = std::less<T>>
std::vector<T> inverse_rsk_word(RSKPair<std::size_t, T> pair, Compare less = Compare{}) {
    if (!is_standard_young_tableau(pair.recording)) throw std::invalid_argument("word RSK recording tableau must be standard");
    const auto biword = inverse_rsk_biword(std::move(pair), std::less<std::size_t>{}, less);
    std::vector<T> word; word.reserve(biword.size());
    for (const auto& item : biword) word.push_back(item.second);
    return word;
}
inline RSKPair<std::size_t, std::size_t> rsk_matrix(const std::vector<std::vector<std::uint64_t>>& matrix,
                                                  std::size_t max_entries = 1000000) {
    const auto width = matrix.empty() ? 0 : matrix.front().size();
    std::size_t count = 0;
    for (const auto& row : matrix) {
        if (row.size() != width) throw std::invalid_argument("RSK matrix must be rectangular");
        for (auto x : row) {
            if (x > max_entries - count) throw std::length_error("RSK expanded biword exceeds budget");
            count += static_cast<std::size_t>(x);
        }
    }
    std::vector<std::pair<std::size_t, std::size_t>> biword; biword.reserve(count);
    for (std::size_t r = 0; r < matrix.size(); ++r) for (std::size_t c = 0; c < width; ++c)
        for (std::uint64_t k = 0; k < matrix[r][c]; ++k) biword.emplace_back(r + 1, c + 1);
    return rsk_biword(biword);
}
inline std::vector<std::vector<std::uint64_t>> inverse_rsk_matrix(RSKPair<std::size_t, std::size_t> pair,
                  std::size_t rows, std::size_t columns, std::size_t max_cells = 1000000) {
    if (rows > max_cells || (rows && columns > max_cells / rows)) throw std::length_error("RSK matrix exceeds cell budget");
    std::vector<std::vector<std::uint64_t>> result(rows, std::vector<std::uint64_t>(rows ? columns : 0));
    for (const auto& item : inverse_rsk_biword(std::move(pair))) {
        if (!item.first || !item.second || item.first > rows || item.second > columns)
            throw std::out_of_range("RSK labels outside matrix dimensions");
        ++result[item.first - 1][item.second - 1];
    }
    return result;
}
} // namespace cp
