#pragma once
// Young 図形・半標準/標準盤・転置・降下・標準盤列挙。
// 説明: docs/combinatorics/young_tableaux.md
#include <algorithm>
#include <climits>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <stdexcept>
#include <type_traits>
#include <vector>
namespace cp {
using YoungShape = std::vector<int>;
template<class T> using YoungTableau = std::vector<std::vector<T>>;
inline bool is_young_shape(const YoungShape& shape) {
    for (std::size_t r = 0; r < shape.size(); ++r)
        if (shape[r] <= 0 || (r && shape[r] > shape[r - 1])) return false;
    return true;
}
inline std::size_t young_shape_size(const YoungShape& shape, std::size_t max_cells = 1000000) {
    if (!is_young_shape(shape)) throw std::invalid_argument("Young rows must be positive and nonincreasing");
    std::size_t n = 0;
    for (int length : shape) {
        if (static_cast<std::size_t>(length) > max_cells - n) throw std::length_error("Young shape exceeds cell budget");
        n += length;
    }
    return n;
}
inline YoungShape conjugate_young_shape(const YoungShape& shape, std::size_t max_cells = 1000000) {
    young_shape_size(shape, max_cells);
    if (shape.empty()) return {};
    if (shape.size() > static_cast<std::size_t>(INT_MAX)) throw std::length_error("too many Young rows");
    YoungShape result(shape.front());
    for (int length : shape) for (int c = 0; c < length; ++c) ++result[c];
    return result;
}
template<class T> YoungShape young_tableau_shape(const YoungTableau<T>& tableau) {
    YoungShape result;
    for (const auto& row : tableau) {
        if (row.size() > static_cast<std::size_t>(INT_MAX)) throw std::length_error("Young row exceeds int range");
        result.push_back(static_cast<int>(row.size()));
    }
    if (!is_young_shape(result)) throw std::invalid_argument("invalid Young tableau shape");
    return result;
}
template<class T, class Compare = std::less<T>>
bool is_semistandard_young_tableau(const YoungTableau<T>& tableau, Compare less = Compare{}) {
    for (std::size_t r = 0; r < tableau.size(); ++r) {
        if (tableau[r].empty() || (r && tableau[r].size() > tableau[r - 1].size())) return false;
        for (std::size_t c = 0; c < tableau[r].size(); ++c) {
            if (c && less(tableau[r][c], tableau[r][c - 1])) return false;
            if (r && !less(tableau[r - 1][c], tableau[r][c])) return false;
        }
    }
    return true;
}
template<class T> bool is_standard_young_tableau(const YoungTableau<T>& tableau) {
    static_assert(std::is_integral<T>::value, "standard tableau labels must be integral");
    if (!is_semistandard_young_tableau(tableau)) return false;
    std::size_t n = 0;
    for (const auto& row : tableau) n += row.size();
    std::vector<bool> seen(n);
    for (const auto& row : tableau) for (T value : row) {
        if (value < 1 || static_cast<std::uintmax_t>(value) > n) return false;
        const auto i = static_cast<std::size_t>(value) - 1;
        if (seen[i]) return false;
        seen[i] = true;
    }
    return true;
}
template<class T> YoungTableau<T> transpose_young_tableau(const YoungTableau<T>& tableau,
                                                        std::size_t max_cells = 1000000) {
    const auto shape = conjugate_young_shape(young_tableau_shape(tableau), max_cells);
    YoungTableau<T> result(shape.size());
    for (std::size_t c = 0; c < shape.size(); ++c) {
        result[c].reserve(shape[c]);
        for (int r = 0; r < shape[c]; ++r) result[c].push_back(tableau[r][c]);
    }
    return result;
}
template<class T> std::vector<std::size_t> young_tableau_descents(const YoungTableau<T>& tableau) {
    if (!is_standard_young_tableau(tableau)) throw std::invalid_argument("descents need a standard tableau");
    std::size_t n = 0;
    for (const auto& row : tableau) n += row.size();
    std::vector<std::size_t> rows(n + 1), result;
    for (std::size_t r = 0; r < tableau.size(); ++r) for (auto x : tableau[r]) rows[x] = r;
    for (std::size_t i = 1; i < n; ++i) if (rows[i] < rows[i + 1]) result.push_back(i);
    return result;
}
// callback(const YoungTableau<size_t>&)->bool。false で早期終了。盤は呼出し中だけ借用。
template<class Callback>
std::size_t enumerate_standard_young_tableaux(const YoungShape& shape, Callback callback,
                                            std::size_t max_tableaux = 100000,
                                            std::size_t max_cells = 1000000) {
    const auto n = young_shape_size(shape, max_cells);
    YoungTableau<std::size_t> tableau(shape.size());
    for (std::size_t r = 0; r < shape.size(); ++r) tableau[r].reserve(shape[r]);
    std::vector<std::size_t> next_row(n + 1), path(n);
    std::size_t depth = 0, count = 0;
    while (true) {
        if (depth == n) {
            if (count == max_tableaux) throw std::length_error("tableau enumeration exceeds output budget");
            ++count;
            if (!callback(static_cast<const YoungTableau<std::size_t>&>(tableau))) return count;
        } else {
            auto& r = next_row[depth];
            while (r < shape.size() && (tableau[r].size() == static_cast<std::size_t>(shape[r]) ||
                   (r && tableau[r].size() == tableau[r - 1].size()))) ++r;
            if (r < shape.size()) {
                path[depth] = r; tableau[r].push_back(depth + 1); ++r;
                next_row[++depth] = 0; continue;
            }
        }
        if (!depth) return count;
        tableau[path[--depth]].pop_back();
    }
}
} // namespace cp
