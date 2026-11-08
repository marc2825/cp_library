#pragma once
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>
#include <vector>
#include "../string_algorithm/z_algorithm.hpp"

namespace cp {
struct GrundyPeriod { std::size_t preperiod = 0, period = 1; };
// 観測列の末尾の反復候補。未来の周期性の証明ではない。
// 最小preperiod、次に最短period。O(N)。docs/game_theory/periodic_grundy.md
inline std::optional<GrundyPeriod> detect_grundy_period(const std::vector<int>& values, std::size_t min_repetitions = 3,
                                                       std::size_t max_period = std::numeric_limits<std::size_t>::max()) {
    if (min_repetitions < 2) throw std::invalid_argument("period detection requires at least two repetitions");
    for (int x : values) if (x < 0) throw std::invalid_argument("Grundy values must be nonnegative");
    const auto n = values.size();
    const auto z = z_algorithm(std::vector<int>(values.rbegin(), values.rend()));
    std::optional<GrundyPeriod> result;
    for (std::size_t p = 1; p <= std::min(max_period, n / min_repetitions); ++p) {
        const auto start = n - p - static_cast<std::size_t>(z[p]);
        if ((n - start) / p >= min_repetitions && (!result || start < result->preperiod)) result = GrundyPeriod{start, p};
    }
    return result;
}
struct SubtractionGrundyResult {
    std::vector<int> values;
    std::optional<GrundyPeriod> period; // 最大手M個の同一窓により証明済み。
    int value(std::uint64_t index) const {
        if (index < values.size()) return values[static_cast<std::size_t>(index)];
        if (!period) throw std::out_of_range("Grundy index exceeds observed values and no period was certified");
        if (!period->period || period->preperiod > values.size() || period->period > values.size() - period->preperiod)
            throw std::logic_error("invalid modified Grundy period record");
        return values[period->preperiod + static_cast<std::size_t>((index - period->preperiod) % period->period)];
    }
};
// 固定された有限の正の引き算手。max_terms個を計算し、同じM窓があれば周期を証明。
// 未検出はperiod=nullopt。空の手集合は全0。O(N*|moves|+N)、O(N+|moves|)。
inline SubtractionGrundyResult subtraction_grundy(std::vector<int> moves, std::size_t max_terms = 100000) {
    string_algorithm_detail::checked_size(max_terms);
    if (!max_terms) throw std::invalid_argument("at least g(0) must be computed");
    for (int move : moves) if (move <= 0) throw std::invalid_argument("subtraction moves must be positive");
    std::sort(moves.begin(), moves.end()); moves.erase(std::unique(moves.begin(), moves.end()), moves.end());
    if (moves.size() >= static_cast<std::size_t>(std::numeric_limits<int>::max())) throw std::length_error("too many subtraction moves");
    SubtractionGrundyResult result; result.values.assign(max_terms, 0);
    std::vector<std::size_t> seen(moves.size() + 1, max_terms);
    for (std::size_t n = 1; n < max_terms; ++n) {
        for (int move : moves) {
            if (static_cast<std::size_t>(move) > n) break;
            seen[result.values[n - move]] = n;
        }
        int value = 0; while (seen[value] == n) ++value;
        result.values[n] = value;
    }
    if (moves.empty()) { result.period = GrundyPeriod{0, 1}; return result; }
    const auto window = static_cast<std::size_t>(moves.back());
    const auto z = z_algorithm(std::vector<int>(result.values.rbegin(), result.values.rend()));
    for (std::size_t p = 1; p < max_terms; ++p) if (static_cast<std::size_t>(z[p]) >= window) {
        const auto start = max_terms - p - static_cast<std::size_t>(z[p]);
        if (!result.period || start < result.period->preperiod) result.period = GrundyPeriod{start, p};
    }
    return result;
}
} // namespace cp
