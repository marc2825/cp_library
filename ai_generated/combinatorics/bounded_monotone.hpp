#pragma once
// 上下境界付き非減少列 (strict=true なら狭義増加)。長方形分割＋畳み込み。
// 説明: docs/combinatorics/bounded_monotone.md
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <utility>
#include <vector>
#include "../fps/detail/common.hpp"
#include "../basic_utility/big_integer.hpp"
namespace cp {
namespace bounded_monotone_detail {
template<int Mod, int Root> class Corridor {
    using F = fps_detail::Field<Mod>;
    struct Edges { std::vector<int> east, north; };
    std::vector<std::size_t> low_, high_;
    std::vector<int> fac_, ifac_;
    std::vector<int> parallel(const std::vector<int>& source, std::size_t distance) const {
        std::vector<int> kernel(source.size());
        for (std::size_t t = 0; t < kernel.size(); ++t)
            kernel[t] = F::mul(fac_[distance - 1 + t], F::mul(ifac_[distance - 1], ifac_[t]));
        auto result = fps_detail::multiply<Mod, Root>(source, kernel);
        result.resize(source.size()); return result;
    }
    std::vector<int> cross(const std::vector<int>& source, std::size_t width) const {
        const auto height = source.size();
        std::vector<int> a(height), kernel(fac_.begin(), fac_.begin() + height + width - 1);
        for (std::size_t j = 0; j < height; ++j) a[j] = F::mul(source[j], ifac_[height - 1 - j]);
        const auto product = fps_detail::multiply<Mod, Root>(a, kernel);
        std::vector<int> result(width);
        for (std::size_t i = 0; i < width; ++i) result[i] = F::mul(product[height - 1 + i], ifac_[i]);
        return result;
    }
    Edges solve(std::size_t x, std::size_t y, std::size_t w, std::size_t h,
                const std::vector<int>& west, const std::vector<int>& south) const {
        if (low_[x] > y + h - 1 || high_[x + w - 1] < y)
            return {std::vector<int>(h), std::vector<int>(w)};
        if (w <= 256 / h) {
            auto row = south;
            std::vector<int> east(h);
            for (std::size_t j = 0; j < h; ++j) {
                int left = west[j];
                for (std::size_t i = 0; i < w; ++i) {
                    row[i] = low_[x + i] <= y + j && y + j <= high_[x + i] ? F::add(left, row[i]) : 0;
                    left = row[i];
                }
                east[j] = row.back();
            }
            return {std::move(east), std::move(row)};
        }
        if (low_[x + w - 1] <= y && y + h - 1 <= high_[x]) {
            auto east = parallel(west, w), north = parallel(south, h);
            const auto e = cross(south, h), n = cross(west, w);
            for (std::size_t i = 0; i < h; ++i) east[i] = F::add(east[i], e[i]);
            for (std::size_t i = 0; i < w; ++i) north[i] = F::add(north[i], n[i]);
            return {std::move(east), std::move(north)};
        }
        if (w >= h) {
            const auto k = w / 2;
            auto a = solve(x, y, k, h, west, {south.begin(), south.begin() + k});
            auto b = solve(x + k, y, w - k, h, a.east, {south.begin() + k, south.end()});
            a.north.insert(a.north.end(), b.north.begin(), b.north.end());
            return {std::move(b.east), std::move(a.north)};
        }
        const auto k = h / 2;
        auto a = solve(x, y, w, k, {west.begin(), west.begin() + k}, south);
        auto b = solve(x, y + k, w, h - k, {west.begin() + k, west.end()}, a.north);
        a.east.insert(a.east.end(), b.east.begin(), b.east.end());
        return {std::move(a.east), std::move(b.north)};
    }
public:
    Corridor(std::vector<std::size_t> low, std::vector<std::size_t> high, std::size_t height)
        : low_(std::move(low)), high_(std::move(high)), fac_(low_.size() + height, 1), ifac_(fac_.size(), 1) {
        for (std::size_t i = 1; i < fac_.size(); ++i) fac_[i] = F::mul(fac_[i - 1], static_cast<int>(i));
        ifac_.back() = F::inv(fac_.back());
        for (std::size_t i = fac_.size() - 1; i; --i) ifac_[i - 1] = F::mul(ifac_[i], static_cast<int>(i));
    }
    int count(std::size_t height) const {
        std::vector<int> west(height), south(low_.size()); west[0] = 1;
        return solve(0, 0, low_.size(), height, west, south).east.back();
    }
};
} // namespace bounded_monotone_detail
template<int Mod = 998244353, int Root = 3>
int count_bounded_monotone_sequences(const std::vector<long long>& lower,
                                    const std::vector<long long>& upper, bool strict = false,
                                    std::size_t max_extent = 1000000) {
    if (lower.size() != upper.size()) throw std::invalid_argument("boundary sizes differ");
    const auto n = lower.size();
    if (!n) return 1;
    if (n >= max_extent) throw std::length_error("corridor exceeds extent budget");
    std::vector<BigInt> lo(n), hi(n);
    for (std::size_t i = 0; i < n; ++i) {
        lo[i] = BigInt(lower[i]) - BigInt(strict ? i : 0);
        hi[i] = BigInt(upper[i]) - BigInt(strict ? i : 0);
        if (i) lo[i] = std::max(lo[i], lo[i - 1]);
    }
    for (std::size_t i = n - 1; i; --i) hi[i - 1] = std::min(hi[i - 1], hi[i]);
    for (std::size_t i = 0; i < n; ++i) if (lo[i] > hi[i]) return 0;
    const BigInt extent = hi.back() - lo.front() + BigInt(n) + BigInt(2);
    if (extent > BigInt(max_extent)) throw std::length_error("corridor exceeds extent budget");
    const auto size = extent.to_integral<std::size_t>(), height = size - n - 1;
    fps_detail::check_precision<Mod, Root>(size);
    if (size > static_cast<std::size_t>(Mod)) throw std::domain_error("corridor factorial indices exceed characteristic");
    std::vector<std::size_t> bottom(n + 1), top(n + 1);
    for (std::size_t i = 0; i <= n; ++i) {
        bottom[i] = (lo[i ? i - 1 : 0] - lo.front()).template to_integral<std::size_t>();
        top[i] = (hi[std::min(i, n - 1)] - lo.front()).template to_integral<std::size_t>();
    }
    return bounded_monotone_detail::Corridor<Mod, Root>(std::move(bottom), std::move(top), height).count(height);
}
} // namespace cp
