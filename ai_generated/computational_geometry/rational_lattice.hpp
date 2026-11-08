#pragma once
#include "rational/kernel.hpp"
namespace cp::geometry::rational {
namespace lattice_detail {
inline BigInt floor_div(const BigInt &a, const BigInt &b) {
    auto qr = divmod(a, b);
    return qr.first - (qr.second.sign() < 0 ? 1 : 0);
}
inline BigInt floor_sum(BigInt n, BigInt m, BigInt a, BigInt b) {
    BigInt result;
    while (true) {
        const auto qa = floor_div(a, m), qb = floor_div(b, m);
        result += qa * n * (n - 1) / 2 + qb * n;
        a -= qa * m;
        b -= qb * m;
        const BigInt maximum = a * n + b;
        if (maximum < m)
            return result;
        n = maximum / m;
        b = maximum % m;
        std::swap(a, m);
    }
}
struct Line {
    Scalar slope, intercept;
    Scalar at(const BigInt &x) const { return slope * Scalar(x) + intercept; }
};
inline BigInt sum_line(const Line &line, const BigInt &l, const BigInt &r) {
    const auto a = line.slope.numerator() * line.intercept.denominator(),
               b = line.intercept.numerator() * line.slope.denominator(),
               m = line.slope.denominator() * line.intercept.denominator();
    return floor_sum(r - l + 1, m, a, a * l + b);
}
struct Slab {
    BigInt l, r;
    Line lower, upper;
};
inline std::vector<Slab> slabs(const std::vector<Point> &p) {
    if (p.empty())
        return {};
    std::vector<Scalar> xs;
    for (const auto &v : p)
        xs.push_back(v.x);
    std::sort(xs.begin(), xs.end());
    xs.erase(std::unique(xs.begin(), xs.end()), xs.end());
    std::vector<Slab> out;
    for (std::size_t k = 1; k < xs.size(); ++k) {
        const auto left = ceil(xs[k - 1]), right = ceil(xs[k]) - 1;
        if (left > right)
            continue;
        const auto middle = (xs[k - 1] + xs[k]) / 2;
        bool have = false;
        Line low, high;
        for (std::size_t i = 0; i < p.size(); ++i) {
            const auto &a = p[i];
            const auto &b = p[(i + 1) % p.size()];
            if (a.x == b.x || middle <= std::min(a.x, b.x) || middle >= std::max(a.x, b.x))
                continue;
            const auto slope = (b.y - a.y) / (b.x - a.x);
            Line line{slope, a.y - slope * a.x};
            const auto value = slope * middle + line.intercept;
            if (!have || value < low.slope * middle + low.intercept)
                low = line;
            if (!have || value > high.slope * middle + high.intercept)
                high = line;
            have = true;
        }
        if (have)
            out.push_back({left, right, low, high});
    }
    const auto last = floor(xs.back());
    if (Scalar(last) == xs.back()) {
        bool have = false;
        Scalar lo, hi;
        for (const auto &v : p)
            if (v.x == xs.back()) {
                if (!have || v.y < lo)
                    lo = v.y;
                if (!have || v.y > hi)
                    hi = v.y;
                have = true;
            }
        out.push_back({last, last, {0, lo}, {0, hi}});
    }
    return out;
}
} // namespace lattice_detail
// Closed rational convex region; input vertex order immaterial. Exact floor-sum, no grid scan.
inline BigInt count_lattice_points(std::vector<Point> polygon) {
    polygon = convex_hull(std::move(polygon));
    BigInt count;
    for (const auto &slab : lattice_detail::slabs(polygon))
        count += lattice_detail::sum_line(slab.upper, slab.l, slab.r) +
                 lattice_detail::sum_line({-slab.lower.slope, -slab.lower.intercept}, slab.l, slab.r) +
                 slab.r - slab.l + 1;
    return count;
}
// Convex hull of all contained lattice points, enumerating at most max_columns integer x slices.
inline std::vector<Point> integer_hull(std::vector<Point> polygon, std::size_t max_columns = 1000000) {
    polygon = convex_hull(std::move(polygon));
    const auto slices = lattice_detail::slabs(polygon);
    BigInt columns;
    for (const auto &s : slices)
        columns += s.r - s.l + 1;
    if (columns > BigInt(max_columns))
        throw std::length_error("integer hull column budget exceeded");
    std::vector<Point> endpoints;
    for (const auto &s : slices)
        for (BigInt x = s.l; x <= s.r; ++x) {
            const auto lo = ceil(s.lower.at(x)), hi = floor(s.upper.at(x));
            if (lo <= hi) {
                endpoints.emplace_back(Scalar(x), Scalar(lo));
                if (lo != hi)
                    endpoints.emplace_back(Scalar(x), Scalar(hi));
            }
        }
    return convex_hull(std::move(endpoints));
}
} // namespace cp::geometry::rational
