#include "combinatorial_optimization/all.hpp"
#include "game_theory/omega_rational.hpp"
#include "numerical_analysis/all.hpp"
#include <functional>
#include <iostream>
#include <random>
#include <set>
using namespace cp;
using Q = BigRational;
#define CHECK(x)                                                                                             \
    do {                                                                                                     \
        if (!(x))                                                                                            \
            throw std::runtime_error(std::string("check line ") + std::to_string(__LINE__) + ": " #x);       \
    } while (false)
std::vector<int> elements(int mask, int n) {
    std::vector<int> s;
    for (int i = 0; i < n; i++)
        if (mask >> i & 1)
            s.push_back(i);
    return s;
}
Q dot(const std::vector<Q> &a, const std::vector<Q> &b) {
    Q value = 0;
    for (std::size_t i = 0; i < a.size(); i++)
        value += a[i] * b[i];
    return value;
}
void certificate(const std::vector<std::vector<Q>> &a, const std::vector<Q> &b, const std::vector<Q> &c,
                 const LinearProgramResult<Q> &r, bool free = false) {
    const auto n = c.size(), m = a.size();
    if (r.has_primal) {
        CHECK(r.primal.size() == n);
        if (!free)
            for (auto x : r.primal)
                CHECK(x >= 0);
        for (std::size_t i = 0; i < m; i++)
            CHECK(dot(a[i], r.primal) <= b[i]);
        CHECK(dot(c, r.primal) == r.value);
    }
    if (r.status == LPStatus::optimal || r.status == LPStatus::infeasible) {
        CHECK(r.dual.size() == m);
        for (auto y : r.dual)
            CHECK(y >= 0);
        for (std::size_t j = 0; j < n; j++) {
            Q z = 0;
            for (std::size_t i = 0; i < m; i++)
                z += r.dual[i] * a[i][j];
            const Q target = r.status == LPStatus::optimal ? c[j] : Q(0);
            CHECK(free ? z == target : z >= target);
        }
        if (r.status == LPStatus::optimal)
            CHECK(dot(b, r.dual) == r.value);
        else
            CHECK(dot(b, r.dual) < 0);
    }
    if (r.status == LPStatus::unbounded) {
        CHECK(r.has_primal);
        CHECK(r.ray.size() == n);
        if (!free)
            for (auto x : r.ray)
                CHECK(x >= 0);
        for (auto row : a)
            CHECK(dot(row, r.ray) <= 0);
        CHECK(dot(c, r.ray) > 0);
    }
}
std::optional<Q> vertices(std::vector<std::vector<Q>> a, std::vector<Q> b, const std::vector<Q> &c) {
    const auto n = c.size();
    for (std::size_t j = 0; j < n; j++) {
        std::vector<Q> row(n);
        row[j] = -1;
        a.push_back(row);
        b.push_back(0);
    }
    std::optional<Q> best;
    std::vector<std::size_t> ids;
    std::function<void(std::size_t)> search = [&](std::size_t start) {
        if (ids.size() == n) {
            Matrix<Q> matrix(n, n);
            std::vector<Q> rhs;
            for (std::size_t i = 0; i < n; i++) {
                rhs.push_back(b[ids[i]]);
                for (std::size_t j = 0; j < n; j++)
                    matrix(i, j) = a[ids[i]][j];
            }
            auto solved = gauss_jordan(matrix, rhs);
            if (solved.status != LinearSystemStatus::unique)
                return;
            for (std::size_t i = 0; i < a.size(); i++)
                if (dot(a[i], solved.particular) > b[i])
                    return;
            const auto value = dot(c, solved.particular);
            if (!best || value > *best)
                best = value;
            return;
        }
        for (std::size_t i = start; i < a.size(); i++) {
            ids.push_back(i);
            search(i + 1);
            ids.pop_back();
        }
    };
    search(0);
    return best;
}
int main() {
    std::mt19937 rng(20261028);
    for (int trial = 0; trial < 240; trial++) {
        const int n = rng() % 4, m = rng() % 6;
        std::vector<std::vector<Q>> a(m, std::vector<Q>(n));
        std::vector<Q> b(m), c(n);
        for (auto &row : a)
            for (auto &x : row)
                x = int(rng() % 7) - 3;
        for (auto &x : b)
            x = int(rng() % 9) - 4;
        for (auto &x : c)
            x = int(rng() % 7) - 3;
        for (int j = 0; j < n; j++) {
            std::vector<Q> row(n);
            row[j] = 1;
            a.push_back(row);
            b.push_back(4);
        }
        auto expected = vertices(a, b, c);
        auto r = linear_program(a, b, c);
        certificate(a, b, c, r);
        CHECK(r.status == (expected ? LPStatus::optimal : LPStatus::infeasible));
        if (expected)
            CHECK(r.value == *expected);
        std::vector<std::vector<long double>> af;
        for (auto row : a) {
            af.emplace_back();
            for (auto x : row)
                af.back().push_back(rational_to_long_double(x));
        }
        std::vector<long double> bf, cf;
        for (auto x : b)
            bf.push_back(rational_to_long_double(x));
        for (auto x : c)
            cf.push_back(rational_to_long_double(x));
        auto approx = linear_program(af, bf, cf);
        CHECK(approx.status == r.status);
        if (expected)
            CHECK(numerical::almost_equal(approx.value, rational_to_long_double(*expected), {1e-8L, 1e-8L}));
    }
    for (const auto &a : std::vector<std::vector<std::vector<Q>>>{{}, {{-1}}, {{0}}}) {
        std::vector<Q> b(a.size(), -1), c{1};
        auto r = linear_program(a, b, c);
        certificate(a, b, c, r);
    }
    for (const auto &c : std::vector<std::vector<Q>>{{1}, {0}, {-1}}) {
        std::vector<std::vector<Q>> a{{1}, {-1}};
        std::vector<Q> b{2, -2};
        auto r = free_linear_program(a, b, c);
        certificate(a, b, c, r, true);
        CHECK(r.value == Q(2) * c[0]);
    }
    auto free_ray = free_linear_program<Q>({}, {}, {-1});
    certificate({}, {}, {-1}, free_ray, true);
    const std::vector<std::vector<Q>> cycling{
        {Q(1, 2), Q(-11, 2), Q(-5, 2), 9}, {Q(1, 2), Q(-3, 2), Q(-1, 2), 1}, {1, 0, 0, 0}};
    auto bland = linear_program<Q>(cycling, {0, 0, 1}, {10, -57, -9, -24});
    CHECK(bland.status == LPStatus::optimal && bland.value == 1 && bland.pivots < 100);
    auto limited = linear_program<Q>({{1}}, {1}, {1}, {0, 0});
    CHECK(limited.status == LPStatus::iteration_limit && limited.has_primal);
    auto phase_limit = linear_program<Q>({{-1}}, {-1}, {1}, {0, 0});
    CHECK(phase_limit.status == LPStatus::iteration_limit && !phase_limit.has_primal);
    auto empty_bad = linear_program<Q>({{}}, {-1}, {});
    certificate({{}}, {-1}, {}, empty_bad);
    CHECK(empty_bad.status == LPStatus::infeasible);
    bool bad = false;
    try {
        linear_program<long double>({{1}}, {1}, {std::numeric_limits<long double>::infinity()});
    } catch (const std::invalid_argument &) {
        bad = true;
    }
    CHECK(bad);
    const auto largest = std::numeric_limits<long double>::max();
    CHECK(linear_program<long double>({{.5L}}, {largest}, {1}).status == LPStatus::numerical_failure);
    const auto interval = low_dimensional_lp<1>({{{1}}, {{-1}}}, {Q(-1, 2), 3}, {{1}});
    CHECK(interval.status == LPStatus::optimal && interval.value == Q(-1, 2));
    for (int trial = 0; trial < 80; trial++) {
        std::vector<std::array<Q, 2>> rows;
        std::vector<std::vector<Q>> a;
        std::vector<Q> b, c{int(rng() % 7) - 3, int(rng() % 7) - 3};
        for (int i = 0; i < 6; i++) {
            Q x = int(rng() % 7) - 3, y = int(rng() % 7) - 3;
            rows.push_back({x, y});
            a.push_back({x, y});
            b.push_back(int(rng() % 9) - 4);
        }
        const auto expected = free_linear_program(a, b, c);
        const auto actual = low_dimensional_lp<2>(rows, b, {c[0], c[1]});
        CHECK(actual.status == expected.status);
        if (actual.status == LPStatus::optimal)
            CHECK(actual.value == expected.value);
        if (actual.has_primal) {
            for (std::size_t i = 0; i < a.size(); i++)
                CHECK(dot(a[i], actual.primal) <= b[i]);
        }
        if (actual.status == LPStatus::unbounded) {
            for (auto row : a)
                CHECK(dot(row, actual.ray) <= 0);
            CHECK(dot(c, actual.ray) > 0);
        }
    }
    auto line = low_dimensional_lp<3>({{{1, -1, 0}}, {{-1, 1, 0}}, {{0, 0, 1}}, {{0, 0, -1}}}, {0, 0, 2, -2},
                                      {{1, -1, 1}});
    CHECK(line.status == LPStatus::optimal && line.value == 2);
    CHECK(low_dimensional_lp<2>({}, {}, {{0, 0}}, {0}).status == LPStatus::iteration_limit);
    for (int trial = 0; trial < 200; trial++) {
        const int n = rng() % 9;
        std::vector<int> g1(n), g2(n);
        std::vector<long long> w(n);
        std::vector<std::pair<int, int>> edges(n);
        for (int i = 0; i < n; i++) {
            g1[i] = rng() % 3;
            g2[i] = rng() % 3;
            w[i] = int(rng() % 17) - 8;
            edges[i] = {int(rng() % 4), int(rng() % 4)};
        }
        PartitionMatroid p(g1, {1, 2, 1}), q(g2, {1, 1, 2});
        GraphicMatroid graph(4, edges);
        for (int kind = 0; kind < 2; kind++) {
            MatroidOracle second = kind ? MatroidOracle(graph) : MatroidOracle(q);
            auto actual = weighted_matroid_intersection(w, p, second);
            std::vector<std::optional<OptimizationWide>> best(n + 1);
            for (int mask = 0; mask < (1 << n); mask++) {
                auto s = elements(mask, n);
                if (!p(s) || !second(s))
                    continue;
                OptimizationWide score = 0;
                for (int e : s)
                    score += w[e];
                if (!best[s.size()] || score > *best[s.size()])
                    best[s.size()] = score;
            }
            OptimizationWide maximum = 0;
            std::size_t count = 0;
            for (std::size_t k = 0; k < best.size(); k++)
                if (best[k]) {
                    CHECK(actual.by_size[k].weight == *best[k]);
                    CHECK(p(actual.by_size[k].selected) && second(actual.by_size[k].selected));
                    maximum = std::max(maximum, *best[k]);
                    ++count;
                }
            CHECK(actual.by_size.size() == count && actual.weight == maximum);
        }
    }
    auto extremes = weighted_matroid_intersection(
        std::vector<long long>{LLONG_MAX, LLONG_MIN}, [](auto s) { return s.size() <= 2; },
        [](auto s) { return s.size() <= 2; });
    CHECK(extremes.weight == OptimizationWide(LLONG_MAX));
    for (int trial = 0; trial < 100; trial++) {
        const int n = rng() % 7;
        std::vector<int> g(n);
        std::vector<std::pair<int, int>> edges(n);
        for (int i = 0; i < n; i++) {
            g[i] = rng() % 3;
            edges[i] = {int(rng() % 4), int(rng() % 4)};
        }
        std::vector<MatroidOracle> oracles{PartitionMatroid(g, {1, 1, 1}), GraphicMatroid(4, edges)};
        auto actual = matroid_partition(n, oracles);
        int best = 0;
        std::vector<std::vector<int>> sets(2);
        std::function<void(int, int)> enumerate = [&](int i, int size) {
            if (i == n) {
                if (oracles[0](sets[0]) && oracles[1](sets[1]))
                    best = std::max(best, size);
                return;
            }
            enumerate(i + 1, size);
            for (auto &s : sets) {
                s.push_back(i);
                enumerate(i + 1, size + 1);
                s.pop_back();
            }
        };
        enumerate(0, 0);
        CHECK(actual.cardinality == best && actual.complete == (best == n));
        for (int j = 0; j < 2; j++)
            CHECK(oracles[j](actual.sets[j]));
    }
    CHECK(matroid_union(0, {}).complete && !matroid_union(3, {}).complete);
    for (int trial = 0; trial < 100; trial++) {
        const int n = rng() % 7;
        std::vector<Q> modular(n), weights(n);
        std::vector<std::vector<int>> cut(n, std::vector<int>(n));
        for (int i = 0; i < n; i++) {
            modular[i] = int(rng() % 9) - 4;
            weights[i] = int(rng() % 9) - 4;
            for (int j = 0; j < n; j++)
                cut[i][j] = rng() % 4;
        }
        auto f = [&](const std::vector<int> &s) {
            std::vector<bool> in(n);
            Q v = 3;
            for (int e : s) {
                in[e] = true;
                v += modular[e];
            }
            for (int i = 0; i < n; i++)
                for (int j = 0; j < n; j++)
                    if (in[i] && !in[j])
                        v += cut[i][j];
            return v;
        };
        auto greedy = base_polyhedron_greedy(weights, f);
        std::vector<Q> indicator(n);
        for (int i = 0; i < n; i++)
            indicator[i] = i % 2;
        std::vector<int> odd;
        for (int i = 1; i < n; i += 2)
            odd.push_back(i);
        CHECK(lovasz_extension(indicator, f) == f(odd));
        Q best = f({});
        for (int mask = 0; mask < (1 << n); mask++) {
            auto s = elements(mask, n);
            Q value = f(s), bound = 0;
            for (int e : s)
                bound += greedy.base[e];
            CHECK(bound <= value - greedy.offset);
            best = std::min(best, value);
        }
        auto minimum = submodular_minimize(n, f);
        CHECK(minimum.status == SubmodularStatus::optimal && minimum.value == best &&
              minimum.lower_bound == best);
        CHECK(f(minimum.selected) == best);
        auto partial = submodular_minimize(n, f, {0, {}});
        CHECK(partial.lower_bound <= best && partial.value >= best);
        auto upper =
            submodular_greedy_cardinality(n, n, [&](const std::vector<int> &s) { return Q(s.size()); });
        CHECK(upper.value == n);
    }
    auto coverage = [](const std::vector<int> &s) {
        std::set<int> covered;
        for (int e : s) {
            covered.insert(e);
            covered.insert(e + 1);
        }
        return Q(covered.size());
    };
    CHECK(base_polyhedron_greedy({3, 2, 1}, coverage, false).value == 7);
    CHECK(submodular_greedy_cardinality(3, 2, coverage).value == 4);
    CHECK(submodular_greedy_cardinality(3, 0, coverage).selected.empty());
    auto modular_pair = [](const std::vector<int> &s) {
        Q value = 0;
        for (int e : s)
            value += e == 0 ? 2 : -1;
        return value;
    };
    const auto sfm_limit = submodular_minimize(2, modular_pair, {10, {0, 0}});
    CHECK(sfm_limit.status == SubmodularStatus::lp_limit && sfm_limit.lower_bound <= -1 &&
          sfm_limit.value >= -1);
    CHECK(bounded_integer_program({}, {}, {}, {}, {}).status == IntegerProgramStatus::optimal);
    CHECK(bounded_integer_program({}, {}, {1}, {2}, {1}).status == IntegerProgramStatus::infeasible);
    for (int trial = 0; trial < 120; trial++) {
        const int n = 1 + rng() % 3, m = rng() % 5;
        std::vector<std::vector<Q>> a(m, std::vector<Q>(n));
        std::vector<Q> b(m), c(n);
        std::vector<BigInt> lo(n, -2), hi(n, 2);
        for (auto &row : a)
            for (auto &x : row)
                x = int(rng() % 7) - 3;
        for (auto &x : b)
            x = int(rng() % 11) - 5;
        for (auto &x : c)
            x = int(rng() % 7) - 3;
        std::optional<Q> best;
        std::vector<Q> x(n);
        std::function<void(int)> enumerate = [&](int j) {
            if (j < n) {
                for (int k = -2; k <= 2; k++) {
                    x[j] = k;
                    enumerate(j + 1);
                }
                return;
            }
            for (int i = 0; i < m; i++)
                if (dot(a[i], x) > b[i])
                    return;
            Q value = dot(c, x);
            if (!best || value > *best)
                best = value;
        };
        enumerate(0);
        auto r = bounded_integer_program(a, b, c, lo, hi);
        CHECK(r.status == (best ? IntegerProgramStatus::optimal : IntegerProgramStatus::infeasible));
        if (best) {
            CHECK(r.value == *best && *r.upper_bound == *best);
            std::vector<Q> answer;
            for (auto z : r.solution)
                answer.emplace_back(z);
            for (int i = 0; i < m; i++)
                CHECK(dot(a[i], answer) <= b[i]);
        }
        auto partial = bounded_integer_program(a, b, c, lo, hi, {1, {}});
        if (best && partial.upper_bound)
            CHECK(*partial.upper_bound >= *best);
        if (partial.has_solution && best)
            CHECK(partial.value <= *best);
    }
    CHECK(bounded_integer_program({{2}}, {1}, {1}, {0}, {2}, {0, {}}).status ==
          IntegerProgramStatus::node_limit);
    CHECK(bounded_integer_program({{2}}, {1}, {1}, {0}, {2}, {10, {0, 0}}).status ==
          IntegerProgramStatus::lp_limit);
    for (int trial = 0; trial < 100; trial++) {
        std::vector<FractionalPoint> points;
        long double best = -100;
        for (int i = 0; i < 8; i++) {
            FractionalPoint p;
            p.numerator = int(rng() % 41) - 20;
            p.denominator = 1 + rng() % 9;
            p.solution = {static_cast<long double>(i)};
            best = std::max(best, p.numerator / p.denominator);
            points.push_back(p);
        }
        auto oracle = [&](long double lambda) -> std::optional<FractionalPoint> {
            auto best = points.front();
            for (auto p : points)
                if (p.numerator - lambda * p.denominator > best.numerator - lambda * best.denominator)
                    best = p;
            return best;
        };
        FractionalOptions options;
        options.denominator_lower_bound = 1;
        for (auto result :
             {fractional_dinkelbach(oracle, 0, options), fractional_bisection(oracle, -30, 30, options)}) {
            CHECK(result.status == FractionalStatus::converged ||
                  result.status == FractionalStatus::stagnation);
            CHECK(numerical::almost_equal(result.ratio, best, {1e-10L, 1e-10L}));
        }
    }
    auto none = [](long double) -> std::optional<FractionalPoint> { return std::nullopt; };
    CHECK(fractional_dinkelbach(none).status == FractionalStatus::infeasible);
    auto constant_ratio = [](long double) -> std::optional<FractionalPoint> {
        return FractionalPoint{3, 2, {}};
    };
    FractionalOptions zero_iterations;
    zero_iterations.max_iterations = 0;
    const auto ratio_limit = fractional_dinkelbach(constant_ratio, 0, zero_iterations);
    CHECK(ratio_limit.status == FractionalStatus::iteration_limit && !ratio_limit.has_solution);
    const auto bisect_limit = fractional_bisection(constant_ratio, 0, 3, zero_iterations);
    CHECK(bisect_limit.status == FractionalStatus::iteration_limit && bisect_limit.has_solution &&
          bisect_limit.ratio == 1.5L && bisect_limit.evaluations == 2);
    CHECK(fractional_bisection(constant_ratio, 2, 3).status == FractionalStatus::invalid_bracket);
    auto infinite_point = [](long double) -> std::optional<FractionalPoint> {
        return FractionalPoint{std::numeric_limits<long double>::infinity(), 1, {}};
    };
    CHECK(fractional_dinkelbach(infinite_point).status == FractionalStatus::non_finite);
    using W = OmegaRational;
    const auto omega = W::omega(), eps = W::infinitesimal();
    CHECK(omega > W(BigRational(BigInt("1000000000000000000000000000000000000"))));
    CHECK(eps > 0 && eps < W(Q(1, 1000000)));
    CHECK(omega * eps == 1);
    CHECK((omega * omega - 1) / (omega - 1) == omega + 1);
    CHECK((1 + eps).standard_part() == std::optional<Q>(1));
    CHECK(!omega.standard_part());
    CHECK((omega - omega).outcome() == PartizanOutcome::previous_wins);
    W moved_source = omega + eps;
    W moved(std::move(moved_source));
    CHECK(moved == omega + eps && moved_source == 0);
    CHECK(moved_source.standard_part() == std::optional<Q>(0));
    moved_source += 2;
    moved_source = std::move(moved);
    CHECK(moved_source == omega + eps && moved == 0);
    moved_source = std::move(moved_source);
    CHECK(moved_source == omega + eps);
    moved_source /= moved_source;
    CHECK(moved_source == 1);
    for (int trial = 0; trial < 200; trial++) {
        std::vector<Q> a(3), b(3), c(3);
        for (auto *p : {&a, &b, &c})
            for (auto &x : *p)
                x = int(rng() % 7) - 3;
        W x(a, {1}), y(b, {1}), z(c, {1});
        CHECK((x + y) + z == x + (y + z));
        CHECK(x * (y + z) == x * y + x * z);
        CHECK(x + y - y == x);
        if (y)
            CHECK(x / y * y == x);
        if (x < y)
            CHECK(-x > -y);
        CHECK(W(a, {2}) + W(a, {2}) == x);
    }
    bad = false;
    try {
        auto value = omega / W(0);
        (void)value;
    } catch (const std::domain_error &) {
        bad = true;
    }
    CHECK(bad);
    std::cout << "optimization: 240 LP vertex/certificate/float checks, 80 exact low-dimensional LP, 400 "
                 "weighted matroid and 100 union exhaustive comparisons, 100 submodular exhaustive minima, "
                 "120 integer enumerations, 200 ratios, 200 omega field identities passed\n";
}
