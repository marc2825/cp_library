#include "combinatorial_optimization/all.hpp"
#include "game_theory/omega_rational.hpp"
#include <iostream>
using namespace cp;
using Q = BigRational;
std::vector<Q> rationals(int n) {
    std::vector<Q> v(n);
    for (auto &x : v)
        std::cin >> x;
    return v;
}
std::vector<int> integers(int n) {
    std::vector<int> v(n);
    for (auto &x : v)
        std::cin >> x;
    return v;
}
template <class T> void print(const std::vector<T> &v) {
    std::cout << v.size();
    for (const auto &x : v)
        std::cout << " " << x;
}
int main() {
    int cases;
    std::cin >> cases;
    while (cases--) {
        char code;
        std::cin >> code;
        if (code == 'L' || code == 'D' || code == 'I') {
            int n, m;
            std::cin >> n >> m;
            std::vector<std::vector<Q>> a;
            for (int i = 0; i < m; i++)
                a.push_back(rationals(n));
            auto b = rationals(m), c = rationals(n);
            if (code == 'L') {
                auto r = linear_program(a, b, c);
                std::cout << lp_status_name(r.status) << " " << r.value << " ";
                print(r.primal);
                std::cout << " ";
                print(r.dual);
            } else if (code == 'D') {
                std::vector<std::array<Q, 2>> rows;
                for (auto row : a)
                    rows.push_back({row[0], row[1]});
                auto r = low_dimensional_lp<2>(rows, b, {c[0], c[1]});
                std::cout << lp_status_name(r.status) << " " << r.value;
            } else {
                std::vector<BigInt> lo(n), hi(n);
                for (auto &x : lo)
                    std::cin >> x;
                for (auto &x : hi)
                    std::cin >> x;
                auto r = bounded_integer_program(a, b, c, lo, hi);
                std::cout << (r.has_solution ? "optimal" : "infeasible") << " " << r.value << " ";
                print(r.solution);
            }
        } else if (code == 'W') {
            int n;
            std::cin >> n;
            auto g1 = integers(n), g2 = integers(n), c1 = integers(3), c2 = integers(3);
            std::vector<long long> w(n);
            for (auto &x : w)
                std::cin >> x;
            auto r = weighted_matroid_intersection(w, PartitionMatroid(g1, c1), PartitionMatroid(g2, c2));
            std::cout << optimization_to_string(r.weight) << " " << r.by_size.size();
            for (auto s : r.by_size)
                std::cout << " " << optimization_to_string(s.weight);
        } else if (code == 'U') {
            int n, k;
            std::cin >> n >> k;
            std::vector<MatroidOracle> oracles;
            for (int j = 0; j < k; j++) {
                auto groups = integers(n), capacity = integers(3);
                oracles.emplace_back(PartitionMatroid(groups, capacity));
            }
            auto r = matroid_union(n, oracles);
            std::cout << r.cardinality << " ";
            print(r.assignment);
        } else if (code == 'S' || code == 'B') {
            int n;
            std::cin >> n;
            auto modular = rationals(n);
            std::vector<std::vector<Q>> edges;
            for (int i = 0; i < n; i++)
                edges.push_back(rationals(n));
            auto f = [&](const std::vector<int> &s) {
                std::vector<bool> in(n);
                Q value = 2;
                for (int e : s) {
                    in[e] = true;
                    value += modular[e];
                }
                for (int i = 0; i < n; i++)
                    for (int j = 0; j < n; j++)
                        if (in[i] && !in[j])
                            value += edges[i][j];
                return value;
            };
            if (code == 'S') {
                auto r = submodular_minimize(n, f);
                std::cout << r.value << " " << r.lower_bound << " ";
                print(r.selected);
            } else {
                auto weights = rationals(n);
                auto r = base_polyhedron_greedy(weights, f);
                std::cout << r.value << " ";
                print(r.base);
            }
        } else if (code == 'O') {
            char op;
            std::cin >> op;
            int n;
            std::cin >> n;
            auto p = rationals(n);
            std::cin >> n;
            auto q = rationals(n);
            std::cin >> n;
            auto r = rationals(n);
            std::cin >> n;
            auto s = rationals(n);
            OmegaRational a(p, q), b(r, s), c;
            if (op == '+')
                c = a + b;
            else if (op == '-')
                c = a - b;
            else if (op == '*')
                c = a * b;
            else
                c = a / b;
            std::cout << c.sign() << " ";
            print(c.numerator());
            std::cout << " ";
            print(c.denominator());
        }
        std::cout << "\n";
    }
}
