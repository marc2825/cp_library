#include <algorithm>
#include <array>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <numeric>
#include <random>
#include <set>
#include <stdexcept>
#include <utility>
#include <vector>
#include "graph_combinatorics/all.hpp"
#define CHECK(x) do { if (!(x)) { std::cerr << "line " << __LINE__ << ": " #x "\n"; std::abort(); } } while(false)
using G = cp::Graph<long long>;

bool is_clique(const G& g, unsigned mask, bool independent = false) {
    for (int u = 0; u < g.size(); ++u) if (mask >> u & 1)
        for (int v = u + 1; v < g.size(); ++v) if (mask >> v & 1) {
            bool adjacent = false; for (auto e : g[u]) if (e.to == v) adjacent = true;
            if (adjacent == independent) return false;
        }
    return true;
}
unsigned to_mask(const std::vector<int>& vertices) {
    CHECK(std::is_sorted(vertices.begin(), vertices.end())); unsigned mask = 0;
    for (int v : vertices) { CHECK(!(mask >> v & 1)); mask |= 1U << v; }
    return mask;
}
int brute_coloring(const G& g) {
    if (!g.size()) return 0;
    std::vector<int> color(g.size(), -1);
    for (int k = 1; k <= g.size(); ++k) {
        auto search = [&](auto&& self, int v) -> bool {
            if (v == g.size()) return true;
            for (int c = 0; c < k; ++c) {
                bool ok = true; for (auto e : g[v]) if (e.to >= 0 && e.to < v && color[e.to] == c) ok = false;
                if (ok) { color[v] = c; if (self(self, v + 1)) return true; }
            }
            color[v] = -1; return false;
        };
        if (search(search, 0)) return k;
    }
    std::abort();
}
void test_simple(const G& g) {
    std::set<unsigned> all, maximal; std::set<std::array<int, 3>> triangles;
    int maximum = 0, independence = 0;
    for (unsigned mask = 1; mask < (1U << g.size()); ++mask) {
        int count = 0; for (int v = 0; v < g.size(); ++v) count += mask >> v & 1;
        if (is_clique(g, mask)) {
            all.insert(mask); maximum = std::max(maximum, count);
            bool extend = false;
            for (int v = 0; v < g.size(); ++v) if (!(mask >> v & 1) && is_clique(g, mask | (1U << v))) extend = true;
            if (!extend) maximal.insert(mask);
            if (count == 3) {
                std::array<int, 3> a{}; int p = 0; for (int v = 0; v < g.size(); ++v) if (mask >> v & 1) a[p++] = v;
                triangles.insert(a);
            }
        }
        if (is_clique(g, mask, true)) independence = std::max(independence, count);
    }
    std::set<unsigned> actual_all, actual_maximal;
    CHECK(cp::enumerate_cliques(g, [&](const auto& c) { CHECK(actual_all.insert(to_mask(c)).second); return true; }));
    CHECK(cp::enumerate_maximal_cliques(g, [&](const auto& c) { CHECK(actual_maximal.insert(to_mask(c)).second); return true; }));
    CHECK(actual_all == all && actual_maximal == maximal);
    int with_empty = 0;
    CHECK(cp::enumerate_cliques(g, [&](const auto&) { ++with_empty; return true; }, true));
    CHECK(with_empty == static_cast<int>(all.size()) + 1);
    int maximal_empty = 0;
    CHECK(cp::enumerate_maximal_cliques(g, [&](const auto&) { ++maximal_empty; return true; }, true));
    CHECK(maximal_empty == static_cast<int>(maximal.size()) + (g.size() == 0));
    std::set<std::array<int, 3>> actual_triangles;
    CHECK(cp::enumerate_triangles(g, [&](int u, int v, int w) { CHECK(actual_triangles.insert({u,v,w}).second); return true; }));
    CHECK(actual_triangles == triangles);
    const auto clique = cp::maximum_clique(g), independent = cp::maximum_independent_set(g);
    CHECK(static_cast<int>(clique.size()) == maximum && is_clique(g, to_mask(clique)));
    CHECK(static_cast<int>(independent.size()) == independence && is_clique(g, to_mask(independent), true));
    const auto coloring = cp::minimum_coloring(g);
    CHECK(coloring.chromatic_number == brute_coloring(g) && cp::chromatic_number(g) == coloring.chromatic_number);
    CHECK(coloring.color.size() == static_cast<std::size_t>(g.size()));
    CHECK(coloring.groups.size() == static_cast<std::size_t>(coloring.chromatic_number));
    std::vector<int> used(g.size());
    for (int c = 0; c < coloring.chromatic_number; ++c) for (int v : coloring.groups[c]) { CHECK(coloring.color[v] == c); ++used[v]; }
    for (int v = 0; v < g.size(); ++v) CHECK(used[v] == 1 && coloring.color[v] >= 0 && coloring.color[v] < coloring.chromatic_number);
    for (auto e : g.edges()) CHECK(coloring.color[e.from] != coloring.color[e.to]);
    int calls = 0;
    CHECK(!cp::enumerate_cliques(g, [&](const auto&) { ++calls; return false; }, true) && calls == 1);
    calls = 0;
    bool completed = cp::enumerate_triangles(g, [&](int,int,int) { ++calls; return false; });
    CHECK(completed == triangles.empty() && calls == (triangles.empty() ? 0 : 1));
    calls = 0;
    CHECK(!cp::enumerate_maximal_cliques(g, [&](const auto&) { ++calls; return false; }, true) && calls == 1);
}

template<int Mod> int brute_trees(const G& g, bool weighted) {
    if (!g.size()) return 0;
    long long answer = 0;
    for (unsigned mask = 0; mask < (1U << g.edge_count()); ++mask) {
        int bits = 0; for (int e = 0; e < g.edge_count(); ++e) bits += mask >> e & 1;
        if (bits != g.size() - 1) continue;
        std::vector<int> parent(g.size()); std::iota(parent.begin(), parent.end(), 0);
        auto root = [&](int v) { while (v != parent[v]) v = parent[v]; return v; };
        long long product = 1; bool ok = true;
        for (auto e : g.edges()) if (mask >> e.idx & 1) {
            int u = root(e.from), v = root(e.to); if (u == v) { ok = false; break; } parent[u] = v;
            long long value = weighted ? e.cost % Mod : 1; if (value < 0) value += Mod;
            product = product * value % Mod;
        }
        if (ok) answer = (answer + product) % Mod;
    }
    return static_cast<int>(answer);
}
template<int Mod> void test_trees(const G& g) {
    const int expected = brute_trees<Mod>(g, false), weighted = brute_trees<Mod>(g, true);
    CHECK(cp::count_spanning_trees<Mod>(g) == expected);
    CHECK(cp::count_weighted_spanning_trees<Mod>(g) == weighted);
    for (int v = 0; v < g.size(); ++v) {
        CHECK(cp::count_spanning_trees<Mod>(g,v) == expected);
        CHECK(cp::count_weighted_spanning_trees<Mod>(g,v) == weighted);
    }
}
long long brute_det(const std::vector<std::vector<long long>>& matrix, int mod) {
    int n = static_cast<int>(matrix.size()); std::vector<int> permutation(n); std::iota(permutation.begin(),permutation.end(),0); long long answer = 0;
    do {
        int inversions = 0; long long product = 1;
        for (int i = 0; i < n; ++i) {
            for (int j = i + 1; j < n; ++j) inversions += permutation[i] > permutation[j];
            long long value = matrix[i][permutation[i]] % mod; if (value < 0) value += mod;
            product = product * value % mod;
        }
        answer = (answer + (inversions % 2 ? -product : product)) % mod;
    } while (std::next_permutation(permutation.begin(),permutation.end()));
    return (answer + mod) % mod;
}

int main() {
    int exhaustive = 0;
    for (int n = 0; n <= 5; ++n) {
        std::vector<std::pair<int,int>> pairs;
        for (int u = 0; u < n; ++u) for (int v = u+1; v < n; ++v) pairs.emplace_back(u,v);
        for (unsigned mask = 0; mask < (1U << pairs.size()); ++mask) {
            G g(n); for (unsigned i = 0; i < pairs.size(); ++i) if (mask >> i & 1) g.add_edge(pairs[i].first,pairs[i].second,static_cast<int>(i)%5-2);
            test_simple(g); test_trees<2>(g); test_trees<101>(g); ++exhaustive;
        }
    }
    std::mt19937 rng(310001);
    for (int trial = 0; trial < 240; ++trial) {
        int n = rng()%10; G g(n);
        for (int u = 0; u < n; ++u) for (int v = u+1; v < n; ++v) if (rng()%3) g.add_edge(u,v,static_cast<int>(rng()%11)-5);
        test_simple(g);
    }
    for (int trial = 0; trial < 240; ++trial) {
        int n = rng()%7; G g(n);
        if (n) for (unsigned i=0,m=rng()%13;i<m;++i) g.add_edge(rng()%n,rng()%n,static_cast<int>(rng()%15)-7);
        test_trees<2>(g); test_trees<101>(g); test_trees<998244353>(g);
        int dim = rng()%7; std::vector<std::vector<long long>> matrix(dim,std::vector<long long>(dim));
        for (auto& row : matrix) for (auto& value : row) value=static_cast<int>(rng()%21)-10;
        CHECK(cp::determinant_mod<2>(matrix)==brute_det(matrix,2));
        CHECK(cp::determinant_mod<101>(matrix)==brute_det(matrix,101));
        CHECK(cp::determinant_mod<2147483647>(matrix)==brute_det(matrix,2147483647));
    }
    struct Opaque {};
    cp::Graph<Opaque> opaque(3); opaque.add_edge(0,1,{}); opaque.add_edge(1,2,{});
    CHECK(cp::maximum_independent_set(opaque).size()==2 && cp::count_spanning_trees(opaque)==1);
    cp::Graph<unsigned long long> extreme(2); extreme.add_edge(0,1,std::numeric_limits<unsigned long long>::max());
    CHECK(cp::count_weighted_spanning_trees<101>(extreme)==std::numeric_limits<unsigned long long>::max()%101);
    G signed_extreme(2); signed_extreme.add_edge(0,1,std::numeric_limits<long long>::min());
    CHECK(cp::count_weighted_spanning_trees<101>(signed_extreme)==(std::numeric_limits<long long>::min()%101+101)%101);
    G complete(64), empty(64);
    for(int u=0;u<64;++u)for(int v=u+1;v<64;++v)complete.add_edge(u,v);
    CHECK(cp::maximum_clique(complete).size()==64 && cp::maximum_independent_set(complete).size()==1);
    CHECK(cp::maximum_clique(empty).size()==1 && cp::maximum_independent_set(empty).size()==64);
    int calls=0; CHECK(!cp::enumerate_cliques(complete,[&](const auto&){return ++calls<100;})); CHECK(calls==100);
    calls=0; CHECK(cp::enumerate_maximal_cliques(complete,[&](const auto& c){CHECK(c.size()==64);++calls;return true;}));CHECK(calls==1);
    CHECK(cp::count_spanning_trees(complete)==cp::graph_combinatorics_detail::power_mod<998244353>(64,62));
    G boundary(22); CHECK(cp::minimum_coloring(boundary).chromatic_number==1);
    bool threw=false;try{(void)cp::minimum_coloring(G(23));}catch(const std::length_error&){threw=true;}CHECK(threw);
    threw=false;try{(void)cp::maximum_clique(G(65));}catch(const std::length_error&){threw=true;}CHECK(threw);
    const int n=200000;G star(n);for(int v=1;v<n;++v)star.add_edge(0,v);
    calls=0;CHECK(cp::enumerate_triangles(star,[&](int,int,int){++calls;return true;}));CHECK(calls==0);
    calls=0;CHECK(cp::enumerate_cliques(star,[&](const auto& c){CHECK(c.size()<=2);++calls;return true;}));CHECK(calls==2*n-1);
    std::cout<<exhaustive<<" simple graphs; 240 random structures; 240 multigraphs/matrices; 64-bit boundaries, n=22 coloring, Cayley and 200k sparse triangles passed\n";
}
