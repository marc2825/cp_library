#include "union_find/all.hpp"
#include "data_structure/disjoint_set_union.hpp"
#include "data_structure/UnionFind.hpp"
#include "graph/minimum_spanning_forest.hpp"
#include "test_values.hpp"

#include <algorithm>
#include <cassert>
#include <iostream>
#include <numeric>
#include <optional>
#include <random>
#include <utility>
#include <vector>

using Result = cp::PotentialMergeResult;
using union_find_test::ModValue;
using union_find_test::XorValue;

// 頂点のラベルを全置換する O(N) の独立した連結性オラクル。
struct NaivePartition {
    std::vector<int> label;
    explicit NaivePartition(int n) : label(n) { std::iota(label.begin(), label.end(), 0); }
    bool merge(int u, int v) {
        int a = label[u], b = label[v];
        if (a == b) return false;
        for (int& x : label) if (x == b) x = a;
        return true;
    }
    std::vector<std::vector<int>> groups() const {
        std::vector<std::vector<int>> result;
        std::vector<bool> used(label.size());
        for (int v = 0; v < static_cast<int>(label.size()); ++v) {
            if (used[v]) continue;
            result.push_back({});
            for (int u = v; u < static_cast<int>(label.size()); ++u)
                if (label[u] == label[v]) { result.back().push_back(u); used[u] = true; }
        }
        return result;
    }
};

static void check_partition(cp::DisjointSetUnion& actual, const NaivePartition& expected) {
    int n = static_cast<int>(expected.label.size());
    assert(actual.vertex_count() == n);
    auto groups = expected.groups();
    assert(actual.groups() == groups);
    assert(actual.component_count() == static_cast<int>(groups.size()));
    for (int u = 0; u < n; ++u) {
        int size = static_cast<int>(std::count(expected.label.begin(), expected.label.end(), expected.label[u]));
        assert(actual.size(u) == size && actual.root(u) == actual.leader(u));
        for (int v = 0; v < n; ++v) assert(actual.same(u,v) == (expected.label[u] == expected.label[v]));
    }
}

static void exhaustive_partition(cp::DisjointSetUnion actual, NaivePartition expected, int depth) {
    check_partition(actual, expected);
    if (depth == 0) return;
    for (int u = 0; u < actual.vertex_count(); ++u) for (int v = 0; v < actual.vertex_count(); ++v) {
        auto next = actual;
        auto naive = expected;
        assert(next.merge(u,v) == naive.merge(u,v));
        exhaustive_partition(std::move(next), std::move(naive), depth-1);
    }
}

// 採用済み制約を辺として保存し、毎回 BFS して相対値を求める。
template <class T>
struct ConstraintGraph {
    std::vector<std::vector<std::pair<int, T>>> adjacency;
    explicit ConstraintGraph(int n) : adjacency(n) {}
    std::vector<std::optional<T>> from(int start) const {
        std::vector<std::optional<T>> result(adjacency.size());
        std::vector<int> todo{start}; result[start] = T(0);
        for (std::size_t i = 0; i < todo.size(); ++i)
            for (const auto& [to, difference] : adjacency[todo[i]]) {
                if (result[to]) continue;
                result[to] = *result[todo[i]] + difference;
                todo.push_back(to);
            }
        return result;
    }
    Result merge(int u, int v, const T& difference) {
        auto existing = from(u)[v];
        if (existing) return *existing == difference ? Result::AlreadyConsistent : Result::Contradiction;
        adjacency[u].emplace_back(v, difference);
        adjacency[v].emplace_back(u, T(0)-difference);
        return Result::Merged;
    }
    std::vector<std::vector<int>> groups() const {
        std::vector<bool> used(adjacency.size());
        std::vector<std::vector<int>> result;
        for (int u = 0; u < static_cast<int>(adjacency.size()); ++u) {
            if (used[u]) continue;
            auto reachable = from(u); result.push_back({});
            for (int v = u; v < static_cast<int>(adjacency.size()); ++v)
                if (reachable[v]) { result.back().push_back(v); used[v] = true; }
        }
        return result;
    }
};

template <class T>
static void check_potential(cp::PotentializedDisjointSetUnion<T>& actual, const ConstraintGraph<T>& expected) {
    int n = static_cast<int>(expected.adjacency.size());
    assert(actual.vertex_count() == n);
    auto groups = expected.groups();
    assert(actual.groups() == groups && actual.component_count() == static_cast<int>(groups.size()));
    for (int u = 0; u < n; ++u) {
        auto differences = expected.from(u);
        int size = 0;
        for (int v = 0; v < n; ++v) {
            assert(actual.same(u,v) == differences[v].has_value());
            assert(actual.diff(u,v) == differences[v]);
            if (differences[v]) {
                assert(actual.potential(v) - actual.potential(u) == *differences[v]);
                ++size;
            }
        }
        assert(actual.size(u) == size && actual.potential(actual.leader(u)) == T(0));
    }
}

static void exhaustive_potential(cp::PotentializedDisjointSetUnion<long long> actual,
                                 ConstraintGraph<long long> expected, int depth) {
    check_potential(actual, expected);
    if (depth == 0) return;
    for (int u = 0; u < actual.vertex_count(); ++u) for (int v = 0; v < actual.vertex_count(); ++v)
        for (long long w = -1; w <= 1; ++w) {
            auto next = actual;
            auto naive = expected;
            assert(next.merge(u,v,w) == naive.merge(u,v,w));
            exhaustive_potential(std::move(next), std::move(naive), depth-1);
        }
}

static void randomized_partition() {
    std::mt19937 rng(3200);
    for (int trial = 0; trial < 300; ++trial) {
        int n = static_cast<int>(rng()%26);
        cp::DisjointSetUnion actual(n); NaivePartition expected(n);
        for (int step = 0; step < 150; ++step) {
            if (rng()%31 == 0) {
                n = static_cast<int>(rng()%26); actual.reset(n); expected = NaivePartition(n);
            }
            if (n) {
                int u = static_cast<int>(rng()%n), v = static_cast<int>(rng()%n);
                assert(actual.merge(u,v) == expected.merge(u,v));
                int count = actual.component_count();
                assert(!actual.merge(v,u) && actual.component_count() == count);
            }
            check_partition(actual, expected);
        }
    }
}

template <class T>
static void randomized_potential(unsigned seed) {
    std::mt19937 rng(seed);
    for (int trial = 0; trial < 250; ++trial) {
        int n = 1 + static_cast<int>(rng()%18);
        cp::PotentializedDisjointSetUnion<T> actual(n); ConstraintGraph<T> expected(n);
        for (int step = 0; step < 120; ++step) {
            int u = static_cast<int>(rng()%n), v = static_cast<int>(rng()%n);
            T weight(static_cast<int>(rng()%41)-20);
            auto difference = expected.from(u)[v];
            if (difference && rng()%3 == 0) weight = *difference;
            int count = actual.component_count();
            Result reference = expected.merge(u,v,weight), result = actual.merge(u,v,weight);
            assert(result == reference);
            assert(actual.component_count() == count - (result == Result::Merged));
            if (step%12 == 0 || result == Result::Contradiction) check_potential(actual, expected);
        }
        check_potential(actual, expected);
        actual.reset(0); check_potential(actual, ConstraintGraph<T>(0));
        actual.reset(3); check_potential(actual, ConstraintGraph<T>(3));
    }
}

static void edge_cases_and_compatibility() {
    cp::DisjointSetUnion empty;
    assert(empty.vertex_count() == 0 && empty.component_count() == 0 && empty.groups().empty());
    cp::PotentializedDisjointSetUnion<long long> weighted_empty;
    assert(weighted_empty.vertex_count() == 0 && weighted_empty.component_count() == 0 && weighted_empty.groups().empty());
    cp::PotentializedDisjointSetUnion<long long> weighted(4);
    assert(weighted.merge(1,2,-5) == Result::Merged);
    assert(weighted.merge(3,1,7) == Result::Merged); // サイズによって接続方向が反転する。
    assert(weighted.diff(3,2) == std::optional<long long>(2));
    assert(weighted.merge(2,3,-2) == Result::AlreadyConsistent);
    assert(weighted.merge(2,3,2) == Result::Contradiction);
    assert(weighted.merge(0,0,0) == Result::AlreadyConsistent);
    assert(weighted.merge(0,0,1) == Result::Contradiction);
    assert(!weighted.diff(0,1) && weighted.component_count() == 2);
    assert(weighted.merge(0,3,1000000000000LL) == Result::Merged);
    assert(weighted.diff(0,2) == std::optional<long long>(1000000000002LL));
    check_potential(weighted_empty, ConstraintGraph<long long>(0));

    unionfind old(4);
    assert(old.unite(0,1) && old.merge(2,3) && old.issame(0,1));
    assert(old.component_count() == 2 && old.size(1) == 2 && old.par[old.root(0)] == -1);
    assert(old.siz[old.root(0)] == 2);
    cp::Graph<long long> graph(4);
    graph.add_edge(0,1,3); graph.add_edge(1,2,1); graph.add_edge(0,2,10);
    auto forest = cp::kruskal(graph);
    assert(forest.cost == 4 && forest.component_count == 2);
}

static void stress() {
    constexpr int n = 200000;
    cp::DisjointSetUnion normal(n);
    cp::PotentializedDisjointSetUnion<long long> weighted(n);
    for (int v = 1; v < n; ++v) {
        assert(normal.merge(v,v-1));
        assert(weighted.merge(v,v-1,1000000000LL) == Result::Merged);
    }
    assert(normal.component_count() == 1 && normal.size(n-1) == n);
    assert(weighted.component_count() == 1 && weighted.size(n-1) == n);
    assert(*weighted.diff(n-1,0) == 1000000000LL*(n-1));
    assert(normal.groups()[0].size() == n && weighted.groups()[0].size() == n);
    // 圧縮前の木が最大限深くなるよう、同サイズの成分を順に併合する。
    normal.reset(n); weighted.reset(n);
    for (int width = 1; width < n; width *= 2)
        for (int left = 0; left + width < n; left += 2*width) {
            normal.merge(left,left+width);
            weighted.merge(left,left+width,width);
        }
    for (int v = n-1; v >= 0; --v) {
        assert(normal.same(0,v) && normal.size(v) == n);
        assert(*weighted.diff(0,v) == v);
    }
}

int main() {
    for (int n = 0; n <= 4; ++n) exhaustive_partition(cp::DisjointSetUnion(n),NaivePartition(n),4);
    exhaustive_potential(cp::PotentializedDisjointSetUnion<long long>(3),ConstraintGraph<long long>(3),3);
    randomized_partition();
    randomized_potential<long long>(3201);
    randomized_potential<ModValue>(3202);
    randomized_potential<XorValue>(3203);
    edge_cases_and_compatibility(); stress();
    std::cout << "union_find: exhaustive histories, 1050 randomized sequences, compatibility, and 200000 vertices passed\n";
}
