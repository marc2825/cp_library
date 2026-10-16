#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <queue>
#include <random>
#include <stdexcept>
#include <tuple>
#include <utility>
#include <vector>
#include "graph_flow/all.hpp"

#define CHECK(x) do { if (!(x)) { std::cerr << "line " << __LINE__ << ": " #x "\n"; std::abort(); } } while (false)
using Edges = std::vector<std::tuple<int, int, long long>>;

long long brute_cut(int n, const Edges& edges, int s, int t) {
    long long best = std::numeric_limits<long long>::max();
    for (int mask = 0; mask < (1 << n); ++mask) {
        if (!(mask >> s & 1) || (mask >> t & 1)) continue;
        long long capacity = 0;
        for (auto [u, v, c] : edges) if ((mask >> u & 1) && !(mask >> v & 1)) capacity += c;
        best = std::min(best, capacity);
    }
    return best;
}

void check_flow(const cp::Dinic<>& network, const Edges& edges, int s, int t, long long total) {
    CHECK(network.edge_count() == static_cast<int>(edges.size()));
    const auto actual = network.edges();
    std::vector<long long> balance(network.size());
    std::vector<std::vector<int>> residual(network.size());
    for (int id = 0; id < network.edge_count(); ++id) {
        const auto e = network.get_edge(id);
        CHECK(e.from == std::get<0>(edges[id]) && e.to == std::get<1>(edges[id]));
        CHECK(e.capacity == std::get<2>(edges[id]) && e.flow >= 0 && e.flow <= e.capacity);
        CHECK(e.flow == actual[id].flow);
        balance[e.from] -= e.flow; balance[e.to] += e.flow;
        if (e.flow < e.capacity) residual[e.from].push_back(e.to);
        if (e.flow > 0) residual[e.to].push_back(e.from);
    }
    for (int v = 0; v < network.size(); ++v) CHECK(balance[v] == (v == s ? -total : v == t ? total : 0));
    const auto side = network.min_cut(s);
    std::vector<bool> reached(network.size(), false); reached[s] = true;
    std::vector<int> queue{s};
    for (std::size_t p = 0; p < queue.size(); ++p)
        for (int v : residual[queue[p]]) if (!reached[v]) { reached[v] = true; queue.push_back(v); }
    CHECK(side == reached);
}

void test_network(int n, Edges edges, int s, int t) {
    cp::Dinic<> network(n);
    for (auto [u, v, c] : edges) { const int id = network.edge_count(); CHECK(network.add_edge(u, v, c) == id); }
    CHECK(network.max_flow(s, t, 0) == 0);
    long long total = network.max_flow(s, t, 1);
    CHECK(total == std::min(1LL, brute_cut(n, edges, s, t)));
    check_flow(network, edges, s, t, total);
    total += network.max_flow(s, t, 2);
    CHECK(total == std::min(3LL, brute_cut(n, edges, s, t)));
    check_flow(network, edges, s, t, total);
    auto cut = cp::minimum_cut(network, s, t);
    const long long expected = brute_cut(n, edges, s, t);
    CHECK(cut.capacity == expected && cut.source_side[s] && !cut.source_side[t]);
    check_flow(network, edges, s, t, expected);
    std::vector<int> ids;
    for (int id = 0; id < network.edge_count(); ++id) {
        auto e = network.get_edge(id);
        if (cut.source_side[e.from] && !cut.source_side[e.to]) { ids.push_back(id); CHECK(e.flow == e.capacity); }
        if (!cut.source_side[e.from] && cut.source_side[e.to]) CHECK(e.flow == 0);
    }
    CHECK(ids == cut.edge_ids && network.max_flow(s, t) == 0);
    network.add_edge(s, t, 1); edges.emplace_back(s, t, 1);
    CHECK(network.max_flow(s, t) == 1);
    CHECK(cp::minimum_cut(network, s, t).capacity == expected + 1);
    CHECK(cut.capacity == expected); // Value snapshot survives later mutations.
    network.reset_flow();
    for (auto e : network.edges()) CHECK(e.flow == 0);
    CHECK(network.max_flow(t, s) == brute_cut(n, edges, t, s));
    check_flow(network, edges, t, s, brute_cut(n, edges, t, s));
}

int brute_matching(int left, int right, const std::vector<std::pair<int, int>>& edges) {
    std::vector<bool> dp(1 << right, false); dp[0] = true;
    for (int u = 0; u < left; ++u) {
        auto next = dp;
        for (int mask = 0; mask < (1 << right); ++mask) if (dp[mask])
            for (auto [a, b] : edges) if (a == u && !(mask >> b & 1)) next[mask | (1 << b)] = true;
        dp.swap(next);
    }
    int best = 0;
    for (int mask = 0; mask < (1 << right); ++mask) if (dp[mask]) {
        int count = 0; for (int v = 0; v < right; ++v) count += mask >> v & 1;
        best = std::max(best, count);
    }
    return best;
}

void test_matching(int left, int right, const std::vector<std::pair<int, int>>& edges) {
    cp::BipartiteMatching graph(left, right);
    cp::Dinic<int> flow(left + right + 2);
    const int s = left + right, t = s + 1;
    for (int u = 0; u < left; ++u) flow.add_edge(s, u, 1);
    for (int v = 0; v < right; ++v) flow.add_edge(left + v, t, 1);
    for (auto [u, v] : edges) { graph.add_edge(u, v); flow.add_edge(u, left + v, 1); }
    const auto result = graph.maximum_matching();
    CHECK(graph.left_size() == left && graph.right_size() == right);
    CHECK(result.cardinality == brute_matching(left, right, edges));
    CHECK(result.cardinality == flow.max_flow(s, t));
    CHECK(result.pairs.size() == static_cast<std::size_t>(result.cardinality));
    CHECK(result.left_match.size() == static_cast<std::size_t>(left));
    CHECK(result.right_match.size() == static_cast<std::size_t>(right));
    CHECK(std::is_sorted(result.pairs.begin(), result.pairs.end()));
    int count = 0;
    for (int u = 0; u < left; ++u) if (result.left_match[u] != -1) {
        int v = result.left_match[u]; CHECK(v >= 0 && v < right && result.right_match[v] == u);
        CHECK(std::find(edges.begin(), edges.end(), std::make_pair(u, v)) != edges.end()); ++count;
    }
    for (int v = 0; v < right; ++v) if (result.right_match[v] != -1) CHECK(result.left_match[result.right_match[v]] == v);
    CHECK(count == result.cardinality);
    const auto repeated = graph.maximum_matching();
    CHECK(repeated.pairs == result.pairs && repeated.left_match == result.left_match);
    if (left > 0 && right > 0) {
        graph.add_edge(0, 0);
        auto appended = edges; appended.emplace_back(0, 0);
        CHECK(graph.maximum_matching().cardinality == brute_matching(left, right, appended));
        CHECK(result.cardinality == brute_matching(left, right, edges));
    }
}

template<class Cap> void extremes() {
    const Cap maximum = std::numeric_limits<Cap>::max();
    cp::Dinic<Cap> graph(3);
    graph.add_edge(0, 0, maximum); graph.add_edge(0, 1, maximum); graph.add_edge(1, 2, maximum);
    CHECK(graph.max_flow(0, 2) == maximum);
    CHECK(graph.get_edge(1).flow == maximum && graph.get_edge(1).capacity == maximum);
    CHECK(cp::minimum_cut(graph, 0, 2).capacity == maximum);
    graph.reset_flow(); CHECK(graph.max_flow(0, 2, Cap{1}) == Cap{1});
    CHECK(cp::minimum_cut(graph, 0, 2).capacity == maximum);
    cp::Dinic<Cap> overflow(2);
    overflow.add_edge(0, 1, maximum); overflow.add_edge(0, 1, maximum);
    bool threw = false;
    try { (void)cp::minimum_cut(overflow, 0, 1); } catch (const std::overflow_error&) { threw = true; }
    CHECK(threw);
    // A later call can finish the residual flow, but the cut sum still cannot fit.
    CHECK(overflow.max_flow(0, 1) == maximum);
    threw = false;
    try { (void)cp::minimum_cut(overflow, 0, 1); } catch (const std::overflow_error&) { threw = true; }
    CHECK(threw);
}

int main() {
    CHECK(cp::Dinic<>().size() == 0);
    int flow_cases = 0, matching_cases = 0;
    for (int n : {2, 3}) {
        std::vector<std::pair<int, int>> pairs;
        for (int u = 0; u < n; ++u) for (int v = 0; v < n; ++v) if (u != v) pairs.emplace_back(u, v);
        int count = 1; for (std::size_t i = 0; i < pairs.size(); ++i) count *= 3;
        for (int code = 0; code < count; ++code) {
            int value = code; Edges edges;
            for (auto [u, v] : pairs) { edges.emplace_back(u, v, value % 3); value /= 3; }
            test_network(n, edges, 0, n - 1); ++flow_cases;
        }
    }
    for (int left = 0; left <= 3; ++left) for (int right = 0; right <= 3; ++right) {
        for (int mask = 0; mask < (1 << (left * right)); ++mask) {
            std::vector<std::pair<int, int>> edges;
            for (int u = 0; u < left; ++u) for (int v = 0; v < right; ++v) if (mask >> (u * right + v) & 1) edges.emplace_back(u, v);
            test_matching(left, right, edges); ++matching_cases;
        }
    }
    std::mt19937 rng(39009);
    for (int trial = 0; trial < 700; ++trial) {
        int n = 2 + rng() % 7; Edges edges;
        for (unsigned i = 0, m = rng() % 35; i < m; ++i) edges.emplace_back(rng() % n, rng() % n, rng() % 9);
        int s = rng() % n, t = (s + 1 + rng() % (n - 1)) % n;
        test_network(n, edges, s, t);
        int left = rng() % 8, right = rng() % 8;
        std::vector<std::pair<int, int>> matching;
        if (left && right) for (unsigned i = 0, m = rng() % 35; i < m; ++i) matching.emplace_back(rng() % left, rng() % right);
        test_matching(left, right, matching);
    }
    extremes<int>(); extremes<long long>(); extremes<unsigned long long>(); extremes<unsigned char>();
    const int n = 200000;
    cp::Dinic<> chain(n);
    for (int v = 1; v < n; ++v) chain.add_edge(v - 1, v, 7);
    CHECK(chain.max_flow(0, n - 1, 3) == 3 && chain.max_flow(0, n - 1) == 4);
    CHECK(cp::minimum_cut(chain, 0, n - 1).capacity == 7);
    cp::BipartiteMatching alternating(n, n);
    for (int u = 0; u + 1 < n; ++u) { alternating.add_edge(u, u); alternating.add_edge(u, u + 1); }
    alternating.add_edge(n - 1, 0);
    const auto result = alternating.maximum_matching();
    CHECK(result.cardinality == n && result.left_match[n - 1] == 0);
    for (int u = 0; u + 1 < n; ++u) CHECK(result.left_match[u] == u + 1);
    std::cout << flow_cases << " capacity assignments; " << matching_cases << " bipartite graphs; 700 random pairs; capacity extremes and 200k iterative paths passed\n";
}
