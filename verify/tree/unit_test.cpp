#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <limits>
#include <numeric>
#include <queue>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>
#include "tree/all.hpp"

#define CHECK(...) do { if (!(__VA_ARGS__)) { std::cerr << "check failed at " << __LINE__ << ": " << #__VA_ARGS__ << '\n'; std::abort(); } } while (false)
using G = cp::Graph<long long>;
std::mt19937 rng(311016);
int forests = 0, random_cases = 0;

struct Oracle {
    std::vector<int> parent, edge, depth, sub, component, pre, post, tin, tout, roots;
    std::vector<long long> dist;
    explicit Oracle(const G& g, int root) : parent(g.size(), -1), edge(g.size(), -1),
        depth(g.size()), sub(g.size(), 1), component(g.size(), -1), tin(g.size()), tout(g.size()), dist(g.size()) {
        std::function<void(int,int)> dfs = [&](int v, int id) {
            component[v] = id; tin[v] = static_cast<int>(pre.size()); pre.push_back(v);
            for (auto e : g[v]) if (e.to != parent[v]) {
                parent[e.to] = v; edge[e.to] = e.idx; depth[e.to] = depth[v] + 1; dist[e.to] = dist[v] + e.cost;
                dfs(e.to, id); sub[v] += sub[e.to];
            }
            tout[v] = static_cast<int>(pre.size()); post.push_back(v);
        };
        std::vector<int> starts;
        if (g.size()) starts.push_back(root);
        for (int v = 0; v < g.size(); ++v) starts.push_back(v);
        for (int v : starts) if (component[v] == -1) { roots.push_back(v); dfs(v, static_cast<int>(roots.size()) - 1); }
    }
};
std::vector<int> path(const G& g, int s, int t) {
    std::queue<int> q; q.push(s);
    std::vector<int> p(g.size(), -2); p[s] = -1;
    while (!q.empty()) { int v = q.front(); q.pop(); for (auto e : g[v]) if (p[e.to] == -2) { p[e.to] = v; q.push(e.to); } }
    if (p[t] == -2) return {};
    std::vector<int> out;
    for (int v = t; v != -1; v = p[v]) out.push_back(v);
    std::reverse(out.begin(), out.end()); return out;
}
long long path_cost(const G& g, const std::vector<int>& vs) {
    long long value = 0;
    for (std::size_t i = 1; i < vs.size(); ++i) {
        bool found = false;
        for (auto e : g[vs[i - 1]]) if (e.to == vs[i]) { value += e.cost; found = true; }
        CHECK(found);
    }
    return value;
}
struct Sum { long long count = 0, sum = 0; };
auto sum_merge = [](const Sum& a, const Sum& b) { return Sum{a.count + b.count, a.sum + b.sum}; };
auto sum_vertex = [](Sum s, int) { ++s.count; return s; };
auto sum_lift = [](Sum s, const cp::Edge<long long>& e) { s.sum += s.count * e.cost; return s; };

void check_forest(const G& g, int root, bool ordered) {
    const int n = g.size();
    Oracle want(g, root); cp::RootedTree<long long> tree(g, root);
    CHECK(tree.parent == want.parent && tree.parent_edge == want.edge && tree.depth == want.depth);
    CHECK(tree.subtree_size == want.sub && tree.component == want.component && tree.preorder == want.pre);
    CHECK(tree.postorder == want.post && tree.roots == want.roots && tree.tin == want.tin && tree.tout == want.tout && tree.distance_from_root == want.dist);
    cp::DoublingLCA<long long> lca(tree); cp::EulerTour<long long> tour(g, root);
    cp::BinaryLifting lift(tree.parent, n + 1);
    for (int v = 0; v < n; ++v) {
        CHECK(tree.subtree_interval(v) == std::make_pair(want.tin[v], want.tout[v]));
        int ancestor = v;
        for (int k = 0; k <= n + 1; ++k) {
            CHECK(lca.kth_ancestor(v, k) == ancestor && lift.jump(v, k) == ancestor);
            if (ancestor != -1) ancestor = want.parent[ancestor];
        }
        CHECK(lca.kth_ancestor(v, std::numeric_limits<long long>::max()) == -1);
        for (int u = 0; u < n; ++u) {
            auto vs = path(g, u, v);
            CHECK(lca.vertex_path(u, v) == vs);
            int w = -1;
            for (int a = u; a != -1 && w == -1; a = want.parent[a])
                for (int b = v; b != -1; b = want.parent[b]) if (a == b) { w = a; break; }
            CHECK(lca.lca(u, v) == w);
            CHECK(tree.is_ancestor(u, v) == (w == u));
            CHECK(tree.same_component(u, v) == !vs.empty());
            if (w != -1) {
                CHECK(lca.edge_distance(u, v) == static_cast<int>(vs.size()) - 1);
                CHECK(lca.distance(u, v) == path_cost(g, vs));
                int low = std::min(tour.first[u], tour.first[v]), high = std::max(tour.first[u], tour.first[v]);
                int best = tour.walk[low];
                for (int i = low; i <= high; ++i) if (want.depth[tour.walk[i]] < want.depth[best]) best = tour.walk[i];
                CHECK(best == w);
            }
            for (int k = 0; k <= static_cast<int>(vs.size()); ++k) CHECK(lca.jump(u, v, k) == (k < static_cast<int>(vs.size()) ? vs[k] : -1));
            for (int x = 0; x < n; ++x) CHECK(lca.is_on_path(u, v, x) == (std::find(vs.begin(), vs.end(), x) != vs.end()));
        }
    }
    std::vector<int> walk, edge_walk, first(n), last(n), in(g.edge_count()), out(g.edge_count());
    std::function<void(int)> dfs = [&](int v) {
        first[v] = static_cast<int>(walk.size()); walk.push_back(v);
        for (auto e : g[v]) if (e.to != want.parent[v]) {
            in[e.idx] = static_cast<int>(edge_walk.size()); edge_walk.push_back(e.idx);
            dfs(e.to);
            out[e.idx] = static_cast<int>(edge_walk.size()); edge_walk.push_back(e.idx); walk.push_back(v);
        }
        last[v] = static_cast<int>(walk.size()) - 1;
    };
    for (int v : want.roots) dfs(v);
    CHECK(tour.walk == walk && tour.edge_walk == edge_walk && tour.first == first && tour.last == last && tour.edge_in == in && tour.edge_out == out);
    auto down = cp::tree_dp(g, Sum{}, sum_merge, sum_vertex, sum_lift, root);
    auto all = cp::rerooting(g, Sum{}, sum_merge, sum_vertex, sum_lift, root);
    for (int v = 0; v < n; ++v) {
        long long total = 0, sub_total = 0, count = 0;
        for (int u = 0; u < n; ++u) {
            auto vs = path(g, v, u);
            if (!vs.empty()) { total += path_cost(g, vs); ++count; }
            if (tree.is_ancestor(v, u)) sub_total += path_cost(g, vs);
        }
        CHECK(down[v].count == want.sub[v] && down[v].sum == sub_total);
        CHECK(all[v].count == count && all[v].sum == total);
    }
    if (!ordered) return;
    struct Text { std::string value; }; // R != S, noncommutative merge, directional edge transform.
    auto merge = [](const std::string& a, const std::string& b) { return a + b; };
    auto vertex = [](const std::string& a, int v) { return Text{"(" + std::to_string(v) + a + ")"}; };
    auto transfer = [&](const Text& a, const cp::Edge<long long>& e) {
        const auto& original = g.edges()[e.idx];
        CHECK(e.cost == original.cost && ((e.from == original.from && e.to == original.to) || (e.to == original.from && e.from == original.to)));
        return "[" + std::to_string(e.idx) + ":" + std::to_string(e.from) + ">" + std::to_string(e.to) + a.value + "]";
    };
    std::function<Text(int,int)> direct = [&](int v, int p) {
        std::string s;
        for (auto e : g[v]) if (e.to != p) s += transfer(direct(e.to, v), {e.to, v, e.cost, e.idx});
        return vertex(s, v);
    };
    auto strings = cp::rerooting(g, std::string{}, merge, vertex, transfer, root);
    auto sub_strings = cp::tree_dp(g, std::string{}, merge, vertex, transfer, root);
    for (int v = 0; v < n; ++v) { CHECK(strings[v].value == direct(v, -1).value); CHECK(sub_strings[v].value == direct(v, want.parent[v]).value); }
}

void check_diameter(const G& g) {
    auto d = cp::tree_diameter(g);
    if (!g.size()) { CHECK(d.length == 0 && d.endpoints == std::make_pair(-1, -1) && d.vertices.empty() && d.edges.empty()); return; }
    long long best = 0;
    for (int u = 0; u < g.size(); ++u) for (int v = 0; v < g.size(); ++v) best = std::max(best, path_cost(g, path(g, u, v)));
    CHECK(d.length == best && path_cost(g, d.vertices) == best);
    CHECK(d.vertices.front() == d.endpoints.first && d.vertices.back() == d.endpoints.second && d.edges.size() + 1 == d.vertices.size());
    for (std::size_t i = 0; i < d.edges.size(); ++i) { auto e = g.edges()[d.edges[i]]; CHECK((e.from == d.vertices[i] && e.to == d.vertices[i + 1]) || (e.to == d.vertices[i] && e.from == d.vertices[i + 1])); }
}

template <class Exception, class Function> void throws(Function f) { bool caught = false; try { f(); } catch (const Exception&) { caught = true; } CHECK(caught); }
void boundaries() {
    G empty; check_forest(empty, 0, true); check_diameter(empty);
    int calls = 0;
    auto merge = [&](long long a, long long b) { ++calls; return a + b; };
    auto vertex = [&](long long a, int) { ++calls; return a + 1; };
    auto transfer = [&](long long a, const cp::Edge<long long>&) { ++calls; return a; };
    CHECK(cp::tree_dp(empty, 0LL, merge, vertex, transfer).empty());
    CHECK(cp::rerooting(empty, 0LL, merge, vertex, transfer).empty());
    CHECK(calls == 0);
    cp::BinaryLifting none({}); CHECK(none.size() == 0 && none.up.size() == 64);
    for (G invalid : {G(1, cp::GraphKind::Directed), G(1), G(2), G(3)}) {
        if (!invalid.directed()) {
            if (invalid.size() == 1) invalid.add_edge(0, 0);
            else if (invalid.size() == 2) { invalid.add_edge(0, 1); invalid.add_edge(0, 1); }
            else { invalid.add_edge(0, 1); invalid.add_edge(1, 2); invalid.add_edge(2, 0); }
        }
        throws<std::invalid_argument>([&] { cp::RootedTree<long long> t(invalid); });
        throws<std::invalid_argument>([&] { cp::DoublingLCA<long long> t(invalid); });
        throws<std::invalid_argument>([&] { cp::EulerTour<long long> t(invalid); });
        throws<std::invalid_argument>([&] { cp::tree_dp(invalid, Sum{}, sum_merge, sum_vertex, sum_lift); });
        throws<std::invalid_argument>([&] { cp::rerooting(invalid, Sum{}, sum_merge, sum_vertex, sum_lift); });
        throws<std::invalid_argument>([&] { cp::tree_diameter(invalid); });
    }
    throws<std::invalid_argument>([] { cp::tree_diameter(G(2)); });
    G over(3); over.add_edge(0, 1, std::numeric_limits<long long>::max() - 1); over.add_edge(1, 2, 1);
    throws<std::overflow_error>([&] { cp::RootedTree<long long> t(over); });
    G signed_g(3); signed_g.add_edge(0, 1, std::numeric_limits<long long>::lowest()); signed_g.add_edge(0, 2, -1);
    cp::DoublingLCA<long long> signed_lca(signed_g);
    throws<std::overflow_error>([&] { signed_lca.distance(1, 2); });
    cp::Graph<unsigned long long> ug(3); ug.add_edge(0, 1, 8); ug.add_edge(1, 2, 9); CHECK(cp::DoublingLCA<unsigned long long>(ug).distance(1, 2) == 9);
    cp::Graph<double> fg(2); fg.add_edge(0, 1, 0.5); CHECK(cp::tree_diameter(fg).length == 0.5);
    cp::RootedTree<long long> snapshot(G(1)); CHECK(snapshot.parent == std::vector<int>{-1});
}
void successors() {
    for (int n = 1; n <= 4; ++n) {
        int states = 1; for (int i = 0; i < n; ++i) states *= n + 1;
        for (int code = 0; code < states; ++code) {
            auto value = code; std::vector<int> next(n);
            for (int& v : next) { v = value % (n + 1) - 1; value /= n + 1; }
            cp::BinaryLifting dp(next, 37);
            for (int v = 0; v < n; ++v) {
                int at = v;
                for (int k = 0; k <= 37; ++k) { CHECK(dp.jump(v, k) == at); if (at != -1) at = next[at]; }
            }
        }
    }
    cp::BinaryLifting cycle({1, 2, 0});
    for (auto k : {std::uint64_t(0), std::uint64_t(1) << 63, std::numeric_limits<std::uint64_t>::max()}) CHECK(cycle.jump(0, k) == static_cast<int>(k % 3));
    CHECK(cp::BinaryLifting({-1}).jump(0, std::numeric_limits<std::uint64_t>::max()) == -1);
    cp::BinaryLifting zero({0}, 0); CHECK(zero.jump(0, 0) == 0 && zero.max_steps() == 0 && zero.up.size() == 1);
    throws<std::out_of_range>([&] { zero.jump(0, 1); });
    cp::BinaryLifting partial({0}, 8); throws<std::out_of_range>([&] { partial.jump(0, 9); });
}
void exhaustive() {
    for (int n = 1; n <= 5; ++n) {
        std::vector<std::pair<int,int>> edges;
        for (int u = 0; u < n; ++u) for (int v = u + 1; v < n; ++v) edges.emplace_back(u, v);
        for (int mask = 0; mask < (1 << edges.size()); ++mask) {
            G g(n); std::vector<int> comp(n); std::iota(comp.begin(), comp.end(), 0); bool forest = true;
            for (int i = 0; i < static_cast<int>(edges.size()); ++i) if (mask >> i & 1) {
                auto [u, v] = edges[i]; if (comp[u] == comp[v]) { forest = false; break; }
                int old = comp[v]; for (int& c : comp) if (c == old) c = comp[u];
                g.add_edge(u, v, i % 4);
            }
            if (!forest) continue;
            for (int root = 0; root < n; ++root) check_forest(g, root, true);
            if (g.edge_count() == n - 1) check_diameter(g);
            ++forests;
        }
    }
}
void randomized() {
    for (int trial = 0; trial < 300; ++trial) {
        const int n = 1 + rng() % 22; G g(n), positive(n);
        std::vector<int> perm(n); std::iota(perm.begin(), perm.end(), 0); std::shuffle(perm.begin(), perm.end(), rng);
        for (int i = 1; i < n; ++i) {
            int p = rng() % i; int a = perm[i], b = perm[p]; if (rng() % 2) std::swap(a, b);
            long long w = static_cast<int>(rng() % 19) - 9;
            positive.add_edge(a, b, std::abs(w)); if (rng() % 4) g.add_edge(a, b, w);
        }
        check_forest(g, rng() % n, true); check_diameter(positive); ++random_cases;
    }
}
void large() {
    const int n = 200000;
    G g(n); for (int v = 1; v < n; ++v) g.add_edge(v - 1, v);
    {
        cp::RootedTree<long long> tree(g); CHECK(tree.subtree_size[0] == n && tree.depth.back() == n - 1 && tree.postorder.front() == n - 1);
        cp::DoublingLCA<long long> lca(std::move(tree)); CHECK(lca.lca(n - 1, n / 2) == n / 2 && lca.jump(n - 1, 0, n - 1) == 0);
    }
    { cp::EulerTour<long long> tour(g); CHECK(tour.walk.size() == 2 * n - 1 && tour.edge_walk.size() == 2 * n - 2); }
    CHECK(cp::tree_diameter(g).length == n - 1);
    { auto dp = cp::tree_dp(g, Sum{}, sum_merge, sum_vertex, sum_lift); CHECK(dp[0].count == n && dp[0].sum == 1LL * n * (n - 1) / 2); }
    for (bool star : {false, true}) {
        if (star) { g = G(n); for (int v = 1; v < n; ++v) g.add_edge(0, v); }
        long long merges = 0, vertices = 0, lifts = 0;
        auto all = cp::rerooting(g, Sum{}, [&](Sum a, Sum b) { ++merges; return sum_merge(a, b); },
            [&](Sum a, int v) { ++vertices; return sum_vertex(a, v); }, [&](Sum a, const cp::Edge<long long>& e) { ++lifts; return sum_lift(a, e); });
        CHECK(merges < 9LL * n && vertices < 4LL * n && lifts < 4LL * n);
        for (int v : {0, n / 2, n - 1}) {
            long long want = star ? (v ? 2LL * n - 3 : n - 1) : 1LL * v * (v + 1) / 2 + 1LL * (n - v - 1) * (n - v) / 2;
            CHECK(all[v].count == n && all[v].sum == want);
        }
    }
    std::vector<int> next(n); for (int v = 0; v < n; ++v) next[v] = (v + 1) % n;
    cp::BinaryLifting jump(next); CHECK(jump.jump(0, std::numeric_limits<std::uint64_t>::max()) == static_cast<int>(std::numeric_limits<std::uint64_t>::max() % n));
}
int main() {
    boundaries(); successors(); exhaustive(); randomized(); large();
    std::cout << forests << " exhaustive forests (every root); " << random_cases << " random forests/trees; 700 successor maps; 64-bit bounds; ordered directional DP; 200000-vertex path/star: passed\n";
}
