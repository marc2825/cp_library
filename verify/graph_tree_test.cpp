#include "graph/all.hpp"
#include "tree/all.hpp"
#include "graph_theory/graph_template.hpp"
#include "graph_theory/Tree/tree_template.hpp"
#include "graph_theory/dijkstra.hpp"
#include "graph_theory/bfs.hpp"
#include "graph_theory/grid_bfs.hpp"
#include "data_structure/UnionFind.hpp"

#include <cassert>
#include <iostream>
#include <limits>
#include <numeric>
#include <random>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>

using ll = long long;
static std::mt19937 rng(2825);
static int random_int(int n) { return static_cast<int>(rng() % n); }
constexpr ll oracle_inf = 1LL << 50;

// 削除した頂点/辺を除いて、隣接リストを直接走査する独立した連結性オラクル。
template <class T>
std::vector<int> labels(const cp::Graph<T>& g, int removed_vertex = -1, int removed_edge = -1) {
    std::vector<int> result(g.size(), -1);
    int id = 0;
    for (int root = 0; root < g.size(); ++root) {
        if (root == removed_vertex || result[root] != -1) continue;
        std::vector<int> todo{root}; result[root] = id;
        for (std::size_t i = 0; i < todo.size(); ++i)
            for (const auto& e : g[todo[i]]) {
                if (e.idx == removed_edge || e.to == removed_vertex || result[e.to] != -1) continue;
                result[e.to] = id; todo.push_back(e.to);
            }
        ++id;
    }
    return result;
}
static int group_count(const std::vector<int>& values) {
    return values.empty() ? 0 : *std::max_element(values.begin(), values.end()) + 1;
}

static std::vector<std::vector<ll>> oracle_distances(const cp::Graph<ll>& g) {
    int n = g.size();
    std::vector<std::vector<ll>> distance(n, std::vector<ll>(n, oracle_inf));
    for (int v = 0; v < n; ++v) {
        distance[v][v] = 0;
        for (const auto& e : g[v]) distance[v][e.to] = std::min(distance[v][e.to], e.cost);
    }
    for (int k = 0; k < n; ++k) for (int s = 0; s < n; ++s) for (int t = 0; t < n; ++t)
        if (distance[s][k] != oracle_inf && distance[k][t] != oracle_inf)
            distance[s][t] = std::min(distance[s][t], distance[s][k] + distance[k][t]);
    return distance;
}

template <class T>
static void check_path(const cp::Graph<ll>& g, const cp::ShortestPathResult<T>& result, int target,
                       bool count_edges = false) {
    auto vertices = result.vertex_path(target), edges = result.edge_path(target);
    if (!result.has_finite_distance(target)) { assert(vertices.empty() && edges.empty()); return; }
    assert(!vertices.empty() && vertices.front() == result.source && vertices.back() == target);
    assert(vertices.size() == edges.size() + 1 && vertices.size() <= static_cast<std::size_t>(g.size()));
    ll cost = 0;
    for (std::size_t i = 0; i < edges.size(); ++i) {
        const auto& e = g.edges().at(edges[i]);
        bool forward = e.from == vertices[i] && e.to == vertices[i+1];
        bool backward = !g.directed() && e.to == vertices[i] && e.from == vertices[i+1];
        assert(forward || backward);
        cost += count_edges ? 1 : e.cost;
    }
    assert(cost == result.dist[target]);
}

static void shortest_paths() {
    for (int iteration = 0; iteration < 450; ++iteration) {
        int n = 1 + random_int(7), mode = iteration % 3;
        cp::Graph<ll> g(n, iteration % 2 ? cp::GraphKind::Directed : cp::GraphKind::Undirected);
        for (int i = random_int(16); i > 0; --i)
            g.add_edge(random_int(n), random_int(n), mode == 0 ? random_int(9)-4 : mode == 1 ? random_int(2) : random_int(10));
        auto expected = oracle_distances(g);
        auto all = cp::floyd_warshall(g);
        for (int s = 0; s < n; ++s) {
            auto bellman = cp::bellman_ford(g, s);
            auto breadth = cp::bfs(g, s);
            auto depth = cp::depth_first_search(g, s);
            for (int t = 0; t < n; ++t) {
                bool reachable = expected[s][t] != oracle_inf, negative = false;
                for (int k = 0; k < n; ++k)
                    negative |= expected[s][k] != oracle_inf && expected[k][k] < 0 && expected[k][t] != oracle_inf;
                assert(bellman.reachable(t) == reachable && all[s].reachable(t) == reachable);
                assert(bellman.negative_cycle[t] == negative && all[s].negative_cycle[t] == negative);
                assert(breadth.reachable(t) == reachable && depth.reachable(t) == reachable);
                if (reachable && !negative) {
                    assert(bellman.dist[t] == expected[s][t]); assert(all[s].dist[t] == expected[s][t]);
                }
                check_path(g, bellman, t); check_path(g, all[s], t);
                check_path(g, breadth, t, true); check_path(g, depth, t, true);
            }
            if (mode != 0) {
                auto dijkstra = cp::dijkstra(g, s);
                for (int t = 0; t < n; ++t) {
                    assert(dijkstra.reachable(t) == (expected[s][t] != oracle_inf));
                    if (dijkstra.reachable(t)) assert(dijkstra.dist[t] == expected[s][t]);
                    check_path(g, dijkstra, t);
                }
                if (mode == 1) assert(cp::zero_one_bfs(g, s).dist == dijkstra.dist);
            }
        }
    }
    cp::Graph<ll> overflow(3, cp::GraphKind::Directed);
    overflow.add_edge(0, 1, std::numeric_limits<ll>::max()-2); overflow.add_edge(1, 2, 4);
    bool threw = false;
    try { (void)cp::dijkstra(overflow); } catch (const std::overflow_error&) { threw = true; }
    assert(threw);
    cp::Graph<double> real(3, cp::GraphKind::Directed);
    real.add_edge(0, 1, 1.25); real.add_edge(1, 2, 2.5);
    assert(cp::dijkstra(real).dist[2] == 3.75);
    cp::Graph<unsigned> positive(2); positive.add_edge(0, 1, 4u);
    assert(cp::dijkstra(positive).dist[1] == 4u);
}

static std::set<std::vector<int>> canonical(std::vector<std::vector<int>> groups) {
    std::set<std::vector<int>> result;
    for (auto& group : groups) { std::sort(group.begin(), group.end()); result.insert(group); }
    return result;
}

// 「どの第3頂点を除いてもつながる」ペアから極大なブロックを全列挙する。
static std::set<std::vector<int>> brute_blocks(const cp::Graph<ll>& g) {
    int n = g.size(); auto original = labels(g);
    std::vector<std::vector<bool>> pair(n, std::vector<bool>(n));
    for (int u = 0; u < n; ++u) for (int v = u+1; v < n; ++v) {
        bool connected = original[u] == original[v];
        for (int removed = 0; removed < n; ++removed) {
            if (removed == u || removed == v) continue;
            auto after = labels(g, removed); connected &= after[u] == after[v];
        }
        pair[u][v] = pair[v][u] = connected;
    }
    std::set<std::vector<int>> result;
    for (int mask = 1; mask < (1 << n); ++mask) {
        bool valid = true;
        for (int u = 0; u < n; ++u) if (mask >> u & 1)
            for (int v = u+1; v < n; ++v) if ((mask >> v & 1) && !pair[u][v]) valid = false;
        if (!valid) continue;
        bool maximal = true;
        for (int v = 0; v < n; ++v) if (!(mask >> v & 1)) {
            bool addable = true;
            for (int u = 0; u < n; ++u) if ((mask >> u & 1) && !pair[u][v]) addable = false;
            if (addable) maximal = false;
        }
        if (!maximal) continue;
        std::vector<int> group;
        for (int v = 0; v < n; ++v) if (mask >> v & 1) group.push_back(v);
        result.insert(group);
    }
    return result;
}

static void graph_decompositions() {
    for (int iteration = 0; iteration < 550; ++iteration) {
        int n = random_int(8); cp::Graph<ll> g(n);
        if (n) for (int i = random_int(15); i > 0; --i) g.add_edge(random_int(n), random_int(n));
        auto original = labels(g); int components = group_count(original);
        auto low = cp::low_link(g);
        std::vector<int> expected_bridges, expected_articulation;
        for (int idx = 0; idx < g.edge_count(); ++idx)
            if (group_count(labels(g, -1, idx)) > components) expected_bridges.push_back(idx);
        for (int v = 0; v < n; ++v)
            if (group_count(labels(g, v)) > components) expected_articulation.push_back(v);
        assert(low.bridge_edges == expected_bridges && low.articulation == expected_articulation);
        auto connected = cp::connected_components(g);
        for (int u = 0; u < n; ++u) for (int v = 0; v < n; ++v)
            assert((connected.component[u] == connected.component[v]) == (original[u] == original[v]));
        auto two = cp::two_edge_connected_components(g);
        for (int u = 0; u < n; ++u) for (int v = 0; v < n; ++v) {
            bool same = original[u] == original[v];
            for (int idx : expected_bridges) { auto after = labels(g, -1, idx); same &= after[u] == after[v]; }
            assert((two.component[u] == two.component[v]) == same);
        }
        cp::RootedTree<ll> forest(two.forest);
        assert(static_cast<int>(forest.roots.size()) == components);
        assert(two.original_edge == expected_bridges);
        auto blocks = cp::biconnected_components(g);
        assert(canonical(blocks) == brute_blocks(g));
        auto cut = cp::block_cut_forest(g);
        cp::RootedTree<int> cut_tree(cut.forest);
        assert(static_cast<int>(cut_tree.roots.size()) == components);
        bool bipartite = false;
        for (int mask = 0; mask < (1 << n); ++mask) {
            bool valid = true;
            for (const auto& e : g.edges()) valid &= ((mask >> e.from) & 1) != ((mask >> e.to) & 1);
            bipartite |= valid;
        }
        assert(cp::bipartite_coloring(g).has_value() == bipartite);
    }
}

static void cycles_and_order() {
    for (int iteration = 0; iteration < 350; ++iteration) {
        int n = random_int(9); bool directed = iteration % 2;
        cp::Graph<ll> g(n, directed ? cp::GraphKind::Directed : cp::GraphKind::Undirected);
        if (n) for (int i = random_int(18); i > 0; --i) g.add_edge(random_int(n), random_int(n));
        bool expected_cycle = false;
        if (directed) {
            auto distance = oracle_distances(g);
            for (const auto& e : g.edges()) expected_cycle |= distance[e.to][e.from] != oracle_inf;
            auto order = cp::topological_sort(g);
            assert(order.has_value() == !expected_cycle);
            if (order) {
                std::vector<int> pos(n);
                for (int i = 0; i < n; ++i) pos[(*order)[i]] = i;
                for (const auto& e : g.edges()) assert(pos[e.from] < pos[e.to]);
            }
        } else {
            expected_cycle = g.edge_count() > n - group_count(labels(g));
        }
        auto cycle = cp::find_cycle(g); assert(cycle.has_value() == expected_cycle);
        if (cycle) {
            assert(cycle->vertices.size() == cycle->edges.size());
            assert(std::set<int>(cycle->edges.begin(), cycle->edges.end()).size() == cycle->edges.size());
            for (int i = 0; i < cycle->size(); ++i) {
                auto e = g.edges()[cycle->edges[i]];
                int u = cycle->vertices[i], v = cycle->vertices[(i+1)%cycle->size()];
                assert((e.from == u && e.to == v) || (!directed && e.to == u && e.from == v));
            }
        }
    }
}

static void spanning_forests() {
    for (int iteration = 0; iteration < 250; ++iteration) {
        int n = random_int(7); cp::Graph<ll> g(n);
        if (n) for (int i = random_int(11); i > 0; --i) g.add_edge(random_int(n), random_int(n), random_int(16)-5);
        auto original = labels(g); int components = group_count(original);
        ll best = oracle_inf;
        for (int mask = 0; mask < (1 << g.edge_count()); ++mask) {
            if (__builtin_popcount(static_cast<unsigned>(mask)) != n-components) continue;
            std::vector<int> group(n); std::iota(group.begin(), group.end(), 0);
            ll cost = 0; bool valid = true;
            for (const auto& e : g.edges()) if (mask >> e.idx & 1) {
                int a = group[e.from], b = group[e.to];
                if (a == b) { valid = false; break; }
                for (int& id : group) if (id == b) id = a;
                cost += e.cost;
            }
            if (valid) best = std::min(best, cost);
        }
        auto a = cp::kruskal(g), b = cp::prim(g);
        assert(a.cost == best && b.cost == best);
        assert(a.component_count == components && b.component_count == components);
        for (const auto* result : {&a, &b}) {
            assert(static_cast<int>(result->edges.size()) == n-components);
            cp::Graph<ll> chosen(n); ll cost = 0;
            for (const auto& e : result->edges) { chosen.add_edge(e.from, e.to, e.cost); cost += e.cost; }
            assert(cost == best && group_count(labels(chosen)) == components);
        }
        for (int i = 0; i < g.edge_count(); ++i) assert(g.edges()[i].idx == i);
    }
}

static std::vector<int> oracle_tree_path(const cp::Graph<ll>& g, int from, int to) {
    std::vector<int> parent(g.size(), -1), todo{from}; parent[from] = from;
    for (std::size_t i = 0; i < todo.size(); ++i)
        for (const auto& e : g[todo[i]]) if (parent[e.to] == -1) {
            parent[e.to] = todo[i]; todo.push_back(e.to);
        }
    if (parent[to] == -1) return {};
    std::vector<int> path;
    for (int v = to; v != from; v = parent[v]) path.push_back(v);
    path.push_back(from); std::reverse(path.begin(), path.end()); return path;
}
static ll path_cost(const cp::Graph<ll>& g, const std::vector<int>& path) {
    ll total = 0;
    for (std::size_t i = 1; i < path.size(); ++i)
        for (const auto& e : g[path[i-1]]) if (e.to == path[i]) total += e.cost;
    return total;
}

static void centroid_structure(const cp::Graph<ll>& g, const cp::CentroidDecomposition<ll>& cd) {
    for (int center = 0; center < g.size(); ++center) {
        std::vector<int> included(g.size()); int total = 0;
        for (int v = 0; v < g.size(); ++v) {
            int ancestor = v;
            while (ancestor != -1 && ancestor != center) ancestor = cd.parent[ancestor];
            if (ancestor == center) { included[v] = 1; ++total; }
        }
        assert(cd.component_size[center] == total);
        assert(cd.level[center] == (cd.parent[center] == -1 ? 0 : cd.level[cd.parent[center]]+1));
        std::vector<bool> visited(g.size()); visited[center] = true;
        for (int root = 0; root < g.size(); ++root) if (included[root] && !visited[root]) {
            std::vector<int> todo{root}; visited[root] = true;
            for (std::size_t i = 0; i < todo.size(); ++i)
                for (const auto& e : g[todo[i]]) if (included[e.to] && !visited[e.to]) {
                    visited[e.to] = true; todo.push_back(e.to);
                }
            assert(static_cast<int>(todo.size()) <= total/2);
        }
    }
}

static void tree_queries() {
    for (int iteration = 0; iteration < 220; ++iteration) {
        int n = random_int(25), root = n ? random_int(n) : 0;
        cp::Graph<ll> g(n);
        for (int v = 1; v < n; ++v) if (random_int(5)) g.add_edge(random_int(v), v, random_int(15)-5);
        cp::DoublingLCA<ll> lca(g, root);
        cp::HeavyLightDecomposition<ll> hld(g, root);
        cp::EulerTour<ll> tour(g, root);
        cp::CentroidDecomposition<ll> cd(g);
        assert(static_cast<int>(cd.roots.size()) == group_count(labels(g)));
        centroid_structure(g, cd);
        for (int v = 0; v < n; ++v) {
            auto [l, r] = hld.subtree_interval(v); int count = 0;
            for (int u = 0; u < n; ++u) {
                auto path = oracle_tree_path(g, root, u);
                // 各成分の根で取り直す。
                if (path.empty()) path = oracle_tree_path(g, lca.tree.roots[lca.tree.component[u]], u);
                bool descendant = std::find(path.begin(), path.end(), v) != path.end();
                assert(lca.tree.is_ancestor(v, u) == descendant);
                assert((l <= hld.pos[u] && hld.pos[u] < r) == descendant);
                count += descendant;
            }
            assert(lca.tree.subtree_size[v] == count);
            assert(tour.walk[tour.first[v]] == v && tour.walk[tour.last[v]] == v);
            for (int k = 0; k <= lca.tree.depth[v] + 1; ++k) {
                int expected = v;
                for (int j = 0; j < k && expected != -1; ++j) expected = lca.tree.parent[expected];
                assert(lca.kth_ancestor(v, k) == expected);
            }
        }
        for (int e = 0; e < g.edge_count(); ++e) {
            assert(tour.edge_walk[tour.edge_in[e]] == e && tour.edge_walk[tour.edge_out[e]] == e);
            assert(tour.edge_in[e] < tour.edge_out[e]);
            assert(std::count(tour.edge_walk.begin(), tour.edge_walk.end(), e) == 2);
        }
        for (int u = 0; u < n; ++u) for (int v = 0; v < n; ++v) {
            auto expected = oracle_tree_path(g, u, v);
            assert(lca.vertex_path(u, v) == expected);
            if (expected.empty()) { assert(lca.lca(u,v) == -1 && hld.lca(u,v) == -1); continue; }
            int ancestor = u;
            std::set<int> ancestors;
            for (int x = v; x != -1; x = lca.tree.parent[x]) ancestors.insert(x);
            while (!ancestors.count(ancestor)) ancestor = lca.tree.parent[ancestor];
            assert(lca.lca(u,v) == ancestor && hld.lca(u,v) == ancestor);
            assert(lca.distance(u,v) == path_cost(g, expected));
            std::vector<int> flattened;
            for (const auto& segment : hld.path_segments(u, v)) {
                if (segment.reversed) for (int i = segment.r-1; i >= segment.l; --i) flattened.push_back(hld.vertex_at[i]);
                else for (int i = segment.l; i < segment.r; ++i) flattened.push_back(hld.vertex_at[i]);
            }
            assert(flattened == expected);
            std::vector<int> expected_edges;
            for (std::size_t i = 1; i < expected.size(); ++i)
                expected_edges.push_back(lca.tree.depth[expected[i-1]] > lca.tree.depth[expected[i]] ? expected[i-1] : expected[i]);
            flattened.clear();
            for (const auto& segment : hld.path_segments(u, v, true)) {
                if (segment.reversed) for (int i = segment.r-1; i >= segment.l; --i) flattened.push_back(hld.vertex_at[i]);
                else for (int i = segment.l; i < segment.r; ++i) flattened.push_back(hld.vertex_at[i]);
            }
            assert(flattened == expected_edges);
            for (std::size_t k = 0; k <= expected.size(); ++k)
                assert(lca.jump(u,v,static_cast<ll>(k)) == (k < expected.size() ? expected[k] : -1));
            for (int x = 0; x < n; ++x)
                assert(lca.is_on_path(u,v,x) == (std::find(expected.begin(), expected.end(), x) != expected.end()));
        }
        auto centers = cp::tree_centroids(g);
        auto original = labels(g);
        for (int v = 0; v < n; ++v) {
            auto after = labels(g, v); std::vector<int> sizes(n);
            int total = 0, largest = 0;
            for (int x = 0; x < n; ++x) if (original[x] == original[v]) {
                ++total; if (x != v) largest = std::max(largest, ++sizes[after[x]]);
            }
            bool expected = largest <= total/2;
            assert((std::find(centers.begin(), centers.end(), v) != centers.end()) == expected);
        }
    }
    for (int iteration = 0; iteration < 100; ++iteration) {
        int n = random_int(25); cp::Graph<ll> g(n);
        for (int v = 1; v < n; ++v) g.add_edge(random_int(v), v, random_int(10));
        auto result = cp::tree_diameter(g); ll best = 0;
        for (int u = 0; u < n; ++u) for (int v = 0; v < n; ++v)
            best = std::max(best, path_cost(g, oracle_tree_path(g,u,v)));
        assert(result.length == best);
        if (n) { assert(result.vertices == oracle_tree_path(g,result.endpoints.first,result.endpoints.second)); }
        else assert(result.vertices.empty() && result.endpoints.first == -1);
    }
    cp::Graph<ll> invalid(2); invalid.add_edge(0,1); invalid.add_edge(0,1);
    bool threw = false;
    try { cp::RootedTree<ll> tree(invalid); } catch (const std::invalid_argument&) { threw = true; }
    assert(threw);
}

static void compatibility() {
    Graph<ll> g(5); g.add_edge(0,1,4,10); g.add_edge(1,2,3,20); g.add_edge(0,2,9,30); g.add_edge(3,4,2,40);
    assert(g.E.size() == 4 && g.e_size() == 4);
    assert(g.dijkstra().dist[2] == 7 && g.dijkstra().Eid[2] == 20);
    assert(g.bfs().dist[2] == 1); g.dfs(); g.bellman_ford(); g.SingleSource_ShortestPath();
    assert(g.floyd_warshall()[0].dist[2] == 7); assert(g.Allpair_ShortestPath()[0].dist[2] == 7);
    assert(g.prim().first == 9 && g.prim().second.size() == 3);
    assert(g.kruskal().first == 9 && g.kruskal().second.size() == 3);
    assert(!g.is_bipartite() && !g.is_DAG()); g.bipartite_grouping(); g.topological_sort();
    g.Calc_Deg(); assert(g.Out_Deg[0] == 2); g.components();
    assert(g.find_cycle().len == 3); assert(g.Bridge().size() == 1 && g.Bridge()[0].first.idx == 40);
    assert(g.Articulation().empty());
    auto [component, forest] = g.TwoEdgeConnectedComponents();
    assert(component[0] == component[2] && forest.size() == 3); forest.build();
    g.BiConnectedComponents(); g.BlockCutTree().build();
    g.add_edge(2,3,1,50); assert(g.Articulation().size() == 2); assert(g.floyd_warshall()[0].dist[4] == 10);
    g.resize(3); assert(g.E.size() == 3 && g.dijkstra().dist[2] == 7);
    Graph<ll> zero(3); zero.add_edge(0,1,0); zero.add_edge(1,2,1); assert(zero.bfs_01().dist[2] == 1);
    Graph<double> simple(3); simple[0].emplace_back(1,2.5);
    auto distance = dijkstra(simple); assert(distance[1] == 2.5 && distance[2] == -1);
    Graph<ll> dag(3); dag.add_directed_edge(0,1); dag.add_directed_edge(1,2);
    assert(dag.is_DAG() && dag.topological_sort() == std::vector<int>({0,1,2}));
    Graph<ll> empty; assert(empty.is_bipartite() && empty.is_DAG());
    Graph<ll> negative(3); negative.add_directed_edge(0,1,1); negative.add_directed_edge(1,1,-1);
    auto result = negative.bellman_ford(); assert(result.dist[1] == -negative.inf && result.dist[2] == negative.inf);

    Tree<ll> tree(5); tree.add_edge(0,1,5); tree.add_edge(1,2,0); tree.add_edge(1,3,2); tree.add_edge(3,4,3);
    assert(tree.dist(0,4) == 10 && tree.LCA(2,4) == 1 && tree.size(1) == 4 && tree.size() == 5);
    assert(!tree.is_on_path(0,1,2)); assert(tree.nxt(2,2) == -1 && tree.nxt(2,4) == 1);
    assert(tree.kth_ancestor(4,4) == -1); tree.dep(4); tree.par(4); tree.path(2,4);
    assert(tree.diameter().first == 10); tree.EulerTour_build(); auto walk = tree.ET_V;
    tree.EulerTour_build(); assert(walk == tree.ET_V); tree.HLD_build(); auto positions = tree.HLD;
    tree.HLD_build(); assert(positions == tree.HLD);
    std::set<int> vertices;
    for (auto [l,r] : tree.HLD_query(2,4)) for (int i = l; i <= r; ++i) vertices.insert(tree.HLD[i]);
    assert(vertices == std::set<int>({1,2,3,4})); tree.centroid(); tree.centroid_decomposition();
    tree.rebuild(4); assert(tree.LCA(0,2) == 1 && tree.dist(0,4) == 10 && tree.ET_V.empty());
    tree.resize(4); tree.rebuild(0); assert(tree.dist(0,3) == 7);
}

static void io_and_grid() {
    std::istringstream input("1 2 4\n2 3 7\n");
    cp::GraphInputOptions options; options.weighted = true;
    auto g = cp::read_graph<ll>(input,3,2,options); assert(cp::dijkstra(g).dist[2] == 11);
    assert(BFS({{1},{2},{}},0) == std::vector<int>({0,1,2}));
    auto grid = Grid_BFS({"...",".#.","..."},0,0);
    assert(grid[2][2] == 4 && grid[1][1] == -1);
    assert(cp::grid_bfs({},0,0).empty()); assert(cp::grid_bfs({"#"},0,0)[0][0] == -1);
    unionfind dsu(3); assert(dsu.unite(0,1) && !dsu.unite(0,1) && dsu.size(1) == 2);
}

static void deep_chain(int n) {
    cp::Graph<ll> g(n);
    for (int v = 1; v < n; ++v) g.add_edge(v-1,v);
    { cp::DoublingLCA<ll> lca(g); assert(lca.lca(n-1,n/2) == n/2 && lca.distance(0,n-1) == n-1); }
    { cp::HeavyLightDecomposition<ll> hld(g); assert(hld.path_segments(n-1,0).size() == 1); }
    { cp::EulerTour<ll> tour(g); assert(static_cast<int>(tour.walk.size()) == 2*n-1); }
    { auto result = cp::low_link(g); assert(static_cast<int>(result.bridge_edges.size()) == n-1); }
    { auto result = cp::biconnected_components(g); assert(static_cast<int>(result.size()) == n-1); }
    { cp::CentroidDecomposition<ll> cd(g); assert(*std::max_element(cd.level.begin(), cd.level.end()) < 20); }
    assert(cp::depth_first_search(g).dist.back() == n-1);
    assert(cp::tree_diameter(g).length == n-1);
}

int main(int argc, char** argv) {
    (void)argv;
    shortest_paths(); graph_decompositions(); cycles_and_order(); spanning_forests(); tree_queries();
    compatibility(); io_and_grid(); deep_chain(argc > 1 ? 200000 : 20000);
    std::cout << "graph/tree: randomized differential tests, compatibility, and deep chain passed\n";
}
