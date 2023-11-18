/*
★ Name        ： Graph/Edge template
 ■ Snipet      : No

 ■ Arguments   : 
 ■ Return      : 
 ■ Complexity  : 

 ■ Description : 
 

 ■ Usage       : 


 ■ Verify      :・次数確認  : https://atcoder.jp/contests/abc061/submissions/44696570 （出次数） 
                             https://atcoder.jp/contests/arc037/submissions/44702792 （入次数）
                             https://atcoder.jp/contests/abc292/submissions/44706628
                             https://atcoder.jp/contests/abc288/submissions/44706798
                ・dijkstra : https://atcoder.jp/contests/abc035/submissions/44674674
                              https://atcoder.jp/contests/arc064/submissions/44674896
                              https://judge.yosupo.jp/submission/156318 (パス中の頂点の復元)
                              https://atcoder.jp/contests/abc252/submissions/44674993 (最短路木の辺集合)
                              https://atcoder.jp/contests/abc305/submissions/44706388
                 ・Bellman-Ford : https://atcoder.jp/contests/abc137/submissions/44680861 (負閉路検出含む)
                                  https://atcoder.jp/contests/abc061/submissions/44680972
                 ・Floyd-Warshall : https://atcoder.jp/contests/abc051/submissions/44685373 （最短路の辺集合の復元）
                                    https://atcoder.jp/contests/abc143/submissions/44694953
                                    https://atcoder.jp/contests/abc079/submissions/44695727 (隣接行列の入力)
                            (TODO :)    https://atcoder.jp/contests/abc191/tasks/abc191_e
                                        https://atcoder.jp/contests/abc160/tasks/abc160_d
                                        https://atcoder.jp/contests/abc073/tasks/abc073_d
                                        https://atcoder.jp/contests/arc159/tasks/arc159_a
                                        https://atcoder.jp/contests/abc243/tasks/abc243_e
                                        https://atcoder.jp/contests/arc035/tasks/arc035_c
                                        https://atcoder.jp/contests/abc012/tasks/abc012_4
                 ・全頂点対最短路    : https://atcoder.jp/contests/abc016/submissions/44696629
                                      https://atcoder.jp/contests/arc025/submissions/46654773
                 ・BFS              : https://atcoder.jp/contests/abc309/submissions/44695442
                                      https://atcoder.jp/contests/abc251/submissions/44695547 (BFS木)
                                      https://atcoder.jp/contests/abc132/submissions/44702688
                                      https://atcoder.jp/contests/abc292/submissions/44706495
                                      https://atcoder.jp/contests/abc218/submissions/45759654
                                      https://atcoder.jp/contests/arc011/submissions/46668040
                 ・DFS              : https://atcoder.jp/contests/abc251/submissions/44695547 (DFS木)
                 ・0-1BFS           : https://atcoder.jp/contests/abc184/submissions/44695926
                 ・二部グラフ        : https://atcoder.jp/contests/abc282/submissions/44696396 
                 ・連結成分に分解    : https://atcoder.jp/contests/abc282/submissions/44696396
                                    https://atcoder.jp/contests/arc037/submissions/44702792
                                    https://atcoder.jp/contests/abc292/submissions/44706628
                                    https://atcoder.jp/contests/abc288/submissions/44706798
                 ・Kruskal          : https://atcoder.jp/contests/abc282/submissions/44700772
                 ・Prim             : https://atcoder.jp/contests/abc282/submissions/44700838
                 ・トポロジカルソート : https://atcoder.jp/contests/dp/submissions/44703075
                                      https://codeforces.com/contest/1851/submission/220685825
                 ・サイクル検出      : https://atcoder.jp/contests/past202107-open/submissions/44703572 (有向)
                                      https://judge.yosupo.jp/submission/156511 (有向、長さ、辺集合、順序関係)
                                      https://judge.yosupo.jp/submission/156512 (無向、長さ、頂点集合、辺集合、順序関係)
                                      https://atcoder.jp/contests/abc311/submissions/44707019 (functional graph)
                    TODO : https://atcoder.jp/contests/abc296/tasks/abc296_e
                           https://atcoder.jp/contests/abc065/tasks/arc076_b
                           https://atcoder.jp/contests/abc277/tasks/abc277_e
                           https://atcoder.jp/contests/abc291/tasks/abc291_f
                           https://atcoder.jp/contests/iroha2019-day2/tasks/iroha2019_day2_d
                           https://atcoder.jp/contests/arc029/tasks/arc029_3
                           https://atcoder.jp/contests/abc237/tasks/abc237_e
                           鉄則本
                           yosupojudge
                           AOJ


 ■ References  : https://ei1333.github.io/library/graph/graph-template.hpp
                 https://github.com/tsuchinoko29/nok0/blob/main/graph/graph.hpp
                 https://github.com/magurofly/cp-library-rs
                 https://nyaannyaan.github.io/library/graph/graph-template.hpp
                 https://kopricky.github.io/code.html
                 https://github.com/kth-competitive-programming/kactl

 ■ TODO        : bfs等も組み込む、グリッドグラフ用、木用の構造体Treeも別でつくる、メモリが大きすぎないかの確認
*/

#pragma once
#include <vector>
#include <queue>
#include <deque>
#include <stack>
#include <algorithm>
#include <cmath>
#include <cassert>
#include <limits>
//#include "../data_structure/UnionFind.hpp"


using namespace std; // TODO : stdをちゃんと書く、英訳する


// Union-Find
struct unionfind{
    vector<int> par, siz;
 
    // 初期化
    unionfind(int n) : par(n, -1), siz(n, 1) {}
 
    // 根を求める
    int root(int x) {
        if (par[x] == -1) return x;
        else return par[x] = root(par[x]);
    }
 
    // xとyの根（グループ）が一致するかどうか
    bool issame(int x, int y){
        return root(x) == root(y);
    }
 
    // xとyのグループの併合
    bool unite(int x, int y){
        x = root(x); y = root(y);
 
        if (x == y) return false;
 
        if (siz[x] < siz[y]) swap(x,y);
 
        par[y] = x;
        siz[x] += siz[y];
        return true;
    }
 
    // xを含むグループのサイズ
    int size(int x){
        return siz[root(x)];
    }
};


/// TODO : グラフアルゴリズム全部載せGraph構造体周りを完成させる

/// 辺 (始点 from、終点 to、コスト cost、インデックス idx) を管理する構造体 Edge
// cost の型Tを渡す (初期設定は long long)
template< typename T = long long >
struct Edge {
    int from, to;
    T cost;
    int idx;

    Edge() = default;

    Edge(int from_, int to_, T cost_ = 1, int idx_ = -1) : from(from_), to(to_), cost(cost_), idx(idx_) {}
};


/// 辺集合 Edges
template< typename T = long long >
using Edges = vector< Edge< T > >;


/// 最短経路に関する情報 (最短距離 dist, 経路復元用 prev (直前の頂点), Eid (直前の辺) ) を保持する構造体 ShortestPath
template< typename T = long long >
struct ShortestPath {
    vector< T > dist;
    vector< int > prev, Eid;

    ShortestPath() = default;

    ShortestPath(int N) : dist(N), prev(N), Eid(N) {}

    ShortestPath(vector< T > &dist_, vector< int > &prev_, vector< int > &Eid_) : dist(dist_), prev(prev_), Eid(Eid_) {}
};


/// サイクルに関する情報 (大きさ len, 頂点番号 Vid , 辺番号 Eid ) を保持する構造体 Cycle
struct Cycle {
    int len = 0;
    vector< int > Vid, Eid;

    Cycle() = default;

    size_t size() const { 
        return len;
    }
};


/// (重み有り有向)グラフを管理する構造体 Graph
//  重み無し -> cost all 1, 無向 -> 両方向に有向辺 と帰着させるような実装
template< typename T = long long >
using graph = vector< vector< Edge< T > > >;
template< typename T = long long >
class Graph {
    /* 基本構造 */
    public :
        graph< T > G;
        Edges< T > E;
        vector< int > In_Deg; // 入次数
        vector< int > Out_Deg; // 出次数
        vector< vector< T > > AdjMat; 
        vector< vector< int > > idxMat; // 隣接行列の辺番号（あれば） 
        const T inf = numeric_limits< T >::max();

        Graph() = default;

        explicit Graph(int n) : G(n), edge_num(0), negative_edge(false), is_weighted(false), is_directed(false) {}

        /// グラフの頂点数を返す
        size_t size() const { 
            return G.size();
        }

        /// グラフの辺の数を返す
        // 無向グラフの場合は E.size() != edge_num となることに注意
        size_t e_size() const {
            return (size_t)edge_num;
        }

        /// グラフの頂点数の変更 (減らす場合は破壊的)
        void resize(int N) {
            if(N < (int)G.size()) {
                Edges< T > newE;
                for(auto &e:E) if(e.from >= N || e.to >= N) newE.emplace_back(e);
                swap(E,newE);
            }
            G.resize(N);
            Calc_Deg();
        }

        /// 有向辺を追加する
        void add_directed_edge(int from, int to, T cost = 1) { // Verified
            assert(0 <= from && from < (int)G.size() && "ERROR : out of bound graph access");
            assert(0 <= to && to < (int)G.size() && "ERROR : out of bound graph access");

            Edge< T > e = {from, to, cost, edge_num};
            G[from].emplace_back(e);
            E.emplace_back(e);

            if(cost < T(0)) negative_edge = true;
            edge_num++;
        }

        /// 無向辺を追加する 
        // 一回で有向辺を両方向に追加していることに注意（Eには便宜上両方追加しているが、どうするか未定）
        void add_edge(int from, int to, T cost = 1) { // Verified
            assert(0 <= from && from < (int)G.size() && "ERROR : out of bound graph access");
            assert(0 <= to && to < (int)G.size() && "ERROR : out of bound graph access");

            Edge< T > e = {from, to, cost, edge_num};
            G[from].emplace_back(e);
            E.emplace_back(e);

            Edge< T > inve = {to, from, cost, edge_num};
            G[to].emplace_back(inve);
            E.emplace_back(inve);

            if(cost < T(0)) negative_edge = true;
            edge_num++;
        }

        /// グラフ（隣接リスト）の一括入力
        // offset (0-indexedや複数グラフ用)、weighted (重み有り/無し)、directed (有向/無向) を指定する
        void input(int M, int offset = -1, bool weighted = false, bool directed = false) { // Verified
            for(int i = 0; i < M; i++) {
                int a, b;
                cin >> a >> b;
                a += offset;
                b += offset;
                T c = T(1);
                if(weighted) {
                    cin >> c;
                    is_weighted = true;
                }
                if(directed) {
                    add_directed_edge(a, b, c);
                    is_directed = true;
                }
                else add_edge(a, b, c);
            }
        }

        /// グラフ（隣接行列）の一括入力
        void input_adjmat() { // Verified
            int N = G.size();
            build_adjmat();
            for(int i = 0; i < N; i++) {
                for(int j = 0; j < N; j++) {
                    T cost; cin >> cost;
                    AdjMat[i][j] = cost;
                }
            }
        }

        /// 入次数・出次数を計算
        void Calc_Deg() {
            In_Deg.assign(G.size(), 0);
            Out_Deg.assign(G.size(), 0);
            for(auto &e : E) {
                Out_Deg[e.from]++;
                In_Deg[e.to]++;
            }
        }

        /// グラフの要素へのアクセス（変更可能・不能を両方定義）
        inline vector< Edge< T > > &operator[](const int &k) {
            return G[k];
        }

        inline const vector< Edge< T > > &operator[](const int &k) const {
            return G[k];
        }



        /* 便利関数 */

        /// Dijkstra 法 (単一始点最短路)
        // O(E log(V))、負辺 NG
        // 始点 start を指定、cannnot reach : numeric_limits< T >::max()
        // priority_queue を用いて実装した基本形
        ShortestPath< T > dijkstra(int start = 0) { // Verified
            assert(!negative_edge && "ERROR : cannot use dijkstra with negative edge");
            assert(0 <= start && start < (int)G.size() && "ERROR : out of bound graph access");

            using P = pair<T, int>;
            int N = (int)G.size();
            vector< T > dist(N, inf);
            vector< int > prev(N, -1);
            vector< int > Eid(N, -1);
            priority_queue<P, vector<P>, greater<P> > pq;
            dist[start] = 0;
            pq.push({0, start});

            while (!pq.empty()) {
                auto [nowcost, from] = pq.top();
                pq.pop();

                if (dist[from] < nowcost) continue;

                for (auto &e : G[from]) {
                    T nxtcost = nowcost + e.cost;
                    if (nxtcost < dist[e.to]) {
                        dist[e.to] = nxtcost;
                        prev[e.to] = from;
                        Eid[e.to] = e.idx;
                        pq.push({dist[e.to], e.to});
                    }
                }
            }

            return {dist, prev, Eid};
        }


        /// Bellman-Ford 法 (単一始点最短路)
        // O(EV)、負辺 OK
        // 始点 start を指定、負閉路があった場合は負閉路によっていくらでも小さくなる頂点のみ-inf、cannnot reach : numeric_limits< T >::max()
        ShortestPath< T > bellman_ford(int start = 0) { // Verified
            assert(0 <= start && start < (int)G.size() && "ERROR : out of bound graph access");

            int N = (int)G.size();
            vector< T > dist(N, inf);
            vector< int > prev(N, -1);
            vector< int > Eid(N, -1);
            dist[start] = 0;

            for(int iter = 0; iter < N; iter++) {
                vi update;
                for(int v = 0; v < N; v++) {
                    if(dist[v] == inf) continue;

                    for(auto &e : G[v]) {
                        T nxtcost = dist[v] + e.cost;
                        if(nxtcost < dist[e.to]) {
                            dist[e.to] = nxtcost;
                            prev[e.to] = v;
                            Eid[e.to] = e.idx;
                            update.pb(e.to);
                        }
                    }
                }
                if(update.empty()) break;

                if(iter == N-1 && !update.empty()) { // 負閉路検出
                    auto dfs_ =[&](auto dfs_, int now) -> void{
                        dist[now] = -inf;
                        for(auto &e : G[now]) {
                            if(dist[e.to] == -inf) continue;
                            dfs_(dfs_, e.to);
                        }
                    };
                    for(auto v:update) {
                        if(dist[v] != -inf) dfs_(dfs_, v);
                    }
                }
            }

            return {dist, prev, Eid};      
        }


        /// 単一始点最短路問題ソルバー
        // 始点 start を指定
        // 負辺が無ければ Dijkstra、有れば Bellman-Ford
        ShortestPath< T > SingleSource_ShortestPath(int start = 0) { // Verified
            if(!negative_edge) return dijkstra(start);
            else return bellman_ford(start);
        }


        /// Floyd–Warshall 法 (全点対間最短路)
        // O(V^3)、負辺OK
        vector< ShortestPath< T > > floyd_warshall() { // Verified
            if(AdjMat.size() != G.size()) build_adjmat();
            int N = G.size();
            vector< ShortestPath< T > > ret(N, ShortestPath< T >(N));
            for(int i = 0; i < N; i++) ret[i].dist = AdjMat[i]; 
            for(int i = 0; i < N; i++) {
                for(int j = 0; j < N; j++) {
                    ret[i].prev[j] = i;
                    ret[i].Eid[j] = idxMat[i][j];
                }
            }

            for(int k = 0; k < N; k++) {
                for(int i = 0; i < N; i++) {
                    for(int j = 0; j < N; j++) {
                        if(ret[i].dist[k] == inf || ret[k].dist[j] == inf) continue;
                        T nxtcost = ret[i].dist[k] + ret[k].dist[j];
                        if(nxtcost < ret[i].dist[j]) {
                            ret[i].dist[j] = nxtcost;
                            ret[i].prev[j] = ret[k].prev[j];
                            ret[i].Eid[j] = ret[k].Eid[j];
                        }
                    }
                }
            }

            return ret;
        }

        /// 全点対間最短路問題ソルバー
        // Floyd-Warshall (O(V^3)), Dijkstra V times (O(VE log(V))) のうち早い方を使う（負辺の有無にも注意）
        vector< ShortestPath< T > > Allpair_ShortestPath() { // Verified
            if(negative_edge) return floyd_warshall();
            else {
                long long V = G.size();
                if(V * V * V <= V * log2(V) * e_size()) return floyd_warshall();
                else {
                    int N = G.size();
                    vector< ShortestPath< T > > ret(N);
                    for(int i = 0; i < N; i++) ret[i] = dijkstra(i);
                    return ret;
                }
            }
        }


        /// BFS
        // O(V+E), arg : start point
        // 重み付きグラフに対しては未定義動作（連結判定には使える）
        ShortestPath< T > bfs(int start = 0) { // Verified
            assert(0 <= start && start < (int)G.size() && "ERROR : out of bound graph access");

            int N = (int)G.size();
            vector< T > dist(N, inf);
            vector< int > prev(N, -1);
            vector< int > Eid(N, -1);
            queue< int > q;
            dist[start] = 0;
            q.push(start);

            while(!q.empty()) {
                int from = q.front();
                q.pop();

                for(auto &e : G[from]) {
                    if(dist[e.to] != inf) continue;
                    dist[e.to] = dist[from] + 1;
                    prev[e.to] = from;
                    Eid[e.to] = e.idx;
                    q.push(e.to);
                }
            }

            return {dist, prev, Eid};
        }


        /// 0-1 BFS
        // O(V+E), arg : start point
        // 辺の重みが0/1のみのグラフ限定 
        ShortestPath< T > bfs_01(int start = 0) { // Verified
            assert(0 <= start && start < (int)G.size() && "ERROR : out of bound graph access");

            int N = (int)G.size();
            vector< T > dist(N, inf);
            vector< int > prev(N, -1);
            vector< int > Eid(N, -1);
            deque< int > dq;
            dist[start] = 0;
            dq.push_back(start);

            while(!dq.empty()) {
                int from = dq.front();
                dq.pop_front();

                for(auto &e : G[from]) {
                    assert(T(0) <= e.cost && e.cost <= T(1) && "ERROR : cost of each edge needs to be 0 or 1");
                    if(e.cost == 1 && dist[from] + 1 < dist[e.to] ) {
                        dist[e.to] = dist[from] + 1;
                        prev[e.to] = from;
                        Eid[e.to] = e.idx;
                        dq.push_back(e.to);
                    }
                    else if(e.cost == 0 && dist[from] < dist[e.to]) {
                        dist[e.to] = dist[from];
                        prev[e.to] = from;
                        Eid[e.to] = e.idx;
                        dq.push_front(e.to);
                    }
                }
            }

            return {dist, prev, Eid};
        }       


        /// DFS
        // O(V+E), arg : start point
        // 最短経路であることは保証されない、重み付きグラフに対しては未定義動作（連結判定には使える）
        ShortestPath< T > dfs(int start = 0) { // Verified?
            assert(0 <= start && start < G.size() && "ERROR : out of bound graph access");

            int N = (int)G.size();
            vector< T > dist(N, inf);
            vector< int > prev(N, -1);
            vector< int > Eid(N, -1);
            dist[start] = 0;
            auto dfs_ = [&](auto dfs_, int from) -> void {
                for(auto &e : G[from]) {
                    if(dist[e.to] != inf) continue;
                    dist[e.to] = dist[from] + 1;
                    prev[e.to] = from;
                    Eid[e.to] = e.idx;
                    dfs_(dfs_, e.to);
                }
                return;
            };
            dfs_(dfs_, start);

            return {dist, prev, Eid};
        }


        /// 二部グラフに分類
        // O(V+E), ret : vector<(int)(0 or 1)>
        // if it is not bipartite, return {}
        vector< int > bipartite_grouping() { // Verified
            int N = G.size();
            vector< int > colors(N, -1);
            auto dfs = [&](auto dfs, int now, int col) -> bool {
                colors[now] = col;
                for(auto &e : G[now]) {
                    if(colors[e.to] != -1) {
                        if(col == colors[e.to]) return false;
                        else continue;
                    }
                    
                    if(!dfs(dfs, e.to, 1 - col)) return false;
                }
                return true;
            };

            for(int i = 0; i < N; i++) {
                if(colors[i] == -1 && !dfs(dfs, i, 0)) return vector< int >{};
            }

            return colors;
        }

        /// 二部グラフ判定
        bool is_bipartite() {
            return !bipartite_grouping().empty();
        }


        /// トポロジカルソート
        // Ο(V) (using DFS)
        // if a directed cycle exists, return {}        
        vector< int > topological_sort() { // Verified
            vector<int> ret;
            int N = G.size();
            vector<int> used(N, 0);
            bool not_DAG = false;
            auto dfs = [&](auto dfs, int now) -> void {
                if(not_DAG) return;
                if(used[now]) {
                    if(used[now] == 1) not_DAG = true;
                    return;
                }
                used[now] = 1;
                for(auto &e : G[now]) dfs(dfs, e.to);
                used[now] = 2;
                ret.push_back(now);                
            };

            for(int i = 0; i < N; i++) {
                dfs(dfs, i);
            }
            if(not_DAG) return vector< int >{};
            reverse(ret.begin(), ret.end());
            return ret;
        }

        /// DAG 判定
        bool is_DAG() { 
            return !topological_sort().empty(); 
        }


        /// Kruskal 法 (最小全域木)
        // O(E logV)
        // コストと辺集合を返す
        pair< T , Edges< T > > kruskal() { // Verified (TODO : Eをコストでソートして他が壊れないかの確認も)
            sort(E.begin(), E.end(), [&](const Edge< T > &a, const Edge< T > &b) {
                return a.cost < b.cost;
            });
            unionfind uf(G.size());
            T total_cost = T(0);
            Edges< T > edges;
            for(auto &e : E) {
                if(uf.issame(e.from, e.to)) continue;
                uf.unite(e.from, e.to);
                total_cost += e.cost;
                edges.emplace_back(e);
                }
            return {total_cost, edges};    
        }

        /// Prim法 (最小全域木)
        // O(E logV)
        // コストと辺集合を返す
        pair< T , Edges< T > > prim() { // Verified
            int N = G.size();
            using P = pair< T, int >;
            T total_cost = T(0);
            Edges< T > edges;
            int cnt = 0;
            vector< bool > used(N);
            priority_queue< P, vector< P >, greater<> > pq;
            
            for(auto &e : G[0]) pq.emplace(e.cost, e.to);
            used[0] = true;
            while(!pq.empty() && cnt < N) {
                auto [cost, to] = pq.top();
                pq.pop();
                if(used[to]) continue;
                used[to] = true;
                total_cost += cost;
                cnt++;
                for(auto &e : G[to]) pq.emplace(e.cost, e.to);
            }
            return {total_cost, edges};            
        }


        /// サイクル検出(多重辺の無向グラフにも対応)
        // 最初に見つかったサイクル1つを返す（無ければ空列を返す）
        // v を含むサイクルのみを検出するか？を withv で指定可能
        Cycle find_cycle(int v = 0, bool withv = false) { // Verified
            int N = G.size();
            vector< int > used(N, 0);
            vector< int > prev(N, -1);
            vector< int > Eid(N, -1);
            Cycle cycle;

            auto dfs = [&](auto dfs, int now, int pareidx) -> bool { 
                used[now] = 1;
                for(auto &e : G[now]) {
                    if(e.idx == pareidx) continue;
                    if(used[e.to] == 0) {
                        prev[e.to] = now;
                        Eid[e.to] = e.idx;
                        if(dfs(dfs, e.to, e.idx)) return true;
                    }
                    else if(used[e.to] == 1) {
                        int cur = now;
                        cycle.Vid.emplace_back(cur);
                        cycle.Eid.emplace_back(e.idx);
                        while(cur != e.to) {
                            cycle.Vid.emplace_back(prev[cur]);
                            cycle.Eid.emplace_back(Eid[cur]);
                            cur = prev[cur];
                        }
                        reverse(cycle.Vid.begin(), cycle.Vid.end());
                        reverse(cycle.Eid.begin(), cycle.Eid.end());
                        cycle.len = (int)cycle.Vid.size();
                        return true;
                    }
                }

                used[now] = 2;
                return false;
            };

            if(withv) {
                dfs(dfs, v, -1);
            }
            else {
                for(int i = 0; i < N; i++) {
                    if(used[i] == 0 && dfs(dfs, i, -1)) break;
                }
            }
            return cycle;
        }


        /// 連結成分に分解（TODO : 機能を増やす？、dsu非使用にする？）
        // 各連結成分の代表元のindexにその連結成分中の頂点番号を全て含める、有向グラフに対しては未定義動作
        vector< vector< int > > components() { // Verified
            int N = G.size();
            vector< vector< int > > ret(N);
            unionfind uf(N);
            for(auto &e : E) uf.unite(e.from, e.to);
            for(int i = 0; i < N; i++) ret[uf.root(i)].emplace_back(i);
            return ret;
        }

    private :
        int edge_num;
        bool negative_edge;
        bool is_weighted;
        bool is_directed;

        /// 隣接行列を構築
        // 多重辺の場合は未定義動作だが、最短路を優先
        void build_adjmat() {
            int N = G.size();
            AdjMat.resize(N);
            for(int i = 0; i < N; i++) AdjMat[i].assign(N, inf);
            for(int i= 0; i < N; i++) AdjMat[i][i] = T(0);
            idxMat.resize(N);
            for(int i = 0; i < N; i++) idxMat[i].assign(N, -1);

            for(auto &e:E) {
                if(e.cost < AdjMat[e.from][e.to]) {
                    AdjMat[e.from][e.to] = e.cost;
                    idxMat[e.from][e.to] = e.idx;
                }
            }
        }



};

        /// 全サイクル列挙 (https://kopricky.github.io/code/Academic/finding_all_cycles.html)
        // 


        /// サイクル基底



        /// 橋・関節点
        


        /// オイラーグラフ判定、オイラー路


        /// 強連結成分分解





