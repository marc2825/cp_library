/*
★ Name        ： Graph/Edge template
 ■ Snipet      : No

 ■ Arguments   : 
 ■ Return      : 
 ■ Complexity  : 

 ■ Description : （ここに含まれてるデアを書く）-> oj bundleを活用して分割してもいいかも
 

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
                              https://atcoder.jp/contests/typical90/submissions/51426032
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
                                      https://atcoder.jp/contests/abc142/submissions/54725039
                                      https://atcoder.jp/contests/abc233/submissions/55200987 (Eid, prev 復元)
                 ・DFS              : https://atcoder.jp/contests/abc251/submissions/44695547 (DFS木)
                 ・0-1BFS           : https://atcoder.jp/contests/abc184/submissions/44695926
                 ・二部グラフ        : https://atcoder.jp/contests/abc282/submissions/44696396 
                                      https://atcoder.jp/contests/typical90/submissions/51471838
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
                 ・橋               : https://judge.u-aizu.ac.jp/onlinejudge/review.jsp?rid=9368892#1
                                      https://atcoder.jp/contests/abc075/submissions/54922838
                                      https://codeforces.com/contest/1986/submission/267284311
                 ・関節点           : https://judge.u-aizu.ac.jp/onlinejudge/review.jsp?rid=9368902#1
                 ・二重辺連結成分分解: https://judge.yosupo.jp/submission/217039
                                     https://atcoder.jp/contests/arc039/submissions/54927935
                                     https://codeforces.com/contest/1000/submission/267313257
                 ・二重（頂点）連結成分分解: https://judge.yosupo.jp/submission/217255
                 ・Block Cut Tree   : https://atcoder.jp/contests/abc318/submissions/54961604
                                      https://atcoder.jp/contests/abc334/submissions/54961918
                                    　https://atcoder.jp/contests/nadafes2022_day2/submissions/54974497 (関節点)
                　・LowLink         : https://atcoder.jp/contests/abc334/submissions/54977970

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

/// Tree / UF ライブラリとの互換性/依存性を考える、小分けにしてoj bundle を活用できるようにする？

#pragma once
#include <vector>
#include <queue>
#include <deque>
#include <stack>
#include <algorithm>
#include <cmath>
#include <cassert>
#include <limits>
//#include "./Tree/tree_template.hpp" // 二重辺連結成分分解で使用
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


/// TODO : グラフアルゴリズム全部載せGraph構造体周りを完成させる、他ライブラリとの互換性/依存性を整理する

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


/// 橋・関節点の検出等に用いる構造体 Low Link
template< typename T = long long >
struct LowLink {
    vector< int > ord; // DFSで頂点iにいつ訪れたか？
    vector< int > low; // 頂点iからDFS木の葉方向の辺を0回以上、後退辺（根方向の辺）を1回以下通って到達可能な頂点のordの最小値

    vector< int > articulation; // 関節点の頂点番号 (TODO: グラフの分解後の連結成分サイズ集合、二重頂点連結成分分解的にやればできそうだが面倒？)
    vector< pair<Edge<T>, long long > > bridge; // (橋の辺情報、グラフの分解後の片側の連結成分サイズ) 
    
    LowLink() = default;

};

template<class T> class Tree;

/// (重み有り有向)グラフを管理する構造体 Graph
//  重み無し -> cost all 1, 無向 -> 両方向に有向辺 と帰着させるような実装
// TODO: 頂点に重みをつける
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
        LowLink< T > LL;

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
        void add_directed_edge(int from, int to, T cost = 1, int idx = -1) { // Verified
            assert(0 <= from && from < (int)G.size() && "ERROR : out of bound graph access");
            assert(0 <= to && to < (int)G.size() && "ERROR : out of bound graph access");
            
            if(idx != -1) swap(idx, edge_num);
            Edge< T > e = {from, to, cost, edge_num};
            if(idx != -1) swap(idx, edge_num);
            G[from].emplace_back(e);
            E.emplace_back(e);

            if(cost < T(0)) negative_edge = true;
            edge_num++;
        }

        /// 無向辺を追加する 
        // 一回で有向辺を両方向に追加していることに注意（Eには便宜上両方追加しているが、どうするか未定）
        void add_edge(int from, int to, T cost = 1, int idx = -1) { // Verified
            assert(0 <= from && from < (int)G.size() && "ERROR : out of bound graph access");
            assert(0 <= to && to < (int)G.size() && "ERROR : out of bound graph access");

            if(idx != -1) swap(idx, edge_num);
            Edge< T > e = {from, to, cost, edge_num};
            G[from].emplace_back(e);
            E.emplace_back(e);

            Edge< T > inve = {to, from, cost, edge_num};
            G[to].emplace_back(inve);
            E.emplace_back(inve);
            if(idx != -1) swap(idx, edge_num);

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
                vector<int> update;
                for(int v = 0; v < N; v++) {
                    if(dist[v] == inf) continue;

                    for(auto &e : G[v]) {
                        T nxtcost = dist[v] + e.cost;
                        if(nxtcost < dist[e.to]) {
                            dist[e.to] = nxtcost;
                            prev[e.to] = v;
                            Eid[e.to] = e.idx;
                            update.push_back(e.to);
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


        /// 関節点の検出
        // https://ei1333.github.io/luzhiled/snippets/graph/lowlink.html
        // Low Link を用いて O(V+E)
        // 関節点となる頂点集合 を返す
        vector< int > Articulation() {
            build_LowLink();
            return LL.articulation;
        }

        /// 橋の検出
        // Low Link を用いて O(V+E)
        // (橋の辺情報、グラフの分解後の片側の連結成分サイズ) を返す
        vector< pair<Edge<T>, long long > > Bridge() {
            build_LowLink();
            return LL.bridge;
        }

        /// 二重辺連結成分分解（辺を1個取り除いても連結である部分グラフ）、連結成分内の任意の2点間には辺素なパスが2本以上存在する
        // Low Link （による橋の検出）を用いて O(V+E)
        // (各頂点が縮約後のグラフのどの頂点に対応するか？、縮約後のグラフ（木）)が返される -> 連結成分が木になる
        // compのコメントアウトを外して、縮約後の各頂点に含まれる頂点集合を返すようにしても良い
        //pair<vector< vector< int > >, Tree< T > >
        pair<vector< int > , Tree< T > > TwoEdgeConnectedComponents() {
            build_LowLink();
            int N = G.size();
            vector< int > inv(N, -1);

            int k = 0;
            auto dfs = [&](auto dfs, int cur, int par)->void{
                if(par != -1 && LL.ord[par] >= LL.low[cur]) inv[cur] = inv[par];
                else inv[cur] = k++;
                for(auto &e : G[cur]) {
                    if(inv[e.to] == -1) dfs(dfs, e.to, cur);
                }
            };

            for(int i=0; i<N; i++) {
                if(inv[i] == -1) dfs(dfs, i, -1);
            }


            //vector< vector< int > > comp(k);
            //rep(i,N) comp[inv[i]].pb(i);

            Tree< T > TECC(k);
            
            for(auto& b:LL.bridge) {
                TECC.add_edge(inv[b.first.from], inv[b.first.to], b.first.cost, b.first.idx);
                // TECC[inv[b.first.from]].emplace_back(Edge<T>(inv[b.first.from], inv[b.first.to], b.first.cost, b.first.idx)); // 木ではなくグラフとして扱いたい場合
                // TECC[inv[b.first.to]].emplace_back(Edge<T>(inv[b.first.to], inv[b.first.from], b.first.cost, b.first.idx));
            }
        
            return {inv, TECC};
        }

        /// 二重（頂点）連結成分分解
        // Low Link を用いて O(V+E)
        // 各二重連結成分（＝任意の頂点を取り除いても連結である部分グラフのうち極大なもの）に含まれる「頂点集合」（関節点は重複して含まれる）を返す
        vector< vector< int > >  BiConnectedComponents() {
            build_LowLink();
            int N = G.size();
            vector< bool > used(N, false);
            vector< vector< int > > comp;
            vector< int > st;
            st.reserve(N);

            auto dfs = [&](auto dfs, int cur, int par)->void{
                used[cur] = true;
                st.push_back(cur);

                int kids = 0;
                for(auto& e : G[cur]) {
                    if(e.to == par) continue;
                    if(!used[e.to]) {
                        kids++;
                        int s = st.size();
                        dfs(dfs, e.to, cur);
                        
                        if((par == -1 && kids > 1) || (par != -1 && LL.low[e.to] >= LL.ord[cur])) { // cur は関節点なので、e.to方向の部分木は二重連結成分になる
                            comp.push_back({});
                            comp.back().push_back(cur);
                            while((int)st.size() > s) {
                                comp.back().push_back(st.back());
                                st.pop_back();
                            }
                        }
                    }
                }
            };

            for(int i=0; i<N; i++) {
                if(!used[i]) {
                    dfs(dfs, i, -1);
                    comp.push_back({});
                    for(auto &x: st) comp.back().push_back(x);
                    st.clear();
                }
            }
            
            return comp;
        }


        /// Block Cut Tree
        // Low Link を用いて O(V+E)
        // 二重連結成分(block)を関節点で繋ぎ合わせて木にしたもの -> 関節点は複数の二重連結成分(block)内にも含まれており、blockが主役であることに注意
        // block-cut tree を、block に通常の頂点を隣接させて拡張しておく （https://twitter.com/noshi91/status/1529858538650374144?s=20&t=eznpFbuD9BDhfTb4PplFUg と Maspy先生のライブラリ参照)
        // - [0, n)：もとの頂点 
        // - [n, n + n_block)：block
        // - 関節点：[0, n) のうちで、degree >= 2 を満たすもの
        // 孤立点は、1 点だけからなる block
        // この木が返される、各[0,n)の頂点の隣接頂点（集合）が所属するblockとなる
        Tree< T > BlockCutTree() {
            build_LowLink();
            int N = G.size();
            vector< int > st;
            st.reserve(N);
            vector< bool > used(N);

            Tree< T > BCT(N);
            int nxt = N;
            auto dfs = [&](auto& dfs, int cur, int par)->void{
                st.push_back(cur);
                used[cur] = true;
                int kids = 0;
                
                for(auto &e: G[cur]) {
                    if(e.to == par) continue;
                    if(!used[e.to]) {
                        kids++;
                        int s = st.size();
                        dfs(dfs, e.to, cur);
                        
                        if((par == -1 && kids > 1) || (par != -1 && LL.low[e.to] >= LL.ord[cur])) { // 関節点なのでつなげる
                            BCT.resize(nxt + 1);
                            BCT.add_edge(nxt, cur);
                            while((int)st.size() > s) {
                                BCT.add_edge(nxt, st.back());
                                st.pop_back();
                            }
                            nxt++;
                        }
                        
                    }
                }
            };

            for(int i=0; i<N; i++) {
                if(!used[i]) {
                    dfs(dfs, i, -1);
                    BCT.resize(nxt + 1);
                    for(auto &x: st) BCT.add_edge(nxt, x);
                    nxt++;
                    st.clear();
                }
            }

            return BCT;     
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

        /// LowLinkの計算
        // 単純グラフ/連結グラフでなくても壊れない（はず）、有向はトポロジカルソート順でdfsすればできそう（非対応）
        void build_LowLink() {
            if(!LL.low.empty()) return;

            int N = (int)G.size();
            vector< bool > used(N);
            LL.low.resize(N);
            LL.ord.resize(N);

            int k = 0;
            auto dfs = [&](auto dfs, int cur, int par) -> int {
                used[cur] = true;
                LL.ord[cur] = k++;
                LL.low[cur] = LL.ord[cur];
                
                bool is_art = false;
                int kidcnt = 0;
                int cursz = 1;
                bool pare = true; // 多重辺用
                for(auto &e : G[cur]) {
                    if(!used[e.to]) {
                        kidcnt++;
                        int kidsz = dfs(dfs, e.to, cur);
                        cursz += kidsz;
                        LL.low[cur] = min(LL.low[cur], LL.low[e.to]); // low linkの更新
                        if(par != -1 && LL.low[e.to] >= LL.ord[cur]) is_art = true; // curの部分木に関して、子がその部分木内にしかlowlinkで遡れない場合、関節点確定
                        if(LL.ord[cur] < LL.low[e.to]) LL.bridge.emplace_back(e, kidsz); // curの部分木に関して、子がcur以下にしかlowlinkで遡れない場合、橋確定
                    }
                    else if(e.to != par || !pare) { // 後退辺なのでlow linkの更新
                        LL.low[cur] = min(LL.low[cur], LL.ord[e.to]);
                    }
                    else pare = false;
                }

                if(par == -1 && kidcnt > 1) is_art = true; // 根が関節点となる場合
                if(is_art) LL.articulation.emplace_back(cur);

                return cursz;
            };

            for(int i=0; i<N; i++) {
                if(!used[i]) dfs(dfs, i, -1);
            }
            
        }

};

        /// 全サイクル列挙 (https://kopricky.github.io/code/Academic/finding_all_cycles.html)
        // 

        /// 最小サイクル検出 -> 全頂点始点BFSで（もっと良いやり方があるかも？）

        /// サイクル基底

        /// link cut tree


        /// オイラーグラフ判定、オイラー路


        /// 強連結成分分解





