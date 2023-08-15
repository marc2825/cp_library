/*
★ Name        ： Graph/Edge template
 ■ Snipet      : No

 ■ Arguments   : 
 ■ Return      : 
 ■ Complexity  : 

 ■ Description : 
 

 ■ Usage       : 


 ■ Verify      : 
 ■ References  : https://ei1333.github.io/library/graph/graph-template.hpp
                https://github.com/tsuchinoko29/nok0/blob/main/graph/graph.hpp
                https://github.com/magurofly/cp-library-rs
                https://nyaannyaan.github.io/library/graph/graph-template.hpp

 ■ TODO (Main) : bfs等も組み込む、グリッドグラフ用、木用の構造体も別でつくる
*/

#pragma once
#include <vector>
#include <queue>
#include <limits>
#include <cassert>
#include <cmath>

using namespace std; // TODO : stdをちゃんと書く


/// 辺 (始点 from、終点 to、コスト cost、インデックス idx) を管理する構造体 Edge
// cost の型Tを渡す (初期設定は long long)
template< typename T = long long >
struct Edge{
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

    ShortestPath(int N) : {
        dist.resize(N);
        prev.resize(N);
        Eid.resize(N);
    }

    ShortestPath(vector< T > &dist_) : dist(dist_) {
        prev.assign(dist.size(), -1); // TODO : 順番が大丈夫か確認
        Eid.assign(dist.size(), -1);
    }
};

// TODO : verify all functions
/// (重み有り有向)グラフを管理する構造体 Graph
//  重み無し -> cost all 1, 無向 -> 両方向に有向辺 と帰着させるような実装
template< typename T = long long >
using graph = vector< vector< Edge< T > > >;
template< typename T = long long >
using tree = graph;

template< typename T = long long >
class Graph {
    /* 基本構造 */
    private :
        int edge_num;
        bool negative_edge;
        const auto inf = numeric_limits< T >::max();


    public :
        graph G;
        vector< vector< T > > AdjMat; 
        vector< vector< int > > idxMat; // 隣接行列の辺番号（あれば） 
        tree Tree; // TODO : Tree 構造体をちゃんとつくる
        Edges E;

        Graph() = default;

        explicit Graph(int n) : G(n), edge_num(0), negative_edge(false) {}

        /// グラフの頂点数を返す
        size_t size() const { 
            return G.size();
        }

        /// グラフの辺の数を返す
        // 無向グラフの場合は E.size() != edge_num となることに注意
        size_t e_size() const {
            return (size_t)edge_num;
        }

        /// 有向辺を追加する
        void add_directed_edge(int from, int to, T cost = 1) {
            assert(0 <= from && from < G.size() && "ERROR : out of bound graph access");
            assert(0 <= to && to < G.size() && "ERROR : out of bound graph access");

            Edge e = {from, to, cost, edge_num};
            G[from].emplace_back(e);
            E.emplace_back(e);

            if(cost < T(0)) negative_edge = true;
            edge_num++
        }

        /// 無向辺を追加する 
        // 一回で有向辺を両方向に追加していることに注意
        void add_edge(int from, int to, T cost = 1) {
            assert(0 <= from && from < G.size() && "ERROR : out of bound graph access");
            assert(0 <= to && to < G.size() && "ERROR : out of bound graph access");

            Edge e = {from, to, cost, edge_num};
            G[from].emplace_back(e);
            E.emplace_back(e);

            Edge inve = {to, from, cost, edge_num};
            G[to].emplace_back(inve);
            E.emplace_back(inve);

            if(cost < T(0)) negative_edge = true;
            edge_num++;
        }

        /// グラフ（隣接リスト）の一括入力
        // offset (0-indexedや複数グラフ用)、weighted (重み有り/無し)、directed (有向/無向) を指定する
        void input(int M, int offset = -1, bool weighted = false, bool directed = false) {
            for(int i = 0; i < M; i++) {
                int a, b;
                cin >> a >> b;
                a += offset;
                b += offset;
                T c = T(1);
                if(weighted) cin >> c;
                if(directed) add_directed_edge(a, b, c);
                else add_edge(a, b, c);
            }
        }

        /// グラフ（隣接行列）の一括入力
        void input_adjmat() {
            resize(AdjMat, N);
            for(int i = 0; i < N; i++) {
                for(int j = 0; j < N; j++) {
                    T cost; cin >> cost;
                    AdjMat[i].emplace_back(cost);
                }
            }
        }

        /// 隣接行列を構築
        // 多重辺の場合は未定義動作だが、最短路を優先
        static void build_adjmat() {
            resize(AdjMat, N);
            for(int i = 0; i < N; i++) AdjMat[i].assign(N, inf);
            for(int i= 0; i < N; i++) AdjMat[i][i] = T(0);
            resize(idxMat, N);
            for(int i = 0; i < N; i++) idxMat[i].assign(N, -1);

            for(auto &e:E) {
                if(e.cost < AdjMat[e.from][e.to]) {
                    AdjMat[e.from][e.to] = e.cost;
                    idxMat[e.from][e.to] = e.idx;
                }
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
        ShortestPath< T > dijkstra(int start = 0) {
            assert(!negative_edge && "ERROR : cannot use dijkstra with negative edge");
            assert(0 <= start && start < G.size() && "ERROR : out of bound graph access");

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
        // 始点 start を指定、負閉路があるかどうかは dist[start] < 0 ? で判断可能、cannnot reach : numeric_limits< T >::max()
        ShortestPath< T > bellman_ford(int start = 0) {
            assert(0 <= start && start < G.size() && "ERROR : out of bound graph access");

            int N = (int)G.size();
            vector< T > dist(N, inf);
            vector< int > prev(N, -1);
            vector< int > Eid(N, -1);
            dist[start] = 0;

            for(int iter = 0; iter < N; iter++) {
                bool update = false;
                for(int v = 0; v < N; v++) {
                    if(dist[v] == inf) continue;

                    for(auto &e : G[v]) {
                        T nxtcost = dist[v] + e.cost;
                        if(nxtcost < dist[e.to]) {
                            dist[e.to] = nxtcost;
                            prev[e.to] = v;
                            Eid[e.to] = e.idx;
                            update = true;
                        }
                    }

                    if(!update) break;

                    if(iter == N-1 && update) { // 負閉路検出
                        dist[start] = T(-1);
                    }
                }
            }

            return {dist, prev, Eid};      
        }


        /// 単一始点最短路問題ソルバー
        // 始点 start を指定
        // 負辺が無ければ Dijkstra、有れば Bellman-Ford
        ShortestPath< T > SingleSource_ShortestPath(int start = 0) {
            if(!negative_edge) dijkstra(start);
            else bellman_ford(start);
        }


        /// Floyd–Warshall 法 (全点対間最短路)
        // O(V^3)、負辺OK
        vector< ShortestPath< T > > floyd_warshall() { // TODO : 経路復元を要検証
            if(AdjMat.empty()) build_adjmat();
            vector< ShortestPath< T > > ret(N);
            for(int i = 0; i < N; i++) ret[i](AdjMat[i]);  
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
                        T nxtcost = ret[i].dist[k] + rest[k].[j];
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
        vector< ShortestPath< T > > Allpair_ShortestPath() {
            if(negative_edge) floyd_warshall();
            else { // TODO : verify ちゃんとすべき
                long long V = G.size();
                if(V * V * V <= V * log2(V) * e_size()) floyd_warshall();
                else {
                    vector< ShortestPath< T > > ret(N);
                    for(int i = 0; i < N; i++) ret[i] = dijkstra(i);
                    return ret;
                }
            }
        }





};

