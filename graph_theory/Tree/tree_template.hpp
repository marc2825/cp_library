/*
★ Name        ：木に関するアルゴリズム詰め合わせ
 ■ Snipet      : No
 
 ■ Arguments   :
 ■ Return      :
 ■ Complexity  :

 ■ Description :
 

 ■ Usage       :


 ■ Verify      : https://atcoder.jp/contests/tessoku-book/submissions/44897237 (部分木の大きさ)
                 https://atcoder.jp/contests/editor-update-test/submissions/44912089 (部分木の大きさ)
                 https://atcoder.jp/contests/abc070/submissions/44904708 (重み付き深さ)
                 https://atcoder.jp/contests/abc070/submissions/44904821 (重み付き距離)
                 https://judge.yosupo.jp/submission/157557 (LCA)
                 https://atcoder.jp/contests/abc014/submissions/44909147 (重みなし距離)
                 https://atcoder.jp/contests/abc267/submissions/44920179 (重み無し直径、根の変更、Level Ancestor Problem(kth_ancestor))
                 https://judge.yosupo.jp/submission/157556 (重み付き直径、パス上の頂点列挙)
                 https://atcoder.jp/contests/abc291/submissions/44911963 (重心分解)
                 https://atcoder.jp/contests/typical90/submissions/44905006 (オイラーツアー、重み無し距離)
                 https://atcoder.jp/contests/abc294/submissions/44918371 (オイラーツアー（辺、頂点）、辺情報、重み付き、パスクエリ)
                 https://atcoder.jp/contests/abc294/submissions/44919946 (HL分解、辺情報、重み付き、パスクエリ)
                 https://atcoder.jp/contests/arc039/submissions/54927935 (is_on_path, 二重辺連結成分分解後の木)
                 https://codeforces.com/contest/1000/submission/267313257 (直径、二重辺連結成分分解後の木)
        
        TODO　 : https://judge.yosupo.jp/problem/jump_on_tree
                 https://judge.yosupo.jp/problem/frequency_table_of_tree_distance


                 
 ■ References  : https://nyaannyaan.github.io/library/tree/tree-query.hpp
                 https://hcpc-hokudai.github.io/archive/graph_tree_001.pdf
                 https://maspypy.com/euler-tour-%E3%81%AE%E3%81%8A%E5%8B%89%E5%BC%B7
                 https://qiita.com/drken/items/4b4c3f1824339b090202
                 https://kopricky.github.io/code/GraphDecomposition/centroid_decomposition.html

 ■ TODO        : 重み付き木に対応させる（Graph, Edge 構造体を有効活用する）
                 オイラーツアーやHLD分解や重心分解に典型クエリは組み込んでしまうか、別の構造体にまとめるか、
                 初期化処理をコンストラクタに組み込むかどうか（graph構造体とのすり合わせ関連）
                 easiest_HLDの実装（こどふぉのやつ）を組み込むか
                 depth[]と直接アクセスしてるのをdepth()と変えるか？
                 無向グラフの場合にEに両方向分追加するか？（e.idxとの辻褄合わせの問題）
                 オイラーツアーの辺のidx,辺と頂点とのすり合わせはこのままでいいか
                 HL分解で辺に情報をもたせる場合をもっと洗練できないか

*/

/// Graph / UF ライブラリとの互換性を考える、oj bundle を活用できるようにする

#pragma once
#include <vector>
#include <algorithm>
#include <cassert>
//#include "../graph_template.hpp"


/// Tree単体の場合は、以下も含める

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

template< typename T = long long >
using graph = vector< vector< Edge< T > > >;



template <typename T = long long>
class Tree {
    private:
        int edge_num;
        bool builded = false;
        bool isweighted = false;

        void dfs_init(int now, int prev, int dep) {
            sz[now] = 1;
            par_dbl[now].push_back(prev);
            depth[now] = dep;
            for(auto &e : G[now]) {
                if(e.to == prev) continue;
                if(isweighted) dist_weighted[e.to] = dist_weighted[now] + e.cost;
                dfs_init(e.to, now, dep + 1);
                sz[now] += sz[e.to];
            }
            return;
        }

        // O(NlogN)
        void doubling() {
            int N = G.size();
            int cnt = 0;
            while(true) {
                bool update = false;
                for(int i = 0; i < N; i++) {
                    if(depth[i] < (1<<cnt) || par_dbl[i][cnt] == root || par_dbl[i][cnt] == -1) continue;
                    update = true;
                    par_dbl[i].push_back(par_dbl[par_dbl[i][cnt]][min(cnt, (int)par_dbl[par_dbl[i][cnt]].size()-1)]);
                }
                if(!update) break;
                cnt++;
            }
        }


    public:
        graph< T > G;
        Edges< T > E;
        int root; // 根
        vector<vector<int> > par_dbl; // 2^j 個上の祖先
        vector<int> depth; // 深さ
        vector<int> sz; // 部分木の大きさ
        vector< T > dist_weighted; // 重み付きの距離
        vector<int> ET_V; // オイラーツアー（頂点）
        vector<int> ET_Vin; // オイラーツアー（頂点）の初訪問index
        vector<int> ET_Vout; // オイラーツアー（頂点）の最終訪問index
        vector<int> ET_E; // オイラーツアー（辺）、辺番号は入力順(e.idx)
        vector<int> ET_Ein; // オイラーツアー（辺）の初訪問index
        vector<int> ET_Eout; // オイラーツアー（辺）の最終訪問index
        vector<int> HLD; // HL分解後の連結成分を並べた配列
        vector<int> HLD_pos; // HLD配列中における各頂点のindex
        vector<int> HLD_head; // HL分解後の連結成分で一番浅い頂点
        vector<bool> CD_removed;  // 重心分解で用いるフラグ (v が既に取り除かれたかどうか)
        vector<int> CD_sz;  // 重心分解後の部分木の大きさ
        vector<int> CD_par;  // 重心分解後の親


        explicit Tree(int n, int root_ = 0) : G(n), root(root_), edge_num(0) {}

        /// グラフの頂点数を返す
        size_t size() const { 
            return G.size();
        }

        /// 有向辺を追加する
        void add_directed_edge(int from, int to, T cost = 1, int idx = -1) {
            assert(0 <= from && from < (int)G.size() && "ERROR : out of bound graph access");
            assert(0 <= to && to < (int)G.size() && "ERROR : out of bound graph access");
            
            if(idx != -1) swap(idx, edge_num);
            Edge< T > e = {from, to, cost, edge_num};
            if(idx != -1) swap(idx, edge_num);
            G[from].emplace_back(e);
            E.emplace_back(e);
            
            edge_num++;
            assert(edge_num <= (int)G.size() - 1 && "ERROR : too many edges for tree");
        }

        /// 無向辺を追加する 
        // 一回で有向辺を両方向に追加していることに注意（Eには便宜上片方のみ追加しているが、どうするか未定）
        void add_edge(int from, int to, T cost = 1, int idx = -1) {
            assert(0 <= from && from < (int)G.size() && "ERROR : out of bound graph access");
            assert(0 <= to && to < (int)G.size() && "ERROR : out of bound graph access");
            
            if(idx != -1) swap(idx, edge_num);
            Edge< T > e = {from, to, cost, edge_num};
            G[from].emplace_back(e);
            E.emplace_back(e);

            Edge< T > inve = {to, from, cost, edge_num};
            G[to].emplace_back(inve);
            if(idx != -1) swap(idx, edge_num);

            edge_num++;
            assert(edge_num <= (int)G.size() - 1 && "ERROR : too many edges for tree");
        }

        /// 木グラフ（隣接リスト）の一括入力
        // offset (0-indexedや複数グラフ用)、weighted (重み有り/無し)、directed (有向/無向) を指定する
        void input(int offset = -1, bool weighted = false, bool directed = false, int M = -1) {
            if(M == -1) M = G.size() - 1;
            for(int i = 0; i < M; i++) {
                int a, b;
                cin >> a >> b;
                a += offset;
                b += offset;
                T c = T(1);
                if(weighted) {
                    isweighted = true;
                    cin >> c;
                }
                if(directed) add_directed_edge(a, b, c);
                else add_edge(a, b, c);
            }
        }

        /// 木グラフ（親の頂点番号形式）の一括入力
        void input_par(int offset = -1, bool weighted = false) { // Verified
            for(int i = 0; i < (int)G.size(); i++) {
                if(i == root) continue;
                int p;
                cin >> p;
                p += offset;
                T c = T(1);
                if(weighted) {
                    isweighted = true;
                    cin >> c;
                }
                add_directed_edge(p, i, c);               
            }
        }

        /// 深さ・親・2^n個上の祖先の前計算を行う
        void build() {
            if(builded) return;
            if(isweighted) dist_weighted.assign(G.size(), 0);
            builded = true;
            depth.resize(G.size());
            sz.resize(G.size());
            par_dbl.resize(G.size());
            dfs_init(root, root, 0);
            doubling();
        }

        /// 木の根を変更した上で、深さ・親・2^n個上の祖先の前計算を再び行う
        // build()で構築される配列以外は初期化していないので注意
        void rebuild(int root_) {
            root = root_;
            if(isweighted) dist_weighted.assign(G.size(), 0);
            builded = true;
            depth.assign(G.size(), 0);
            sz.assign(G.size(), 0);
            par_dbl.clear();
            par_dbl.resize(G.size());
            dfs_init(root, root, 0);
            doubling();       
        }

        /// vの深さを返す
        int dep(int v) {
            assert(0 <= v && v < (int)G.size() && "ERROR : out of bound graph access");
            if(!builded) build();
            return depth[v];
        }

        /// vの親を返す
        int par(int v) {
            assert(0 <= v && v < (int)G.size() && "ERROR : out of bound graph access");
            if(!builded) build();
            return par_dbl[v][0];            
        }

        /// vを根とする部分木の大きさを返す
        int size(int v = -1) { // Verified
            if(v == -1) v = root;
            assert(0 <= v && v < (int)G.size() && "ERROR : out of bound graph access");
            if(!builded) build();
            return sz[v];            
        }

        /// v の k個上の祖先の頂点番号を返す
        // O(logN)
        int kth_ancestor(int v, int k) {
            assert(0 <= v && v < (int)G.size() && "ERROR : out of bound graph access");
            assert(0 <= k && "ERROR : k cannot be negative");
            if(!builded) build();
            if(depth[v] < k) return -1;
            int cnt = 0;
            while(k > 0) {
                if(k & 1) v = par_dbl[v][cnt];
                cnt++;
                k >>= 1;
            }
            return v;
        }

        /// v と u の LCA を返す
        // O(log N)
        int LCA(int u, int v) {
            assert(0 <= v && v < (int)G.size() && "ERROR : out of bound graph access");
            assert(0 <= u && u < (int)G.size() && "ERROR : out of bound graph access");
            if(!builded) build();
            if(depth[u] != depth[v]) {
                if(depth[u] > depth[v]) swap(u,v);
                v = kth_ancestor(v, depth[v] - depth[u]);
            }
            if(u == v) return u;
            for(int i = par_dbl[v].size() - 1; i >= 0; i--) {
                if(depth[u] < (1 << i)) continue;
                if(par_dbl[u][i] != par_dbl[v][i]) u = par_dbl[u][i], v = par_dbl[v][i];                
            }
            return par_dbl[u][0];
        }

        /// v と u の距離を返す
        // O(log N)
        T dist(int u, int v) {
            assert(0 <= v && v < (int)G.size() && "ERROR : out of bound graph access");
            assert(0 <= u && u < (int)G.size() && "ERROR : out of bound graph access");
            if(!builded) build();
            if(!isweighted) return depth[u] + depth[v] - 2 * depth[LCA(u, v)];
            else return dist_weighted[u] + dist_weighted[v] - 2 * dist_weighted[LCA(u, v)];
        }

        /// s-t間のパス上に、頂点vが含まれるか？を判定
        // O(log N)
        bool is_on_path(int s, int t, int v) {
            assert(0 <= s && s < (int)G.size() && "ERROR : out of bound graph access");
            assert(0 <= t && t < (int)G.size() && "ERROR : out of bound graph access");
            assert(0 <= v && v < (int)G.size() && "ERROR : out of bound graph access");
            if(!builded) build();
            return dist(s, v) + dist(v, t) == dist(s, t);
        }

        /// s-t間のパスに含まれる頂点の列を返す
        // O((パスの長さ))
        vector<int> path(int s, int t) {
            assert(0 <= s && s < (int)G.size() && "ERROR : out of bound graph access");
            assert(0 <= t && t < (int)G.size() && "ERROR : out of bound graph access");
            if(!builded) build();
            vector<int> pre, suf;
            while (depth[s] > depth[t]) {
                pre.push_back(s);
                s = par_dbl[s][0];
            }
            while (depth[s] < depth[t]) {
                suf.push_back(t);
                t = par_dbl[t][0];
            }
            while (s != t) {
                pre.push_back(s);
                suf.push_back(t);
                s = par_dbl[s][0];
                t = par_dbl[t][0];
            }
            pre.push_back(s);
            reverse(suf.begin(), suf.end());
            copy(suf.begin(), suf.end(), back_inserter(pre));
            return pre;
        }

        /// s-t間のパス上の点のうち、sに隣接する頂点を返す
        // O(logN)
        int nxt(int s, int t) {
            if(!builded) build();
            if (depth[s] >= depth[t]) return par(s);
            int u = kth_ancestor(t, depth[t] - depth[s] - 1);
            return par_dbl[u][0] == s ? u : par_dbl[s][0];
        }

        /// 木の重心(1or2個)を返す
        // O(N)
        vector<int> centroid(int root_ = -1, int par = -1) {
            if(root_ == -1) root_ = root;
            if(CD_removed.empty()) {
                if(!builded) build();
                CD_removed.assign(G.size(), false);
                CD_sz = sz;
            }
            else {
                auto calc_size = [&](auto calc_size, int now, int par_) -> void {
                    CD_sz[now] = 1;
                    for(auto &e : G[now]) {
                        if(e.to == par_ || CD_removed[e.to]) continue;
                        calc_size(calc_size, e.to, now);
                        CD_sz[now] += CD_sz[e.to];
                    }
                    return;
                };
                calc_size(calc_size, root_, par);
            }


            vector<int> ret;
            int size = CD_sz[root_];
            auto dfs_Cen = [&](auto dfs_Cen, int now, int par_) -> void{
                bool isCentroid = true;
                for(auto &e : G[now]) {
                    if(e.to == par_ || CD_removed[e.to]) continue;
                    if(size <= 2 * CD_sz[e.to]) {
                        if(size < 2 * CD_sz[e.to]) isCentroid = false;
                        dfs_Cen(dfs_Cen, e.to, now);
                    }
                }
                if(isCentroid && 2 * (size - CD_sz[now]) <= size) ret.push_back(now);
            };
            dfs_Cen(dfs_Cen, root_, par);
            return ret;
        }

        /// 木の重心分解（全体に対して）
        // 一回でO(N)、再帰的に木全体に対して行っており、O(NlogN) (同心円状に分割統治を行う際に用いる)
        // 必要な操作を適宜書き加えたり（返り値の型等を）書き換えたりする、重心分解後の情報は CD(sz/par)でアクセス可能
        void centroid_decomposition(int root_ = -1, int par = -1) {
            if(root_ == -1) root_ = root;
            if(CD_par.empty()) {
                if(!builded) build();
                CD_par.resize(G.size());
                for(int i = 0; i < (int)G.size(); i++) CD_par[i] = par_dbl[i][0];
            }

            int center = centroid(root_, par)[0];
            CD_par[center] = par;


            /* 何らかの操作 */


            CD_removed[center] = true;
            for (auto &e : G[center]) {
                if (CD_removed[e.to]) continue;
                centroid_decomposition(e.to, center);
            }
        }


        /// 木の直径の大きさと端点を返す
        // O(N)
        pair<T, pair<int, int> > diameter() {
            if(!builded) build();
            int p;
            if(!isweighted) p = max_element(depth.begin(), depth.end()) - depth.begin();
            else p = max_element(dist_weighted.begin(), dist_weighted.end()) - dist_weighted.begin();
            int q = 0;
            T mxd = 0;
            auto dfs_dia = [&](auto dfs_dia, int now, int par, T d) -> void {
                if(d > mxd) {
                    mxd = d;
                    q = now;
                }
                for(auto &e : G[now]) {
                    if(e.to == par) continue;
                    dfs_dia(dfs_dia, e.to, now, d + e.cost);
                }
            };
            dfs_dia(dfs_dia, p, -1, 0);
            return {mxd, {p, q}};
        }

        /// オイラーツアー
        // 構築 : O(N), (クエリにはO((query)))
        // ET_V(-/in/out), ET_E(-/in/out) でアクセス出来る（辺のindexは入力順e.idx、(N-1は開始/終了を表しており、頂点と対応関係を一致させるために便宜上追加している)）、結合率と逆元が必要（なのでmin/max系は大変）、セグ木等に適宜載せる（部分木クエリ向き？）
        void EulerTour_build() {
            int N = G.size();
            ET_Vin.resize(N);
            ET_Vout.resize(N);
            ET_Ein.resize(N);
            ET_Eout.resize(N);
            auto dfs_ET = [&](auto dfs_ET, int now, int par) -> void {
                ET_Vin[now] = ET_V.size();
                ET_V.push_back(now);
                for(auto &e : G[now]) {
                    if(e.to == par) continue;
                    ET_Ein[e.idx] = ET_E.size();
                    ET_E.push_back(e.idx);
                    dfs_ET(dfs_ET, e.to, now);
                    ET_Eout[e.idx] = ET_E.size();
                    ET_E.push_back(e.idx);
                    ET_V.push_back(now);
                }
                ET_Vout[now] = ET_V.size() - 1;
            };
            ET_Ein[N-1] = ET_E.size();
            ET_E.push_back(N-1);
            dfs_ET(dfs_ET, root, -1);
            ET_E.push_back(N-1);
            ET_Eout[N-1] = ET_E.size();
        }

        /// HL分解
        // 構築 : O(N), (クエリにはO((query)*logN))
        // HLD(-/pos/head)でアクセス出来る、結合率が必要（逆元が不要）、パス上の高々logN個の連結成分に対してそれぞれクエリを適用する（パスクエリ向き？）
        // 頂点に情報をもたせるのは素直にできるが、辺に情報をもたせる場合は、子がその辺の情報を担っているとして考える
        void HLD_build() {
            if(!builded) build();
            int N = G.size();
            HLD_pos.resize(N);
            HLD_head.resize(N);
            auto dfs_HLD = [&](auto dfs_HLD, int now, int par, int head) -> void {
                HLD_pos[now] = HLD.size();
                HLD.push_back(now);
                HLD_head[now] = head;
                if(sz[now] == 1) return;
                int mx = 0;
                int mx_idx;
                for(auto &e : G[now]) {
                    if(e.to == par) continue;
                    if(sz[e.to] > mx) {
                        mx = sz[e.to];
                        mx_idx = e.to;
                    }
                }
                dfs_HLD(dfs_HLD, mx_idx, now, head);
                for(auto &e : G[now]) {
                    if(e.to == par || e.to == mx_idx) continue;
                    dfs_HLD(dfs_HLD, e.to, now, e.to);
                }
            };
            dfs_HLD(dfs_HLD, root, -1, root);
        }

        /// HL分解後の木に対するパスクエリ処理
        // u-vパスに対するクエリを処理するのと同等の、HLD配列上の処理すべき区間一覧を返す
        // 辺の情報を取得する場合はis_edge = true に
        vector<pair<int, int> > HLD_query(int u, int v, bool is_edge = false) {
            if(HLD_head.empty()) HLD_build();
            vector<pair<int, int> > ret;
            int lca;
            if(is_edge) lca = LCA(u,v);
            while(HLD_head[u] != HLD_head[v]) {
                if(depth[HLD_head[u]] <= depth[HLD_head[v]]) {
                    ret.emplace_back(HLD_pos[HLD_head[v]], HLD_pos[v]);
                    v = par_dbl[HLD_head[v]][0];
                }
                else {
                    ret.emplace_back(HLD_pos[HLD_head[u]], HLD_pos[u]);
                    u = par_dbl[HLD_head[u]][0];                    
                }
            }
            if(is_edge) {
                int mn = min(HLD_pos[u], HLD_pos[v]);
                int mx = max(HLD_pos[u], HLD_pos[v]);
                if(mn == HLD_pos[lca]) mn++;
                else mx--;
                if(mn <= mx) ret.emplace_back(mn, mx);
            }
            else ret.emplace_back(min(HLD_pos[u], HLD_pos[v]), max(HLD_pos[u], HLD_pos[v]));
            return ret;
        }
        
};