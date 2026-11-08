# グラフ・木ライブラリ

基本探索・最短路・MST は [3.7 の個別説明](graph_basic/README.md) と [操作できる図解](../visualizations/graph_basic/index.html) に整理しました。新規コードは `graph_basic/`、既存の `graph/` の同名パスは互換入口として利用できます。共有 Graph と木・LowLink 等の API はこのガイドを参照してください。

## 共通の約束

- C++17 以降。外部依存なし。必要なヘッダだけ include できます。
- 新APIはすべて `cp` 名前空間。`using namespace std` やグローバルなマクロを定義しません。
- 頂点番号と辺番号は0始まり。新APIの区間はすべて半開区間 `[l,r)`。
- `Graph<T>` の既定は無向。生成時に `cp::GraphKind::Directed` を渡すと有向になります。
- `add_edge(u,v,cost)` は追加順の辺番号を返します。`edges()` は元の辺を1回ずつ保持します。
- 無向辺の隣接リストには両方向を入れ、同じ `idx` を付けます。自己ループ・多重辺も保持します。
- `g[v]` と `g.edges()` は読み取り専用です。アルゴリズムも入力グラフを変更しません。
- 木の前計算オブジェクトは構築時のスナップショットです。Graph を変更したら作り直します。元の Graph を破棄しても問い合わせできます。

```cpp
#include "graph/graph.hpp"

cp::Graph<long long> directed(n, cp::GraphKind::Directed);
int id = directed.add_edge(u, v, cost);
for (const auto& e : directed[u]) {
    // e.from, e.to, e.cost, e.idx
}
```

入力補助は `graph/input.hpp` に分離しました。入力が1始まりなら、既定の `index_base=1` が内部で0始まりに変換します。

```cpp
cp::GraphInputOptions options;
options.kind = cp::GraphKind::Directed;
options.weighted = true;
options.index_base = 1;
auto g = cp::read_graph<long long>(std::cin, n, m, options);
```

不正な頂点・重みなどの使用条件は `assert` で確認します。読み込み不足は `std::runtime_error`、木に閉路がある場合は `std::invalid_argument` を送出します。

## 最短路

| API | 条件 | 時間計算量 |
|---|---|---|
| `bfs(g,s)` | 重みを無視し、辺の本数を数える | O(V+E) |
| `zero_one_bfs(g,s)` | 各辺の重みが0または1 | O(V+E) |
| `dijkstra(g,s)` | 各辺の重みが非負 | O((V+E) log(V+E)) |
| `bellman_ford(g,s)` | 負辺・負閉路に対応 | O(V+VE) |
| `floyd_warshall(g)` | 負辺・負閉路に対応 | O(V³)、空間 O(V²) |

有向・無向、自己ループ、多重辺に対応します。単一始点の関数は有効な始点が必要です。空グラフの全点対最短路は空配列です。

`bfs` の距離型は `int`、その他はグラフの重み型 `T` です。結果は `ShortestPathResult<T>` で統一しています。

```cpp
#include "graph/dijkstra.hpp"

auto result = cp::dijkstra(g, s);
if (result.has_finite_distance(t)) {
    auto distance = result.dist[t];
    auto vertices = result.vertex_path(t);
    auto edge_ids = result.edge_path(t);
}
```

- `dist[v]`：最短距離。到達不能は `cp::distance_inf<T>()` (`numeric_limits<T>::max()`)。
- `parent[v]` / `parent_edge[v]`：直前の頂点・辺番号。始点と到達不能は `-1`。
- `negative_cycle[v]`：始点から到達可能な負閉路によって、距離をいくらでも小さくできる頂点。
- `reachable(v)`：到達可能か。負閉路の影響下でも到達可能なら `true`。
- `has_finite_distance(v)`：到達可能で、負閉路の影響を受けないか。
- `vertex_path(v)` / `edge_path(v)`：始点からの経路。到達不能・負閉路の影響下なら空配列。始点自身への頂点列は `{s}`。

負閉路の影響下の `dist` は最短距離として使えません。`floyd_warshall(g)[s]` も同じ形式で、始点 `s` の結果を持ちます。経路復元はパス長に比例します。

距離型は標準の整数型・浮動小数点型を使います。最大値は到達不能専用であり、有限な辺の重み・距離・計算途中の値は型の範囲内かつ最大値未満である必要があります。範囲外の加算・減算は `std::overflow_error` とし、符号付きオーバーフローを起こしません。負閉路の検出途中もこの条件が必要です。木の根からの距離、距離問い合わせ、全域森の総コストにも同じ計算方針を使います。

`depth_first_search(g,s)` も経路情報を返しますが、距離は DFS 木の深さです。最短路が必要な場合は上記の関数を選びます。

## 連結性・閉路・全域森

連結成分・SCC・Lowlink・橋・関節点・二部グラフは [3.8 の個別説明](graph_connectivity/README.md) と [操作できる図解](../visualizations/graph_connectivity/index.html) に整理しました。既存の `graph/low_link.hpp` 等も引き続き利用できます。

- `scc(g)` / `strongly_connected_components(g)`：有向グラフの強連結成分。成分番号はトポロジカル順。`condensation_graph(g)` は重複をまとめた縮約 DAG と元辺の代表 ID を返します。いずれも再帰なし、O(V+E)。

- `connected_components(g)`：無向グラフの `component[v]` と `groups` を O(V+E) で返します。空のグラフでは両方空です。
- `bipartite_coloring(g)`：無向グラフの0/1彩色。二部でなければ `nullopt`。空のグラフは成功した空配列です。O(V+E)。
- `topological_sort(g)`：有向グラフの順序。閉路があれば `nullopt`。空の DAG は成功した空配列です。O(V+E)。
- `find_cycle(g)`：最初に見つかった閉路1個、なければ `nullopt`。有向・無向、自己ループ・多重辺に対応します。`find_cycle(g,s)` は `s` から到達する範囲に限定します。O(V+E)。

閉路は `vertices` と `edges` を持ち、`edges[i]` が `vertices[i]` から次の頂点への辺に対応します。最後の辺で最初の頂点に戻ります。頂点列の末尾に始点を重ねません。

`kruskal(g)` / `prim(g)` は無向グラフを受け取り、負辺・自己ループ・多重辺・非連結に対応します。結果は `cost`、採用した `edges`、`component_count`、`connected()`。非連結なら各成分の最小全域木を合わせた森になります。空グラフのコスト・成分数は0です。

`kruskal` と `prim` は O(V+E log(E+1))。元の辺列を並べ替えず、辺番号も変えません。

## LowLink と分解

いずれも無向グラフ用で、非連結・自己ループ・多重辺に対応します。再帰を使わず O(V+E) で処理します。

| API | 結果 |
|---|---|
| `low_link(g)` | `ord`, `low`, `parent`, `parent_edge`, `subtree_size`, `articulation`, `bridge_edges` |
| `two_edge_connected_components(g)` | `component`, `groups`, 縮約後の `forest`, `original_edge` |
| `biconnected_components(g)` | 各二重頂点連結成分の頂点集合 |
| `block_cut_forest(g)` | `blocks` と、元の頂点・ブロックを結ぶ `forest` |

`articulation` は頂点番号順、`bridge_edges` は辺番号順です。LowLink の `subtree_size` は DFS 木における部分木サイズです。

二重辺連結成分の `forest` は連結入力なら木、非連結入力なら森です。縮約した辺にも新しく0始まりの辺番号を付け、`original_edge[new_id]` から元の辺番号を取得できます。

二重頂点連結成分では、橋も2頂点のブロック、孤立点は1頂点のブロックとします。自己ループは頂点連結性に影響しないためブロック生成から除外します。自己ループだけを持つ頂点は1頂点のブロックになります。関節点は複数のブロックに含まれます。

Block Cut Forest は `[0,n)` を元の頂点、`[n,n+blocks.size())` をブロックとして、所属関係を辺にします。元の頂点のうち次数2以上が関節点です。孤立点にも1頂点のブロックを接続します。

## 木の前計算を選ぶ

LCA・Euler Tour・直径・部分木／深さと、新しい Binary Lifting・Tree DP・全方位木DPは [3.11 の個別説明](tree/README.md) と [操作できる図解](../visualizations/tree/index.html) に整理しました。既存の木API・旧互換入口も保持します。

グラフを持つ巨大な `Tree` は新APIにはありません。同じ `cp::Graph<T>` から、必要な問い合わせ用オブジェクトを構築します。

| クラス・関数 | 前計算 | 主な用途 |
|---|---|---|
| `RootedTree<T>(g,root)` | O(N) | 親、深さ、部分木サイズ、根からの距離、DFS順 |
| `DoublingLCA<T>(g,root)` | O(N log N) | LCA、祖先、距離、パス上の移動 |
| `EulerTour<T>(g,root)` | O(N) | 頂点・辺の往復列 |
| `HeavyLightDecomposition<T>(g,root)` | O(N) | 部分木区間とパスの区間列 |
| `tree_diameter(g)` | O(N) | 非負重みの連結な木の直径と復元 |
| `tree_centroids(g)` | O(N) | 各成分の重心 |
| `CentroidDecomposition<T>(g)` | O(N log N) | 重心分解木 |

無向の木・森を入力します。自己ループ・多重辺も閉路なので受け付けません。直径以外は森にも対応し、`root` の成分を先に、他の成分は頂点番号が最小の頂点を根として処理します。空の森も構築できます。

`RootedTree` の主な配列は `parent`、`parent_edge`、`depth`、`subtree_size`、`distance_from_root`、`component`、`preorder`、`postorder`、`tin`、`tout` です。各成分の根の `parent` は `-1`。部分木は `preorder` の `[tin[v],tout[v])` になり、`subtree_interval(v)` でも取得できます。

### LCA・距離・パス

```cpp
#include "tree/doubling_lca.hpp"
cp::DoublingLCA<long long> lca(g, 0);
int w = lca.lca(u, v);
long long d = lca.distance(u, v);
int next = lca.jump(u, v, 1);       // u -> v の1辺先
int ancestor = lca.kth_ancestor(v, k);
auto vertices = lca.vertex_path(u, v);
```

`lca`、`distance`、`edge_distance`、`jump`、`kth_ancestor`、`is_on_path` は O(log N)。`vertex_path` はパス長に比例します。

- 別成分の `lca` / `jump` は `-1`、`vertex_path` は空配列です。
- `distance` / `edge_distance` は同一成分の頂点を指定します。
- `kth_ancestor` は祖先がなければ `-1`。根の0個上は根自身、1個上は `-1`。
- `jump(u,v,k)` はパスの0始まりの k 番目。パス長を超えたら `-1`。
- `distance` は重み付き距離、`edge_distance` は辺の本数です。
- 直径以外は負辺・重み0も扱えます。`is_on_path` は辺の本数で判定するので、重み0や負辺でも正しく判定します。

すでに `RootedTree` がある場合は `DoublingLCA<T>(std::move(rooted))` で前計算を引き継げます。前計算配列は `lca.tree` と `lca.up[k][v]` にあります。

### HLD と区間の向き

```cpp
#include "tree/heavy_light_decomposition.hpp"
cp::HeavyLightDecomposition<long long> hld(g, 0);
auto [l, r] = hld.subtree_interval(v); // [l,r)
for (auto segment : hld.path_segments(u, v)) {
    // [segment.l,segment.r) を処理する。
    // reversed=true なら r-1 から l へ、false なら l から r-1 へ。
}
```

`pos[v]` は分解後の位置、`vertex_at[i]` はその位置の頂点、`head[v]` は連鎖の先頭です。`tree` に親・深さなどの共通情報を持ちます。

`path_segments` は u から v への順に O(log N) 個の区間を返します。和・最大値などでは向きを無視できます。文字列結合・行列積など順序が重要な場合は区間の向きも処理してください。`lca(u,v)` も O(log N) で取得できます。

辺の値を子の位置に載せる場合は `path_segments(u,v,true)` を使います。LCA の位置が除かれ、同じ頂点同士の辺区間は空配列です。パスクエリは同一成分に限定します。

### Euler Tour・直径・重心

- Euler Tour：`walk` は往復の頂点列、`first/last` は各頂点の最初/最後の位置。`edge_walk` は往復の辺番号列、`edge_in/edge_out` は往路/復路の位置です。部分木クエリには `tour.tree.preorder` と `subtree_interval` を使うと簡単です。森では成分ごとの往復列を連結します。
- 直径：`length`、`endpoints`、`vertices`、`edges` を返します。非負重みの連結な木用。空の木は長さ0、端点 `(-1,-1)`、空のパスです。非連結入力は `invalid_argument`。
- 重心：各成分1個または2個を頂点番号順に返します。辺の重みは重心の選択に影響しません。
- 重心分解：`parent`、`level`、`component_size`、`roots` を持ちます。根の親は `-1`。`component_size[v]` は v を重心として選んだ時の成分の頂点数です。元のグラフを変更しません。

## 旧APIからの移行

`graph_theory/graph_template.hpp` と `graph_theory/Tree/tree_template.hpp` は互換ヘッダとして残しています。同時 include、旧単独 Dijkstra、Union-Find の同時利用にも対応します。旧入口は一括読み込みになり、グラフを新API用に変換する O(V+E) の処理を含みます。新規コードは機能別ヘッダを使うと必要な処理だけを読み込めます。

| 旧API | 新API |
|---|---|
| `Graph<T> g(n); g.add_edge(u,v,w)` | `cp::Graph<T> g(n); g.add_edge(u,v,w)` |
| `g.add_directed_edge(u,v,w)` | 生成時に `GraphKind::Directed` を指定し `add_edge` |
| `g.dijkstra(s)` / `g.bfs(s)` | `cp::dijkstra(g,s)` / `cp::bfs(g,s)` |
| `result.prev` / `result.Eid` | `result.parent` / `result.parent_edge` |
| `g.Articulation()` / `g.Bridge()` | `cp::low_link(g).articulation` / `.bridge_edges` |
| `g.TwoEdgeConnectedComponents()` | `cp::two_edge_connected_components(g)` |
| `g.BlockCutTree()` | `cp::block_cut_forest(g)` |
| `Tree<T> t(n); t.LCA(u,v)` | `cp::DoublingLCA<T> t(g,root); t.lca(u,v)` |
| `t.dist(u,v)` / `t.nxt(u,v)` | `lca.distance(u,v)` / `lca.jump(u,v,1)` |
| `t.HLD_query(u,v)` | `hld.path_segments(u,v)` |
| `unionfind` | `cp::DisjointSetUnion` |

互換層でも修正・整理した仕様があります。

- `Graph::E` は無向辺も1回ずつ保存します。両方向の走査は `G` または `g[v]` を使います。
- 旧 `Edge<T>(to,cost)` は隣接リスト形式として扱います。始点・終点の形式では `Edge<T>(from,to,cost)` と重みを明示します。
- 旧単独 `dijkstra(g,s)` は `Graph<T>` の重み型を保持し、従来どおり到達不能を `-1` で返します。`g[u].emplace_back(v,cost)` の構築も使えます。ただし `Graph` はもとの vector の型エイリアスから互換クラスへ変わったため、vector 固有の全操作には対応しません。
- 旧 Graph メソッドの到達不能は最大値、Bellman–Ford の負閉路の影響下は最大値の負値です。新APIは負閉路フラグを使います。
- 旧 `HLD_query` **だけ**は閉区間 `[l,r]` を維持します。新APIへ移行したら `r` を含めないようにします。
- 重みを直接 `Tree::add_edge` で渡した場合も距離計算へ反映します。根の変更・辺の追加後は関連する前計算を作り直します。`EulerTour_build` / `HLD_build` の再呼び出しは列を重ねません。
- `Tree::size()` は頂点数、`size(v)` は部分木サイズ。`nxt(s,s)` は `-1`。根を削除する `resize` の後は `rebuild(valid_root)` を呼びます。
- 旧 Euler Tour の根用ダミー辺番号は `ET_root_edge` で取得できます。通常は N-1 ですが、任意の辺番号と衝突する場合は大きい番号へずらします。
- 旧重心分解は全体に対する呼び出しを残しています。部分領域に対する `centroid_decomposition(root,parent)` の手動再帰は、新APIの重心分解木をたどる形へ移行します。
- 旧公開配列 `G` / `E` を直接変更すると両者の同期を保てません。単独 Dijkstra 用の隣接リスト構築以外では、追加・削除を `add_edge` / `add_directed_edge` / `resize` で行ってください。
- 旧 `Grid_BFS` の名前は残します。`include` マクロや `dx1/dy1` のグローバル定義は廃止しました。

以前の提出リンクと参考資料は [履歴](graph_tree_history.md) に保存しています。履歴のコメントは現行実装の仕様ではありません。`user_snippet.json` の従来の貼り付け用実装はこの移行では変更していません。新APIを使う場合はヘッダを読み込んでください。

## 動作確認

```sh
python3 verify/run_graph_tree.py
python3 verify/run_graph_tree.py --sanitize --stress
```

検証はローカルで完結します。最短路と経路復元、負閉路の影響、全域森の部分集合全探索、頂点・辺の除去による橋/関節点判定、二重連結成分の全探索、木のパス列との LCA/HLD 比較、直径・重心・重心分解の性質を確認します。外部ジャッジへの再提出は行っていません。

[最短路](../examples/graph_shortest_path.cpp)、[LCA・距離](../examples/tree_queries.cpp)、[順序を持つHLD](../examples/tree_hld.cpp) の使用例もコンパイル・実行します。

## F-15のグリッド共通化（2026-10-19）

[グリッドユーティリティ](basic_utility/grid.md)に4／8近傍・境界・回転・転置を追加しました。既存grid_bfsは方向と境界判定を共有し、矩形・非空盤面の始点・距離型に収まるセル数を例外で検査します。探索と有効入力の結果は従来どおりです。`make verify-foundation-tools`で全3×3盤面を比較します。
