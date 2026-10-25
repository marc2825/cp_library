---
title: グラフ：基本探索・最短路・MST
documentation_of: //graph_basic/all.hpp
---

# 3.7 グラフ：基本探索・最短路・MST

必要なアルゴリズムだけ include する C++17 ライブラリです。`namespace cp`、頂点・論理辺番号は 0 始まり。共通のグラフ表現と、頂点列・辺番号列を復元できる結果型を使います。

| やりたいこと | 説明・usage | ヘッダ | 図解 |
|---|---|---|---|
| 静的隣接リスト・辺ID・逆向きarc | [CSR Graph](csr_graph.md) | [csr_graph.hpp](../../graph/csr_graph.hpp) | [辺の配置](../../visualizations/foundation_structures/index.html#csr_graph) |
| 最小の辺数 | [BFS](bfs.md) | [bfs.hpp](../../graph_basic/bfs.hpp) | [FIFO](../../visualizations/graph_basic/index.html#bfs) |
| 到達探索・行きがけ順・帰りがけ順 | [DFS](dfs.md) | [dfs.hpp](../../graph_basic/dfs.hpp) | [スタック](../../visualizations/graph_basic/index.html#dfs) |
| 重み 0/1 の最短路 | [0-1 BFS](zero_one_bfs.md) | [zero_one_bfs.hpp](../../graph_basic/zero_one_bfs.hpp) | [deque](../../visualizations/graph_basic/index.html#zero-one) |
| 非負重みの最短路 | [Dijkstra](dijkstra.md) | [dijkstra.hpp](../../graph_basic/dijkstra.hpp) | [優先候補](../../visualizations/graph_basic/index.html#dijkstra) |
| 負辺・負閉路の影響 | [Bellman–Ford](bellman_ford.md) | [bellman_ford.hpp](../../graph_basic/bellman_ford.hpp) | [緩和・伝播](../../visualizations/graph_basic/index.html#bellman) |
| 全点対最短路 | [Warshall–Floyd](floyd_warshall.md) | [floyd_warshall.hpp](../../graph_basic/floyd_warshall.hpp) | [距離行列](../../visualizations/graph_basic/index.html#floyd) |
| 依存順に並べる | [トポロジカルソート](topological_sort.md) | [topological_sort.hpp](../../graph_basic/topological_sort.hpp) | [入次数](../../visualizations/graph_basic/index.html#topological) |
| 閉路を一つ取り出す | [閉路検出](cycle.md) | [cycle.hpp](../../graph_basic/cycle.hpp) | [3 色 DFS](../../visualizations/graph_basic/index.html#cycle) |
| 最小全域森を辺の重み順で作る | [Kruskal](kruskal.md) | [kruskal.hpp](../../graph_basic/kruskal.hpp) | [成分の接続](../../visualizations/graph_basic/index.html#kruskal) |
| 最小全域森を木から広げる | [Prim](prim.md) | [prim.hpp](../../graph_basic/prim.hpp) | [辺の候補](../../visualizations/graph_basic/index.html#prim) |
| 経路を頂点列・辺番号列で得る | [経路復元](shortest_path.md) | [shortest_path.hpp](../../graph_basic/shortest_path.hpp) | [親を逆に辿る](../../visualizations/graph_basic/index.html#path) |

まとめて使う場合は `#include "graph_basic/all.hpp"`。基本の型は [Graph](graph.md)・[入力補助](input.md)・[最小全域森の結果型](spanning_forest.md) を参照してください。Kruskal と Prim の共通入口は [minimum_spanning_forest.hpp](minimum_spanning_forest.md) です。

F-09で追加したCSRGraphは`Graph`との相互変換を提供します。既存探索へ渡す場合は`to_graph()`でコピーします。[追加分の検証](../foundation_structures/README.md)は`make verify-foundation-structures`。

## 共通の約束

- `Graph` の既定は**無向**。有向なら `cp::GraphKind::Directed` を明示する。
- 無向辺の両方向は同じ論理辺 ID。`edges()` は元の辺を一度だけ保持する。
- 自己ループ・多重辺・非連結を扱う。最短路の単一始点版には有効な始点が必要。
- BFS は重みを無視、DFS の dist は DFS 木の深さ。重みの最短距離とは区別する。
- 到達不能は `distance_inf<T>()`。負閉路の影響は `negative_cycle[v]`。影響下の数値 dist を使わない。
- MST 関数は非連結で**最小全域森**を返す。`component_count` と `connected()` を確認する。
- 結果は入力グラフから独立した値。グラフ変更後の問い合わせには、アルゴリズムを実行し直す。

重み・距離には標準の整数／浮動小数点型を使います。最大値は到達不能専用で、有限の辺重み・距離・全ての加算途中の値は最大値未満かつ型の範囲内である必要があります。範囲外の加算は `std::overflow_error`。負閉路の検出途中にも条件が必要で、値を下限へ丸める実装ではありません。NaN・無限大を入力しないでください。浮動小数は丸め誤差の影響を受け、厳密な整数最短路と同じ保証にはなりません。

使用条件の assert は `NDEBUG` で無効になります。競技入力で条件を満たすことを利用者が確認してください。計算量は型の比較・演算を O(1) とし、E は論理辺数です。

## 既存コードからの移行

アルゴリズムの本体は `graph_basic/` に整理しました。`graph/bfs.hpp` 等の既存パスは新しい本体へ転送し、関数名・結果型は維持しています。共有の `Graph` と入力補助は `graph/graph.hpp`・`graph/input.hpp` に保持し、`graph_basic/` にも入口を用意しました。`tree/`、LowLink 等、`graph_theory/` の旧 API も引き続き利用できます。

今回の追加は `dfs_forest` と、Kruskal／Prim の個別ヘッダです。Bellman–Ford は元の辺列を直接走査し、孤立点の多いグラフでも各周に V 個の頂点を走査しないようにしました。同距離で選ぶ経路の順序は API の保証に含めません。

[既存のグラフ・木ガイド](../graph_tree.md) / [木の使用例](../../examples/tree_queries.cpp)

## 動く使用例

- [探索と全成分 DFS](../../examples/graph_basic/traversal.cpp)
- [最短路と経路復元](../../examples/graph_basic/shortest_paths.cpp)
- [トポロジカル順序・閉路](../../examples/graph_basic/order_and_cycle.cpp)
- [Kruskal / Prim と非連結な森](../../examples/graph_basic/spanning_forest.cpp)

## 検証

```sh
python3 verify/run_graph_basic.py
python3 verify/run_graph_basic.py --sanitize
python3 verify/run_graph_tree.py --stress
python3 verify/run_graph_tree.py --sanitize --stress
node verify/visualizations/graph_basic_model_test.js
node verify/visualizations/graph_basic_browser_test.mjs --browser /path/to/chromium
```

16 ヘッダを debug/release で単独 include、既存 graph/tree と混在 include、逆順 include、複数翻訳単位で検証します。4 使用例と、10 ジャッジ形式・250 ケースを独立した Python 解で確認します。

600 グラフでは辺数を制限した歩道 DP、負閉路からの伝播、復元経路の全辺と重み、再帰 DFS、順列による DAG 判定、全辺部分集合による最小全域森を照合します。20 万頂点の鎖、20 万孤立点を含む 1 辺の負閉路、多重辺・自己ループ・負辺・空グラフも含みます。debug/release 両方、ASan/UBSan で検査します。

既存グラフ・木の検証でも 1,920 ランダムケース、旧 API、LowLink、LCA/HLD 等、20 万頂点の鎖を確認します。図解のモデルも独立オラクルと比較し、ブラウザでは全 11 モードと入力変更・再生・スマホ幅を検証します。**オンライン提出・AC を実施したという意味ではありません。**

[参考資料・設計判断](references.md) / [解説一覧](../../visualizations/index.html) / [引き継ぎ書](../../HANDOFF.md)

## A項目の補完（2026-10-17）

| 機能 | 実装 | 説明・使用例 | 図解 |
|---|---|---|---|
| Radix Heap Dijkstra | [hpp](../../graph_basic/radix_heap_dijkstra.hpp) | [説明](radix_heap_dijkstra.md) | [操作](../../visualizations/graph_a/index.html#radix_heap_dijkstra) |
| Dial's Algorithm | [hpp](../../graph_basic/dial_shortest_path.hpp) | [説明](dial_shortest_path.md) | [操作](../../visualizations/graph_a/index.html#dial_shortest_path) |
| DAG最短路 | [hpp](../../graph_basic/dag_shortest_path.hpp) | [説明](dag_shortest_path.md) | [操作](../../visualizations/graph_a/index.html#dag_shortest_path) |
| Johnson's Algorithm | [hpp](../../graph_basic/johnson.hpp) | [説明](johnson.md) | [操作](../../visualizations/graph_a/index.html#johnson) |
| 重み付き経路の結果型 | [hpp](../../graph_basic/weighted_path.hpp) | [説明](weighted_path.md) | [操作](../../visualizations/graph_a/index.html#k_shortest_paths) |
| K-th Shortest Path | [hpp](../../graph_basic/k_shortest_paths.hpp) | [説明](k_shortest_paths.md) | [操作](../../visualizations/graph_a/index.html#k_shortest_paths) |
| K-th Shortest Walk | [hpp](../../graph_basic/k_shortest_walks.hpp) | [説明](k_shortest_walks.md) | [操作](../../visualizations/graph_a/index.html#k_shortest_walks) |
| 補グラフBFS | [hpp](../../graph_basic/complement_bfs.hpp) | [説明](complement_bfs.md) | [操作](../../visualizations/graph_a/index.html#complement_bfs) |
| Functional Graph | [hpp](../../graph_basic/functional_graph.hpp) | [説明](functional_graph.md) | [操作](../../visualizations/graph_a/index.html#functional_graph) |

今回の追加/共有入口は `python3 verify/run_graph_a.py` / `--sanitize`（`make verify-graph-a`）で検証します。[検証範囲・共有設計](../graph_a/README.md) / [一次資料と問題例](../graph_a/references.md)。オンラインACは未確認です。

## B項目（2026-10-18）

[全体の対応範囲・比較検証](../b_7_11/README.md) / [一次資料・設計判断](../b_7_11/references.md)。`make verify-b-7-11`でまとめて検証します。

| API | 説明・条件 | 操作できる図解 |
|---|---|---|
| [directed_mst.hpp](../../graph_basic/directed_mst.hpp) | [Directed MST（Chu–Liu/Edmonds）](directed_mst.md) | [図解](../../visualizations/b_7_11/index.html#directed_mst) |
| [manhattan_mst.hpp](../../graph_basic/manhattan_mst.hpp) | [Manhattan MST](manhattan_mst.md) | [図解](../../visualizations/b_7_11/index.html#manhattan_mst) |
| [shortest_directed_cycle.hpp](../../graph_basic/shortest_directed_cycle.hpp) | [最短有向閉路](shortest_directed_cycle.md) | [図解](../../visualizations/b_7_11/index.html#shortest_directed_cycle) |
| [shortest_undirected_cycle.hpp](../../graph_basic/shortest_undirected_cycle.hpp) | [最短無向閉路](shortest_undirected_cycle.md) | [図解](../../visualizations/b_7_11/index.html#shortest_undirected_cycle) |
| [mst_sensitivity.hpp](../../graph_basic/mst_sensitivity.hpp) | [最小全域木の感度解析](mst_sensitivity.md) | [図解](../../visualizations/b_7_11/index.html#mst_sensitivity) |
| [second_best_mst.hpp](../../graph_basic/second_best_mst.hpp) | [全域木の第二最小コスト](second_best_mst.md) | [図解](../../visualizations/b_7_11/index.html#second_best_mst) |

## G-01〜G-15の追加（2026-10-25）

[対応範囲・検証](../graph_extensions/README.md) / [出典](../graph_extensions/references.md)。

| API | 説明 | 図解 |
|---|---|---|
| [segment_tree_graph.hpp](../../graph_basic/segment_tree_graph.hpp) | [区間辺グラフ](segment_tree_graph.md) | [操作](../../visualizations/graph_extensions/index.html#segment_tree_graph) |
| [minimum_mean_cycle.hpp](../../graph_basic/minimum_mean_cycle.hpp) | [最小平均閉路 / Karp](minimum_mean_cycle.md) | [操作](../../visualizations/graph_extensions/index.html#minimum_mean_cycle) |
| [boruvka.hpp](../../graph_basic/boruvka.hpp) | [Borůvka・暗黙MST・XOR-MST](boruvka.md) | [操作](../../visualizations/graph_extensions/index.html#boruvka) |
| [incremental_msf.hpp](../../graph_basic/incremental_msf.hpp) | [Incremental Minimum Spanning Forest](incremental_msf.md) | [操作](../../visualizations/graph_extensions/index.html#incremental_msf) |
| [difference_constraints.hpp](../../graph_basic/difference_constraints.hpp) | [差分制約](difference_constraints.md) | [操作](../../visualizations/graph_extensions/index.html#difference_constraints) |
| [negative_cycle.hpp](../../graph_basic/negative_cycle.hpp) | [負閉路の抽出](negative_cycle.md) | [操作](../../visualizations/graph_extensions/index.html#negative_cycle) |
