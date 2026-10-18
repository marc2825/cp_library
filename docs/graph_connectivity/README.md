---
title: グラフ：連結成分・構造分解
documentation_of: //graph_connectivity/all.hpp
---

# 3.8 グラフ：連結成分・構造分解

共有の `cp::Graph<T>` を使い、連結成分、強連結成分、橋・関節点、二部グラフの彩色を求めます。C++17、0 始まりの頂点・論理辺 ID、外部ライブラリ不要です。

| やりたいこと | 説明・usage | 実装 | 操作できる図解 |
|---|---|---|---|
| 有向グラフで相互に到達できる範囲をまとめる | [SCC](scc.md) | [scc.hpp](../../graph_connectivity/scc.hpp) | [2 回の DFS](../../visualizations/graph_connectivity/index.html#scc) |
| SCC を頂点とする DAG を作る | [縮約 DAG](condensation.md) | [condensation.hpp](../../graph_connectivity/condensation.hpp) | [縮約と重複辺](../../visualizations/graph_connectivity/index.html#condensation) |
| 発見順・戻れる祖先・橋・関節点をまとめて取得 | [Lowlink](low_link.md) | [low_link.hpp](../../graph_connectivity/low_link.hpp) | [ord / low の伝播](../../visualizations/graph_connectivity/index.html#lowlink) |
| 削除すると分断される辺を得る | [橋](bridges.md) | [bridges.hpp](../../graph_connectivity/bridges.hpp) | [厳密な比較と仮削除](../../visualizations/graph_connectivity/index.html#bridges) |
| 削除すると分断される頂点を得る | [関節点](articulation_points.md) | [articulation_points.hpp](../../graph_connectivity/articulation_points.hpp) | [根の特例・仮削除](../../visualizations/graph_connectivity/index.html#articulation) |
| 0/1 に彩色する・二部グラフか判定する | [二部グラフ](bipartite.md) | [bipartite.hpp](../../graph_connectivity/bipartite.hpp) | [色の伝播・衝突](../../visualizations/graph_connectivity/index.html#bipartite) |
| 無向グラフの連結成分をまとめる | [連結成分](connected_components.md) | [connected_components.hpp](../../graph_connectivity/connected_components.hpp) | [成分ごとの探索](../../visualizations/graph_connectivity/index.html#components) |

まとめて使うときは `#include "graph_connectivity/all.hpp"`。全て O(V+E) 時間です。SCC／Lowlink は再帰を使わず、20 万頂点の鎖にも対応します。重みを判定に使わないため、負のコストがあっても扱いは変わりません。

## 選び方・共通の契約

- SCC／縮約 DAG は有向グラフ用。他の関数は無向グラフ用です。[Graph の既定は無向](graph.md) なので、SCC は `GraphKind::Directed` を指定します。
- 自己ループ・多重辺・非連結・孤立点・空グラフに対応します。無向辺の両方向は同じ辺 ID を使います。
- SCC の成分番号はトポロジカル順。異なる成分への辺 u→v なら `component[u]<component[v]`。各 groups の頂点は昇順です。
- 橋は `ord[parent]<low[child]`、根以外の関節点は `ord[parent]≤low[child]`。DFS の根は子が二つ以上かで別に判定します。
- 橋と関節点が両方必要なら `low_link` を一度呼びます。個別の補助関数を両方呼ぶと、各々で計算します。
- 二部グラフは成功時に色の vector、失敗時に nullopt。空グラフは値を持つ空 vector です。
- 頂点・辺・向きの使用条件は assert。`NDEBUG` では無効になるため、利用者が入力条件を満たしてください。
- 結果の公開配列は読み取り用です。入力グラフを変更したら再計算します。

既存 `graph/low_link.hpp`・`graph/bipartite.hpp`・`graph/connected_components.hpp` は互換入口として残しています。共有 Graph は `graph/graph.hpp` のままで、3.7 や木、既存の二重辺／頂点連結成分と共用できます。`graph/all.hpp` からも今回の関数を利用できます。既存の二重連結分解は [グラフ・木ガイド](../graph_tree.md) に記載しています。

## 動く使用例

- [SCC と同成分判定](../../examples/graph_connectivity/scc.cpp)
- [縮約 DAG と元辺への対応](../../examples/graph_connectivity/condensation.cpp)
- [Lowlink・橋・関節点](../../examples/graph_connectivity/lowlink.cpp)
- [連結成分と二部グラフ](../../examples/graph_connectivity/components_and_bipartite.cpp)

## 検証

```sh
python3 verify/run_graph_connectivity.py
python3 verify/run_graph_connectivity.py --sanitize
python3 verify/run_graph_tree.py --stress
python3 verify/run_graph_tree.py --sanitize --stress
node verify/visualizations/graph_connectivity_model_test.js
node verify/visualizations/graph_connectivity_browser_test.mjs --browser /path/to/chromium
```

9 ヘッダの単独 include（debug/release）、逆順 include、既存 graph/graph_basic/tree の混在、複数翻訳単位、4 使用例、5 ジャッジ形式・240 ケースを検証します。

- 3 頂点までの全531有向グラフ（自己ループ込み）を、相互到達性と比較。
- 3 頂点までの全760無向多重グラフ（各辺0〜2本、自己ループ込み）で、各辺・各頂点を実際に削除して橋／関節点を確認。
- 600 ランダムグラフで、SCC、縮約辺の重複と元辺対応、low と DFS 部分木、全0/1彩色を独立オラクルと比較。
- 20 万頂点の鎖・大きな SCC・縮約 DAG・Lowlink・彩色。演算を定義しない重み型でも検証。
- ASan/UBSan、旧 API と木・二重連結分解の回帰検証。**オンライン AC を主張するものではありません。**

7 モードの図解は 1,559 ケース・37,101 スナップショットを独立オラクルと照合します。仮削除は図解の機能で、入力を書き換えずに原理を確かめるものです。ブラウザでは編集・再生・逆グラフ・削除・スマホ幅を確認します。

[参考資料・設計判断](references.md) / [解説一覧](../../visualizations/index.html) / [引き継ぎ書](../../HANDOFF.md)

## A項目の補完（2026-10-17）

| 機能 | 実装 | 説明・使用例 | 図解 |
|---|---|---|---|
| Functional Graph分解の共通入口 | [hpp](../../graph_connectivity/functional_graph.hpp) | [説明](functional_graph.md) | [操作](../../visualizations/graph_a/index.html#functional_graph) |
| Block-Cut Tree | [hpp](../../graph_connectivity/block_cut_tree.hpp) | [説明](block_cut_tree.md) | [操作](../../visualizations/graph_a/index.html#block_cut_tree) |
| 2-SAT | [hpp](../../graph_connectivity/two_sat.hpp) | [説明](two_sat.md) | [操作](../../visualizations/graph_a/index.html#two_sat) |
| Eulerian Trail | [hpp](../../graph_connectivity/eulerian_trail.hpp) | [説明](eulerian_trail.md) | [操作](../../visualizations/graph_a/index.html#eulerian_trail) |

今回の追加/共有入口は `python3 verify/run_graph_a.py` / `--sanitize`（`make verify-graph-a`）で検証します。[検証範囲・共有設計](../graph_a/README.md) / [一次資料と問題例](../graph_a/references.md)。オンラインACは未確認です。

## B項目（2026-10-18）

[全体の対応範囲・比較検証](../b_7_11/README.md) / [一次資料・設計判断](../b_7_11/references.md)。`make verify-b-7-11`でまとめて検証します。

| API | 説明・条件 | 操作できる図解 |
|---|---|---|
| [dominator_tree.hpp](../../graph_connectivity/dominator_tree.hpp) | [Dominator Tree](dominator_tree.md) | [図解](../../visualizations/b_7_11/index.html#dominator_tree) |
| [three_edge_connected_components.hpp](../../graph_connectivity/three_edge_connected_components.hpp) | [三重辺連結成分](three_edge_connected_components.md) | [図解](../../visualizations/b_7_11/index.html#three_edge_connected_components) |
| [cactus_decomposition.hpp](../../graph_connectivity/cactus_decomposition.hpp) | [Cactus Graph分解](cactus_decomposition.md) | [図解](../../visualizations/b_7_11/index.html#cactus_decomposition) |
| [chordal_graph.hpp](../../graph_connectivity/chordal_graph.hpp) | [Chordal Graph判定](chordal_graph.md) | [図解](../../visualizations/b_7_11/index.html#chordal_graph) |
| [tree_decomposition_width_2.hpp](../../graph_connectivity/tree_decomposition_width_2.hpp) | [Tree Decomposition（幅2）](tree_decomposition_width_2.md) | [図解](../../visualizations/b_7_11/index.html#tree_decomposition_width_2) |
| [incremental_scc.hpp](../../graph_connectivity/incremental_scc.hpp) | [Incremental SCC](incremental_scc.md) | [図解](../../visualizations/b_7_11/index.html#incremental_scc) |
| [dag_reachability.hpp](../../graph_connectivity/dag_reachability.hpp) | [DAG Reachability](dag_reachability.md) | [図解](../../visualizations/b_7_11/index.html#dag_reachability) |
