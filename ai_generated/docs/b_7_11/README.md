# 3.7〜3.11 B項目

対象36掲載箇所を整備しました。[36モードの図解](../../visualizations/b_7_11/index.html) / [一次資料と設計判断](references.md)。35個の新しい公開ヘッダと既存Link-Cut Treeを入口とし、36個別説明・36実行例を用意しています。極大クリークと木のマージ履歴は既存実体へのforward入口で、アルゴリズムを複製しません。

## 対応範囲

| カテゴリ | B掲載数 | 主な内容 |
|---|---:|---|
| 3.7 graph_basic | 6 | 有向MST、Manhattan MST、最短閉路2種、MST感度、第二最小木 |
| 3.8 graph_connectivity | 7 | Dominator、三重辺連結、Cactus、Chordal、幅2木分解、増分SCC、DAG到達 |
| 3.9 graph_flow | 7 | b-flow、Stoer–Wagner、Push–Relabel、循環流、DM、二部辺彩色、重み二部マッチング |
| 3.10 graph_combinatorics | 7 | 彩色多項式、C4、既存極大clique、Steiner、DAG cover、素パス、1-WL |
| 3.11 tree | 9 | Static Top Tree、既存LCT、ETT、動的reroot、完全同型、距離分布、Contour、動的直径、既存MergeTree |

最大重みマッチングは二部版です。一般グラフの重み付きBlossomは計画Cに別記されています。Static Top Treeは各列の中点分割で高さO(log²V)、動的rerootと動的直径は固定topologyの更新です。link/cutはLCTとETTで提供します。動的部分木集約・Fully Dynamic Tree DPは計画Cのままです。

有向MSTはO(VE)、最短無向閉路は辺ごとのDijkstra、増分SCCは最悪線形走査とDAG再構築、b-flowは容量依存SSP、二部辺彩色は交互路版です。高度な高速版と同じ時間保証は主張しません。簡易同型の色一致は未確定を返し、木の同型はハッシュを使わない完全判定です。

## ローカル検証

```sh
python3 verify/run_b_7_11.py
python3 verify/run_b_7_11.py --sanitize
make verify-b-7-11
node verify/visualizations/b_7_11_model_test.js
node verify/visualizations/b_7_11_browser_test.mjs --browser /path/to/chromium
```

36公開入口の単独・重複includeをdebug/releaseで確認します。全カテゴリall.hppと逆順includeの複数翻訳単位、36使用例と文書中usage、以下の独立比較テスト、3ジャッジ形式×24入力をdebug/releaseで検証します。CHECKはNDEBUGでも有効です。--sanitizeはASan/UBSanを付けて同じ実行対象を検証します。

| 検証コード | 独立した比較方法 |
|---|---|
| [graph_test.cpp](../../verify/b_7_11/graph_test.cpp) | 全辺部分集合の全域木・有向木、完全L1グラフのKruskal、単純閉路DFS、頂点除去による支配、三重辺連結は最大流>=3、全閉路での辺所属、誘導cycle、全消去順、更新ごとの既存SCC、BFS到達。10万頂点の支配木も確認。 |
| [flow_test.cpp](../../verify/b_7_11/flow_test.cpp) | 負下限・負費用・ループを含む全流量列挙、全cut、既存Dinic、全二部マッチングの使用可能辺・最大重み、彩色端点衝突。signed64上限の二乗費用とunsigned64を超える初期preflow合計も確認。 |
| [combinatorics_test.cpp](../../verify/b_7_11/combinatorics_test.cpp) | 彩色全列挙から多項式評価、4頂点の全順序、clique全subset、Steiner全辺subset、全二部マッチングとDAG cover、全単純パスの組合せ。1-WLの未確定例も確認。 |
| [tree_test.cpp](../../verify/b_7_11/tree_test.cpp) | 更新後の全根距離和、非線形dp[v]=value[v]+Πdp[child]、全頂点対直径・BFS距離帯の両方向、非可換文字列パスと動的森、全置換同型、MergeTreeの葉集合。4万頂点のchain/starと式の高さも確認。 |
| [ジャッジ形式](../../verify/b_7_11/) | Directed MST、Minimum Cost b-flow、Frequency Table of Tree Distance。Pythonの全列挙／BFSと比較し、親木・流量上下限・balance・双対potentialも検査。 |

ジャッジ形式は入出力を合わせたローカル検証用です。特にO(VE)のDirected MSTと容量依存b-flowは、Library Checkerの大きな全制約での時間内完了を保証しません。オンライン提出・ACは未確認です。

## 公開API一覧

| 項目 | 実装 | 説明 | 実行例 |
|---|---|---|---|
| Directed MST（Chu–Liu/Edmonds） | [graph_basic/directed_mst.hpp](../../graph_basic/directed_mst.hpp) | [API・条件・計算量](../graph_basic/directed_mst.md) | [使用例](../../examples/graph_basic/directed_mst_b.cpp) |
| Manhattan MST | [graph_basic/manhattan_mst.hpp](../../graph_basic/manhattan_mst.hpp) | [API・条件・計算量](../graph_basic/manhattan_mst.md) | [使用例](../../examples/graph_basic/manhattan_mst_b.cpp) |
| 最短有向閉路 | [graph_basic/shortest_directed_cycle.hpp](../../graph_basic/shortest_directed_cycle.hpp) | [API・条件・計算量](../graph_basic/shortest_directed_cycle.md) | [使用例](../../examples/graph_basic/shortest_directed_cycle_b.cpp) |
| 最短無向閉路 | [graph_basic/shortest_undirected_cycle.hpp](../../graph_basic/shortest_undirected_cycle.hpp) | [API・条件・計算量](../graph_basic/shortest_undirected_cycle.md) | [使用例](../../examples/graph_basic/shortest_undirected_cycle_b.cpp) |
| 最小全域木の感度解析 | [graph_basic/mst_sensitivity.hpp](../../graph_basic/mst_sensitivity.hpp) | [API・条件・計算量](../graph_basic/mst_sensitivity.md) | [使用例](../../examples/graph_basic/mst_sensitivity_b.cpp) |
| 全域木の第二最小コスト | [graph_basic/second_best_mst.hpp](../../graph_basic/second_best_mst.hpp) | [API・条件・計算量](../graph_basic/second_best_mst.md) | [使用例](../../examples/graph_basic/second_best_mst_b.cpp) |
| Dominator Tree | [graph_connectivity/dominator_tree.hpp](../../graph_connectivity/dominator_tree.hpp) | [API・条件・計算量](../graph_connectivity/dominator_tree.md) | [使用例](../../examples/graph_connectivity/dominator_tree_b.cpp) |
| 三重辺連結成分 | [graph_connectivity/three_edge_connected_components.hpp](../../graph_connectivity/three_edge_connected_components.hpp) | [API・条件・計算量](../graph_connectivity/three_edge_connected_components.md) | [使用例](../../examples/graph_connectivity/three_edge_connected_components_b.cpp) |
| Cactus Graph分解 | [graph_connectivity/cactus_decomposition.hpp](../../graph_connectivity/cactus_decomposition.hpp) | [API・条件・計算量](../graph_connectivity/cactus_decomposition.md) | [使用例](../../examples/graph_connectivity/cactus_decomposition_b.cpp) |
| Chordal Graph判定 | [graph_connectivity/chordal_graph.hpp](../../graph_connectivity/chordal_graph.hpp) | [API・条件・計算量](../graph_connectivity/chordal_graph.md) | [使用例](../../examples/graph_connectivity/chordal_graph_b.cpp) |
| Tree Decomposition（幅2） | [graph_connectivity/tree_decomposition_width_2.hpp](../../graph_connectivity/tree_decomposition_width_2.hpp) | [API・条件・計算量](../graph_connectivity/tree_decomposition_width_2.md) | [使用例](../../examples/graph_connectivity/tree_decomposition_width_2_b.cpp) |
| Incremental SCC | [graph_connectivity/incremental_scc.hpp](../../graph_connectivity/incremental_scc.hpp) | [API・条件・計算量](../graph_connectivity/incremental_scc.md) | [使用例](../../examples/graph_connectivity/incremental_scc_b.cpp) |
| DAG Reachability | [graph_connectivity/dag_reachability.hpp](../../graph_connectivity/dag_reachability.hpp) | [API・条件・計算量](../graph_connectivity/dag_reachability.md) | [使用例](../../examples/graph_connectivity/dag_reachability_b.cpp) |
| Min-Cost b-Flow | [graph_flow/min_cost_b_flow.hpp](../../graph_flow/min_cost_b_flow.hpp) | [API・条件・計算量](../graph_flow/min_cost_b_flow.md) | [使用例](../../examples/graph_flow/min_cost_b_flow_b.cpp) |
| 一般グラフの最小カット（Stoer–Wagner） | [graph_flow/stoer_wagner.hpp](../../graph_flow/stoer_wagner.hpp) | [API・条件・計算量](../graph_flow/stoer_wagner.md) | [使用例](../../examples/graph_flow/stoer_wagner_b.cpp) |
| Push-Relabel | [graph_flow/push_relabel.hpp](../../graph_flow/push_relabel.hpp) | [API・条件・計算量](../graph_flow/push_relabel.md) | [使用例](../../examples/graph_flow/push_relabel_b.cpp) |
| 最小費用循環流 | [graph_flow/min_cost_circulation.hpp](../../graph_flow/min_cost_circulation.hpp) | [API・条件・計算量](../graph_flow/min_cost_circulation.md) | [使用例](../../examples/graph_flow/min_cost_circulation_b.cpp) |
| DM分解 | [graph_flow/dulmage_mendelsohn.hpp](../../graph_flow/dulmage_mendelsohn.hpp) | [API・条件・計算量](../graph_flow/dulmage_mendelsohn.md) | [使用例](../../examples/graph_flow/dulmage_mendelsohn_b.cpp) |
| 二部グラフ辺彩色 | [graph_flow/bipartite_edge_coloring.hpp](../../graph_flow/bipartite_edge_coloring.hpp) | [API・条件・計算量](../graph_flow/bipartite_edge_coloring.md) | [使用例](../../examples/graph_flow/bipartite_edge_coloring_b.cpp) |
| 最大重み二部マッチング | [graph_flow/maximum_weight_bipartite_matching.hpp](../../graph_flow/maximum_weight_bipartite_matching.hpp) | [API・条件・計算量](../graph_flow/maximum_weight_bipartite_matching.md) | [使用例](../../examples/graph_flow/maximum_weight_bipartite_matching_b.cpp) |
| 彩色多項式 | [graph_combinatorics/chromatic_polynomial.hpp](../../graph_combinatorics/chromatic_polynomial.hpp) | [API・条件・計算量](../graph_combinatorics/chromatic_polynomial.md) | [使用例](../../examples/graph_combinatorics/chromatic_polynomial_b.cpp) |
| C4の個数 | [graph_combinatorics/count_four_cycles.hpp](../../graph_combinatorics/count_four_cycles.hpp) | [API・条件・計算量](../graph_combinatorics/count_four_cycles.md) | [使用例](../../examples/graph_combinatorics/count_four_cycles_b.cpp) |
| 極大クリーク列挙 | [graph_combinatorics/maximal_clique_enumeration.hpp](../../graph_combinatorics/maximal_clique_enumeration.hpp) | [API・条件・計算量](../graph_combinatorics/maximal_clique_enumeration.md) | [使用例](../../examples/graph_combinatorics/maximal_clique_enumeration_b.cpp) |
| Steiner Tree DP | [graph_combinatorics/steiner_tree.hpp](../../graph_combinatorics/steiner_tree.hpp) | [API・条件・計算量](../graph_combinatorics/steiner_tree.md) | [使用例](../../examples/graph_combinatorics/steiner_tree_b.cpp) |
| DAGの最小Path Cover | [graph_combinatorics/dag_minimum_path_cover.hpp](../../graph_combinatorics/dag_minimum_path_cover.hpp) | [API・条件・計算量](../graph_combinatorics/dag_minimum_path_cover.md) | [使用例](../../examples/graph_combinatorics/dag_minimum_path_cover_b.cpp) |
| 辺素・頂点素パス | [graph_combinatorics/disjoint_paths.hpp](../../graph_combinatorics/disjoint_paths.hpp) | [API・条件・計算量](../graph_combinatorics/disjoint_paths.md) | [使用例](../../examples/graph_combinatorics/disjoint_paths_b.cpp) |
| グラフ同型の簡易判定 | [graph_combinatorics/graph_isomorphism_filter.hpp](../../graph_combinatorics/graph_isomorphism_filter.hpp) | [API・条件・計算量](../graph_combinatorics/graph_isomorphism_filter.md) | [使用例](../../examples/graph_combinatorics/graph_isomorphism_filter_b.cpp) |
| Static Top Tree | [tree/static_top_tree.hpp](../../tree/static_top_tree.hpp) | [API・条件・計算量](../tree/static_top_tree.md) | [使用例](../../examples/tree/static_top_tree_b.cpp) |
| Link-Cut Tree（B項目） | [tree/link_cut_tree.hpp](../../tree/link_cut_tree.hpp) | [API・条件・計算量](../tree/link_cut_tree_b.md) | [使用例](../../examples/tree/link_cut_tree_b.cpp) |
| Euler Tour Tree | [tree/euler_tour_tree.hpp](../../tree/euler_tour_tree.hpp) | [API・条件・計算量](../tree/euler_tour_tree.md) | [使用例](../../examples/tree/euler_tour_tree_b.cpp) |
| 動的木DP（Rerooting対応） | [tree/dynamic_rerooting.hpp](../../tree/dynamic_rerooting.hpp) | [API・条件・計算量](../tree/dynamic_rerooting.md) | [使用例](../../examples/tree/dynamic_rerooting_b.cpp) |
| 木の同型判定 | [tree/tree_isomorphism.hpp](../../tree/tree_isomorphism.hpp) | [API・条件・計算量](../tree/tree_isomorphism.md) | [使用例](../../examples/tree/tree_isomorphism_b.cpp) |
| 木の距離分布 | [tree/tree_distance_distribution.hpp](../../tree/tree_distance_distribution.hpp) | [API・条件・計算量](../tree/tree_distance_distribution.md) | [使用例](../../examples/tree/tree_distance_distribution_b.cpp) |
| Contour Query | [tree/contour_query.hpp](../../tree/contour_query.hpp) | [API・条件・計算量](../tree/contour_query.md) | [使用例](../../examples/tree/contour_query_b.cpp) |
| 動的木直径 | [tree/dynamic_tree_diameter.hpp](../../tree/dynamic_tree_diameter.hpp) | [API・条件・計算量](../tree/dynamic_tree_diameter.md) | [使用例](../../examples/tree/dynamic_tree_diameter_b.cpp) |
| 木上のマージ過程 | [tree/merge_tree.hpp](../../tree/merge_tree.hpp) | [API・条件・計算量](../tree/merge_tree.md) | [使用例](../../examples/tree/merge_tree_b.cpp) |
