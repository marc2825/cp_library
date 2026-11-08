# 3.21〜3.25 B項目

全28掲載箇所に公開実装・説明・usage・使用例・検証・図解を揃えました。[実装計画](../implementation_plan.md) / [28モードの図解](../../visualizations/b_21_25/index.html) / [一次資料と設計判断](references.md)。これで3.1〜3.25のB全168掲載箇所を整備しています。

| カテゴリ | B掲載数 | 対応 |
|---|---:|---|
| 3.21 文字列 | 7 | Lyndon根のRuns、静的Suffix Tree、両側wildcard、Treap動的ハッシュ、最小回転、厳密複数比較、Series Linkの回文分割 |
| 3.22 幾何 | 8 | 最小包含円、円の交点、凸層、凸包Minkowski、共有直径/MST、Pick、有理数/浮動の矩形内半平面交差 |
| 3.23 最適化 | 5 | 非重み付きMatroid Intersection、Graphic/Partition、重み付き独立集合、線形Matroid Parity |
| 3.24 ゲーム | 3 | GF(2^64) Nimber、閉路のwin/lose/draw、有限short Partizan |
| 3.25 探索 | 5 | Multi-Start SA、採用率温度制御、paired-seed tuning、UCB1/epsilon-greedy、重み付き近傍管理とSA |

## 適用範囲

文字列はbyte・半開区間。静的Suffix TreeはSA/LCPから構築し、動的ハッシュには衝突があります。幾何は整数の検査付き128bit/有理数とlong doubleを分離。半平面交差は明示的な境界矩形の中の共通部分で、逐次切断O(H²)です。

重み付き独立集合は1マトロイドの貪欲法と一般グラフ64頂点の厳密分枝限定を区別します。線形パリティは小規模の決定的厳密版と確率的代数版を併記し、後者は返す集合の独立性を検査しますが最大性の失敗確率があります。Partizanは有限short gameと明示した資源上限内の操作。探索は最適性・改善・未知seedでの品質を保証しません。詳細な計算量・数値条件は個別説明を参照してください。

## 公開API一覧

| 項目 | 実装 | 説明 | 動く例 |
|---|---|---|---|
| Runs Enumeration | [string_algorithm/runs_enumeration.hpp](../../string_algorithm/runs_enumeration.hpp) | [API・条件・計算量](../string_algorithm/runs_enumeration.md) | [使用例](../../examples/string_algorithm/runs_enumeration_b.cpp) |
| Suffix Tree（静的） | [string_algorithm/suffix_tree.hpp](../../string_algorithm/suffix_tree.hpp) | [API・条件・計算量](../string_algorithm/suffix_tree.md) | [使用例](../../examples/string_algorithm/suffix_tree_b.cpp) |
| Wildcard Pattern Matching | [string_algorithm/wildcard_pattern_matching.hpp](../../string_algorithm/wildcard_pattern_matching.hpp) | [API・条件・計算量](../string_algorithm/wildcard_pattern_matching.md) | [使用例](../../examples/string_algorithm/wildcard_pattern_matching_b.cpp) |
| Dynamic Rolling Hash | [string_algorithm/dynamic_rolling_hash.hpp](../../string_algorithm/dynamic_rolling_hash.hpp) | [API・条件・計算量](../string_algorithm/dynamic_rolling_hash.md) | [使用例](../../examples/string_algorithm/dynamic_rolling_hash_b.cpp) |
| 文字列の最小巡回シフト | [string_algorithm/minimum_rotation.hpp](../../string_algorithm/minimum_rotation.hpp) | [API・条件・計算量](../string_algorithm/minimum_rotation.md) | [使用例](../../examples/string_algorithm/minimum_rotation_b.cpp) |
| 複数文字列の厳密比較 | [string_algorithm/multi_string_compare.hpp](../../string_algorithm/multi_string_compare.hpp) | [API・条件・計算量](../string_algorithm/multi_string_compare.md) | [使用例](../../examples/string_algorithm/multi_string_compare_b.cpp) |
| 回文分割DP | [string_algorithm/palindrome_partition.hpp](../../string_algorithm/palindrome_partition.hpp) | [API・条件・計算量](../string_algorithm/palindrome_partition.md) | [使用例](../../examples/string_algorithm/palindrome_partition_b.cpp) |
| Minimum Enclosing Circle | [computational_geometry/minimum_enclosing_circle.hpp](../../computational_geometry/minimum_enclosing_circle.hpp) | [API・条件・計算量](../computational_geometry/minimum_enclosing_circle.md) | [使用例](../../examples/computational_geometry/minimum_enclosing_circle_b.cpp) |
| Circle-Line / Circle-Circle Intersection | [computational_geometry/circle_intersection.hpp](../../computational_geometry/circle_intersection.hpp) | [API・条件・計算量](../computational_geometry/circle_intersection.md) | [使用例](../../examples/computational_geometry/circle_intersection_b.cpp) |
| Convex Layers | [computational_geometry/convex_layers.hpp](../../computational_geometry/convex_layers.hpp) | [API・条件・計算量](../computational_geometry/convex_layers.md) | [使用例](../../examples/computational_geometry/convex_layers_b.cpp) |
| Minkowski Sum | [computational_geometry/minkowski_sum.hpp](../../computational_geometry/minkowski_sum.hpp) | [API・条件・計算量](../computational_geometry/minkowski_sum.md) | [使用例](../../examples/computational_geometry/minkowski_sum_b.cpp) |
| Farthest Pair | [computational_geometry/farthest_pair.hpp](../../computational_geometry/farthest_pair.hpp) | [API・条件・計算量](../computational_geometry/farthest_pair.md) | [使用例](../../examples/computational_geometry/farthest_pair_b.cpp) |
| Manhattan MST（共通入口） | [computational_geometry/manhattan_mst.hpp](../../computational_geometry/manhattan_mst.hpp) | [API・条件・計算量](../computational_geometry/manhattan_mst.md) | [使用例](../../examples/computational_geometry/manhattan_mst_b.cpp) |
| 格子点数（Pickの定理） | [computational_geometry/pick.hpp](../../computational_geometry/pick.hpp) | [API・条件・計算量](../computational_geometry/pick.md) | [使用例](../../examples/computational_geometry/pick_b.cpp) |
| 半平面交差（明示的な境界矩形内） | [computational_geometry/half_plane_intersection.hpp](../../computational_geometry/half_plane_intersection.hpp) | [API・条件・計算量](../computational_geometry/half_plane_intersection.md) | [使用例](../../examples/computational_geometry/half_plane_intersection_b.cpp) |
| Matroid Intersection | [combinatorial_optimization/matroid_intersection.hpp](../../combinatorial_optimization/matroid_intersection.hpp) | [API・条件・計算量](../combinatorial_optimization/matroid_intersection.md) | [使用例](../../examples/combinatorial_optimization/matroid_intersection_b.cpp) |
| Graphic Matroid | [combinatorial_optimization/graphic_matroid.hpp](../../combinatorial_optimization/graphic_matroid.hpp) | [API・条件・計算量](../combinatorial_optimization/graphic_matroid.md) | [使用例](../../examples/combinatorial_optimization/graphic_matroid_b.cpp) |
| Partition Matroid | [combinatorial_optimization/partition_matroid.hpp](../../combinatorial_optimization/partition_matroid.hpp) | [API・条件・計算量](../combinatorial_optimization/partition_matroid.md) | [使用例](../../examples/combinatorial_optimization/partition_matroid_b.cpp) |
| Maximum Weight Independent Set | [combinatorial_optimization/maximum_weight_independent_set.hpp](../../combinatorial_optimization/maximum_weight_independent_set.hpp) | [API・条件・計算量](../combinatorial_optimization/maximum_weight_independent_set.md) | [使用例](../../examples/combinatorial_optimization/maximum_weight_independent_set_b.cpp) |
| Linear Matroid Parity | [combinatorial_optimization/linear_matroid_parity.hpp](../../combinatorial_optimization/linear_matroid_parity.hpp) | [API・条件・計算量](../combinatorial_optimization/linear_matroid_parity.md) | [使用例](../../examples/combinatorial_optimization/linear_matroid_parity_b.cpp) |
| Nimberの体演算 | [game_theory/nimber.hpp](../../game_theory/nimber.hpp) | [API・条件・計算量](../game_theory/nimber.md) | [使用例](../../examples/game_theory/nimber_b.cpp) |
| グラフゲーム（閉路・引き分け） | [game_theory/graph_game.hpp](../../game_theory/graph_game.hpp) | [API・条件・計算量](../game_theory/graph_game.md) | [使用例](../../examples/game_theory/graph_game_b.cpp) |
| Partizan Gameの基本操作 | [game_theory/partizan_game.hpp](../../game_theory/partizan_game.hpp) | [API・条件・計算量](../game_theory/partizan_game.md) | [使用例](../../examples/game_theory/partizan_game_b.cpp) |
| Multi-Start SA | [heuristic_search/multi_start_sa.hpp](../../heuristic_search/multi_start_sa.hpp) | [API・条件・計算量](../heuristic_search/multi_start_sa.md) | [使用例](../../examples/heuristic_search/multi_start_sa_b.cpp) |
| 温度制御・採用率の較正 | [heuristic_search/temperature_control.hpp](../../heuristic_search/temperature_control.hpp) | [API・条件・計算量](../heuristic_search/temperature_control.md) | [使用例](../../examples/heuristic_search/temperature_control_b.cpp) |
| パラメータチューニング | [heuristic_search/parameter_tuning.hpp](../../heuristic_search/parameter_tuning.hpp) | [API・条件・計算量](../heuristic_search/parameter_tuning.md) | [使用例](../../examples/heuristic_search/parameter_tuning_b.cpp) |
| 多腕バンディット | [heuristic_search/multi_armed_bandit.hpp](../../heuristic_search/multi_armed_bandit.hpp) | [API・条件・計算量](../heuristic_search/multi_armed_bandit.md) | [使用例](../../examples/heuristic_search/multi_armed_bandit_b.cpp) |
| 焼きなましの近傍管理 | [heuristic_search/neighborhood_manager.hpp](../../heuristic_search/neighborhood_manager.hpp) | [API・条件・計算量](../heuristic_search/neighborhood_manager.md) | [使用例](../../examples/heuristic_search/neighborhood_manager_b.cpp) |

## ローカル検証

```sh
python3 verify/run_b_21_25.py
python3 verify/run_b_21_25.py --sanitize
python3 verify/run_b_21_25.py --links-only
make verify-b-21-25
node verify/visualizations/b_21_25_model_test.js
node verify/visualizations/b_21_25_browser_test.mjs --browser /path/to/chromium
```

28公開入口の単独・重複include（debug/release）、全カテゴリall.hpp混在の複数翻訳単位、28使用例とusageを確認。CHECKはNDEBUGでも有効。--sanitizeはASan/UBSan/LeakSanitizerを使います。

- [文字列](../../verify/b_21_25/string_test.cpp)：長さ12まで全二値列のRuns・回文分割・巡回シフト、静的Suffix TreeとKMP、任意byte・両側wildcard、7000回の動的編集、NUL/FFを含む全substring対、3万/10万byteの境界。
- [幾何](../../verify/b_21_25/geometry_test.cpp)：全1/2/3点候補円、全点対和の凸包、支持直線による凸層、全格子点、全境界直線対の半平面領域、全点対の直径と密PrimによるManhattan MST。
- [マトロイド・ゲーム](../../verify/b_21_25/matroid_game_test.cpp)：全subsetの独立性・重み、法3の全subset rank、代数版の実行可能性、4局面までの全有向グラフ、全幅Nimber、左右の完全ゲーム木によるPartizan値の差とoutcome。
- [探索](../../verify/b_21_25/heuristic_test.cpp)：独立したseed・温度・SA採否の再現、直接平均採用率、paired seed、直接累積平均のUCB index、線形重み抽選、巨大重み削除・全近傍無効・観測統計。
- [4ジャッジ形式](../../verify/b_21_25/)：Runs、Wildcard、AOJ円-直線/円-円。各24入力をPythonの独立な整数/浮動計算とdebug/releaseで比較します。ヒューリスティックの架空PROBLEMは付けません。

2026-10-18の実行結果：通常・ASan/UBSan/LeakSanitizerのdebug/release、28使用例とC++参照値、4ジャッジ形式×24入力が成功。既存12系統と全図解モデルの回帰も成功しました。図解は2,478入力・20,239状態・21拒否条件を検査し、28モードのブラウザ操作・全フレームのSVG範囲・390px表示が成功。デスクトップとモバイルの画像を確認し、関連ローカル文書リンク1,413件に欠落はありません。

オンライン提出・ACやスコア改善は未確認です。[図解の範囲と簡略化](../../visualizations/b_21_25/README.md)。
