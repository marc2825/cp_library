[![AtCoder Algorithm Rating](https://badgen.org/img/atcoder/marc2825/rating/algorithm)](https://atcoder.jp/users/marc2825)
[![AtCoder Heuristic Rating](https://badgen.org/img/atcoder/marc2825/rating/heuristic)](https://atcoder.jp/users/marc2825?contestType=heuristic)
[![Codeforces Rating](https://codeforces-readme-stats.vercel.app/api/badge?username=marc2825)](https://codeforces.com/profile/marc2825)

# cp_library

競技中に必要な機能だけ読み込む C++ ライブラリです。機能別に分離した新しいカテゴリは C++17 以降、外部ライブラリ不要で利用できます。

## カテゴリ別ライブラリ

`3.xx` ごとに独立したカテゴリを設けます。実装はカテゴリフォルダ、説明は `docs/<カテゴリ>/`、インタラクティブな図解は `visualizations/<カテゴリ>/` に配置します。

| カテゴリ | 実装 | 説明・使用例・計算量・問題例 | インタラクティブ解説 |
|---|---|---|---|
| 3.2 Union-Find・連結性管理 | [union_find/](union_find/) | [docs](docs/union_find/README.md) | [図解](visualizations/union_find/index.html) |
| 3.3 セグメント木・Fenwick Tree | [range_query/](range_query/) | [docs](docs/range_query/README.md) | [図解](visualizations/range_query/index.html) |
| 3.4 静的クエリ・順序統計・集合管理 | [static_query/](static_query/) | [docs](docs/static_query/README.md) | [図解](visualizations/static_query/index.html) |
| 3.5 平衡二分探索木・永続データ構造 | [ordered_structure/](ordered_structure/) | [docs](docs/ordered_structure/README.md) | [図解](visualizations/ordered_structure/index.html) |
| 3.6 オフラインクエリ・二次元データ構造 | [offline_query/](offline_query/) | [docs](docs/offline_query/README.md) | [図解](visualizations/offline_query/index.html) |
| 3.7 グラフ：基本探索・最短路・MST | [graph_basic/](graph_basic/) | [docs](docs/graph_basic/README.md) | [図解](visualizations/graph_basic/index.html) |
| 3.8 グラフ：連結成分・構造分解 | [graph_connectivity/](graph_connectivity/) | [docs](docs/graph_connectivity/README.md) | [図解](visualizations/graph_connectivity/index.html) |
| 3.9 グラフ：フロー・マッチング・最適化 | [graph_flow/](graph_flow/) | [docs](docs/graph_flow/README.md) | [図解](visualizations/graph_flow/index.html) |
| 3.10 グラフ：組合せ構造・数え上げ | [graph_combinatorics/](graph_combinatorics/) | [docs](docs/graph_combinatorics/README.md) | [図解](visualizations/graph_combinatorics/index.html) |
| 3.11 木アルゴリズム | [tree/](tree/) | [docs](docs/tree/README.md) | [図解](visualizations/tree/index.html) |
| 3.12 DP・最適化 | [dp_optimization/](dp_optimization/) | [docs](docs/dp_optimization/README.md) | [図解](visualizations/dp_optimization/index.html) |
| 3.13 数論：基本整数演算 | [number_theory_basic/](number_theory_basic/) | [docs](docs/number_theory_basic/README.md) | [図解](visualizations/number_theory_basic/index.html) |
| 3.14 数論：乗法的関数・高度な整数問題 | [number_theory_convolution/](number_theory_convolution/) | [docs](docs/number_theory_convolution/README.md) | [図解](visualizations/number_theory_convolution/index.html) |
| 3.15 組合せ論・数え上げ | [combinatorics/](combinatorics/) | [docs](docs/combinatorics/README.md) | [図解](visualizations/combinatorics/index.html) |
| 3.16 畳み込み・高速変換 | [convolution/](convolution/) | [docs](docs/convolution/README.md) | [図解](visualizations/convolution/index.html) |
| 3.17 多項式・形式的冪級数（FPS） | [fps/](fps/) | [docs](docs/fps/README.md) | [図解](visualizations/fps/index.html) |
| 3.18 線形漸化式・母関数 | [linear_recurrence/](linear_recurrence/) | [docs](docs/linear_recurrence/README.md) | [図解](visualizations/linear_recurrence/index.html) |
| 3.19 集合関数・Bitmask変換 | [set_function/](set_function/) | [docs](docs/set_function/README.md) | [図解](visualizations/set_function/index.html) |
| 3.20 線形代数 | [linear_algebra/](linear_algebra/) | [docs](docs/linear_algebra/README.md) | [図解](visualizations/linear_algebra/index.html) |
| 3.21 文字列アルゴリズム | [string_algorithm/](string_algorithm/) | [docs](docs/string_algorithm/README.md) | [図解](visualizations/string_algorithm/index.html) |
| 3.22 計算幾何 | [computational_geometry/](computational_geometry/) | [docs](docs/computational_geometry/README.md) | [図解](visualizations/computational_geometry/index.html) |
| 3.23 組合せ最適化・マトロイド | [combinatorial_optimization/](combinatorial_optimization/) | [docs](docs/combinatorial_optimization/README.md) | [図解](visualizations/combinatorial_optimization/index.html) |
| 3.24 ゲーム理論 | [game_theory/](game_theory/) | [docs](docs/game_theory/README.md) | [図解](visualizations/game_theory/index.html) |
| 3.25 探索・ヒューリスティック | [heuristic_search/](heuristic_search/) | [docs](docs/heuristic_search/README.md) | [図解](visualizations/heuristic_search/index.html) |

[すべてのインタラクティブ解説](visualizations/index.html) / [ドキュメントの構成方針](docs/README.md)

新しいセッションで開発を続ける場合は [引き継ぎ書](HANDOFF.md) を参照してください。

## グラフ・木の入口

```cpp
#include "graph/dijkstra.hpp"
#include "tree/doubling_lca.hpp"

cp::Graph<long long> g(3); // 既定は無向。添字・辺番号は0始まり。
g.add_edge(0, 1, 5);
g.add_edge(1, 2, 2);

auto shortest = cp::dijkstra(g, 0);
auto path = shortest.vertex_path(2); // {0,1,2}
cp::DoublingLCA<long long> tree(g, 0);
auto distance = tree.distance(0, 2); // 7
```

ライブラリのルートに include パスを通します。

```sh
g++ -std=c++17 -O2 -I /path/to/cp_library main.cpp
```

| やりたいこと | ヘッダ | API |
|---|---|---|
| グラフを保持する | [graph/graph.hpp](graph/graph.hpp) | `cp::Graph<T>` |
| 辺リストを入力する | [graph/input.hpp](graph/input.hpp) | `cp::read_graph<T>` |
| 辺の本数による最短路 | [graph/bfs.hpp](graph/bfs.hpp) | `cp::bfs` |
| 非負重みの最短路 | [graph/dijkstra.hpp](graph/dijkstra.hpp) | `cp::dijkstra` |
| 重み0/1の最短路 | [graph/zero_one_bfs.hpp](graph/zero_one_bfs.hpp) | `cp::zero_one_bfs` |
| 負辺を含む最短路 | [graph/bellman_ford.hpp](graph/bellman_ford.hpp) | `cp::bellman_ford` |
| 全頂点対の最短路 | [graph/floyd_warshall.hpp](graph/floyd_warshall.hpp) | `cp::floyd_warshall` |
| 最小全域木・森 | [graph/minimum_spanning_forest.hpp](graph/minimum_spanning_forest.hpp) | `cp::kruskal`, `cp::prim` |
| DFSを行う | [graph/dfs.hpp](graph/dfs.hpp) | `cp::depth_first_search` |
| 連結成分に分ける | [graph/connected_components.hpp](graph/connected_components.hpp) | `cp::connected_components` |
| 二部グラフを判定する | [graph/bipartite.hpp](graph/bipartite.hpp) | `cp::bipartite_coloring` |
| DAGを順序付ける | [graph/topological_sort.hpp](graph/topological_sort.hpp) | `cp::topological_sort` |
| 閉路を1つ復元する | [graph/cycle.hpp](graph/cycle.hpp) | `cp::find_cycle` |
| グリッドを探索する | [graph/grid_bfs.hpp](graph/grid_bfs.hpp) | `cp::grid_bfs` |
| 橋・関節点を求める | [graph/low_link.hpp](graph/low_link.hpp) | `cp::low_link` |
| 二重辺連結成分に分ける | [graph/two_edge_connected_components.hpp](graph/two_edge_connected_components.hpp) | `cp::two_edge_connected_components` |
| 二重頂点連結成分・Block Cut Forest | [graph/biconnected_components.hpp](graph/biconnected_components.hpp) | `cp::biconnected_components`, `cp::block_cut_forest` |
| 親・深さ・部分木を求める | [tree/rooted_tree.hpp](tree/rooted_tree.hpp) | `cp::RootedTree<T>` |
| LCA・距離・パス上の移動 | [tree/doubling_lca.hpp](tree/doubling_lca.hpp) | `cp::DoublingLCA<T>` |
| Euler Tourを構築する | [tree/euler_tour.hpp](tree/euler_tour.hpp) | `cp::EulerTour<T>` |
| 部分木・パスクエリを処理する | [tree/heavy_light_decomposition.hpp](tree/heavy_light_decomposition.hpp) | `cp::HeavyLightDecomposition<T>` |
| 木の直径を求める | [tree/diameter.hpp](tree/diameter.hpp) | `cp::tree_diameter` |
| 重心・重心分解を求める | [tree/centroid_decomposition.hpp](tree/centroid_decomposition.hpp) | `cp::tree_centroids`, `cp::CentroidDecomposition<T>` |

[3.11 木の個別説明・木DP](docs/tree/README.md) / [使い方・仕様・旧APIからの移行](docs/graph_tree.md) / [動く使用例](examples/) / [以前の検証履歴と参考資料](docs/graph_tree_history.md)

基本探索・最短路・MST は `graph_basic/all.hpp`、LowLink 等を含むグラフ全体は `graph/all.hpp`、木全体は `tree/all.hpp` を利用できます。既存の `graph/bfs.hpp` 等も互換入口として利用できます。

## ローカル検証

```sh
python3 verify/run_graph_tree.py
python3 verify/run_graph_tree.py --sanitize --stress
python3 verify/run_union_find.py
python3 verify/run_union_find.py --sanitize
node verify/visualizations/union_find_model_test.js
python3 verify/run_range_query.py
python3 verify/run_range_query.py --sanitize
node verify/visualizations/range_query_model_test.js
python3 verify/run_static_query.py
python3 verify/run_static_query.py --sanitize
node verify/visualizations/static_query_model_test.js
python3 verify/run_ordered_structure.py
python3 verify/run_ordered_structure.py --sanitize
node verify/visualizations/ordered_structure_model_test.js
python3 verify/run_offline_query.py
python3 verify/run_offline_query.py --sanitize
node verify/visualizations/offline_query_model_test.js
python3 verify/run_graph_basic.py
python3 verify/run_graph_basic.py --sanitize
node verify/visualizations/graph_basic_model_test.js
python3 verify/run_graph_connectivity.py
python3 verify/run_graph_connectivity.py --sanitize
node verify/visualizations/graph_connectivity_model_test.js

python3 verify/run_graph_flow.py
python3 verify/run_graph_flow.py --sanitize
node verify/visualizations/graph_flow_model_test.js

python3 verify/run_graph_combinatorics.py
python3 verify/run_graph_combinatorics.py --sanitize
node verify/visualizations/graph_combinatorics_model_test.js

python3 verify/run_tree.py
python3 verify/run_tree.py --sanitize
node verify/visualizations/tree_model_test.js

python3 verify/run_dp_optimization.py
python3 verify/run_dp_optimization.py --sanitize
node verify/visualizations/dp_optimization_model_test.js

python3 verify/run_number_theory_basic.py
python3 verify/run_number_theory_basic.py --sanitize
node verify/visualizations/number_theory_basic_model_test.js

python3 verify/run_number_theory_convolution.py
python3 verify/run_number_theory_convolution.py --sanitize
node verify/visualizations/number_theory_convolution_model_test.js

python3 verify/run_combinatorics.py
python3 verify/run_combinatorics.py --sanitize
node verify/visualizations/combinatorics_model_test.js

python3 verify/run_convolution.py
python3 verify/run_convolution.py --sanitize
node verify/visualizations/convolution_model_test.js

python3 verify/run_fps.py
python3 verify/run_fps.py --sanitize
node verify/visualizations/fps_model_test.js

python3 verify/run_linear_recurrence.py
python3 verify/run_linear_recurrence.py --sanitize
node verify/visualizations/linear_recurrence_model_test.js
python3 verify/run_set_function.py
python3 verify/run_set_function.py --sanitize
node verify/visualizations/set_function_model_test.js
python3 verify/run_linear_algebra.py
python3 verify/run_linear_algebra.py --sanitize
node verify/visualizations/linear_algebra_model_test.js
python3 verify/run_string_algorithm.py
python3 verify/run_string_algorithm.py --sanitize
node verify/visualizations/string_algorithm_model_test.js
python3 verify/run_computational_geometry.py
python3 verify/run_computational_geometry.py --sanitize
node verify/visualizations/computational_geometry_model_test.js
python3 verify/run_combinatorial_optimization.py
python3 verify/run_combinatorial_optimization.py --sanitize
node verify/visualizations/combinatorial_optimization_model_test.js
python3 verify/run_game_theory.py
python3 verify/run_game_theory.py --sanitize
node verify/visualizations/game_theory_model_test.js
python3 verify/run_heuristic_search.py
python3 verify/run_heuristic_search.py --sanitize
node verify/visualizations/heuristic_search_model_test.js
```

単独 include、include 順序、複数翻訳単位でのリンク、使用例、1,920個のランダムなグラフ・木、旧APIを検証します。`--stress` は20万頂点のパスを使います。`--compiler clang++` などでコンパイラを切り替えられます。

Union-Find は併合／制約履歴の全探索、1,050個のランダム系列、20万要素、ジャッジ形式の入出力を検証します。図解も独立した BFS オラクルと比較し、ブラウザ操作の検証方法は [visualizations/README.md](visualizations/README.md) に記載しています。

区間クエリは 720 個のランダム系列・154,400 操作を素朴配列と比較し、空配列、非可換演算、更新の合成順序、境界探索、旧 API、20 万要素を検証します。

3.4 は三値配列 9,841 個の全探索、240 個の FIFO 履歴・120,000 操作、20 万要素を検証します。同値の境界、最左 argmin、SWAG の非可換演算、比較・集約回数も確認します。

3.5 は標準 set/map/multiset の補助操作と、座標圧縮＋BIT の順位管理を検証します。486 頻度状態の全探索、400 履歴・152,000 更新操作、昇降順、未登録値、個数不足、LLONG_MAX、20 万座標を含みます。

3.6 は Mo・並列二分探索・静的長方形和の Sweep Line・点追加と長方形和の CDQ を検証します。1,093 配列の全区間、200 グラフ履歴、300 点更新履歴・48,000 操作、625 格子長方形、20 万要素／状態／操作・50 万 CDQ イベントを素朴解と比較します。

3.7 は 600 グラフで最短路・全ての復元経路・全成分 DFS・DAG・最小全域森を独立オラクルと比較します。20 万頂点、10 ジャッジ形式・250 ケース、既存 graph/tree・旧 API の回帰検証を含みます。

3.8 は SCC・縮約 DAG・Lowlink・橋・関節点・二部グラフ・無向連結成分を検証します。531 有向グラフ・760 無向多重グラフの全探索、600 ランダムグラフ、20 万頂点、5 ジャッジ形式・240 ケースを含みます。

3.9 は Dinic・最小カット復元・二部最大マッチングを検証します。738 容量割当と 689 二部グラフの全探索、700 組のランダム入力、流量保存・残余到達性・型の極値・20 万頂点のフロー経路と 20 万左頂点の交互路、3 ジャッジ形式・192 ケースを含みます。

3.10 は三角形・全／極大クリーク列挙、最大クリーク・独立集合、最小彩色、行列木定理を検証します。1,100 単純グラフの全探索、240 ランダムグラフ、240 多重グラフ／行列、64ビット境界・22頂点の色DP・20万頂点の疎グラフ、6 ジャッジ形式・288 ケースを含みます。

3.11 は LCA・木の直径・Euler Tour・部分木サイズ／深さ・全方位木DP・Binary Lifting・Tree DP を検証します。339個の森を全根で、300ランダム入力を全頂点対で比較し、非可換DPと方向付きの辺変換、700遷移表、uint64境界、20万頂点のパス・星型、7ジャッジ形式・336ケースを含みます。

3.12 は LIS・LCS・編集距離、0-1／完全／個数制限／Bitsetナップサック、区間・桁・部分集合DP、Largest Rectangleを実装します。9,841配列・3,969文字列組・225編集グラフ・6,575品物入力・5,058グリッドと独立に照合し、解の復元・整数極値・64ビット境界、20万要素／容量・10万桁、12ジャッジ形式・576ケースを検証します。

3.13 は GCD/LCM・拡張Euclid・mod累乗／逆元・素数篩・SPF・素因数分解・約数列挙を実装します。40,401符号付き整数対、0〜10,000の素数・SPFと約数・分解、Python任意精度20,630ケース、200万の篩、9ジャッジ形式・432ケースで、負数・整数極値・LCM超過・合成数法を検証します。

3.14 は約数／倍数ゼータ・メビウス変換、Dirichlet・GCD・LCM畳み込みを実装します。10,242列・8,586組を直接集計と比較し、合成数法・非可換な乗算・添字0・空・異なる長さ・上限での切り捨て・整数超過、100万上限の全出力と更新回数、3ジャッジ形式・288ケースを検証します。

3.15 は階乗・逆階乗、二項係数、順列／組合せ、Catalan数、重複組合せ、包除原理を実装します。階乗の逆元の条件とPascalの任意法、69,905集合族、任意精度186ブロック・6,531出力、200万階乗・1500行のPascal表・22集合の包除、6ジャッジ形式・384ケースを検証します。

3.16 はNTT、通常のmod／整数畳み込み、実数・複素数FFTを実装します。14,641配列対の全探索、直接DFT、Python任意精度858組・2,838出力、2^20点変換と10万×10万の積、浮動小数点の誤差・非有限値、5ジャッジ形式・360ケースで検証します。

3.17 は多項式加減乗・微積分・切り詰め、FPSのinv/log/exp/pow/sqrt、除算・剰余、多点評価・補間、Taylor Shift、Bostan–Moriを実装します。1,093三値級数を9精度で、Python任意精度4,352クエリ、10ジャッジ形式・480ケース、65,536項・8,192点・4,096次の漸化式で検証します。

3.18 は行列累乗による漸化式、Kitamasa、Berlekamp–Massey、Bostan–Moriを提供します。BMの9,233有限列を連立方程式の最小可解次数と比較し、600漸化式・160任意行列・Python任意精度3,504クエリ・5ジャッジ形式320ケース、uint64の極値と大きな次数を検証します。

3.19 はSubmask／Supermask列挙、和・汎用SOS DP、部分集合／上位集合ゼータ・メビウス変換を提供します。2,047基準mask・6,654三値表・Python任意精度2,428クエリ・4ジャッジ形式256ケース、64ビット境界・整数の途中超過・2^20項の全出力と更新回数、旧DP入口を検証します。

3.20 はMatrix、行列積・累乗、Gauss–Jordan、XOR基底を提供します。有限体5,915系の全解、37,449 XOR系列、Python任意精度1,361クエリ・4ジャッジ形式256ケースを独立に比較し、特解・零空間・64ビット境界・整数超過・実数のpivotを確認します。

3.21 はZ、Prefix Function・KMP、二重Rolling Hash、バイト列Trie、Suffix Array・LCP、Manacherを提供します。9,841文字列・3,937検索組・48,000 Trie操作・Python独立検証4,126クエリ・8ジャッジ形式512ケースを定義と比較し、空・重複・0/FF・20万文字・ハッシュの既知の二重衝突も確認します。

3.22 は整数と浮動小数点のAPIを分離し、Point/Vector・内外積・CCW・線分交差・直線交点・射影・距離・凸包、偏角・面積・包含・凸切断・直径・最近点対を提供します。整数の非整数座標は有理数、中間値は検査付き128ビットです。12使用例・12ジャッジ形式768ケース、Python Fractionの2,244ケース、全格子・全点対・20万点と図解12モードを検証します。

3.23 はHungarian、長方形・禁止辺・最大化のAssignment、一般グラフのBlossom最大マッチング、負費用に対応する最小費用流を実装し、二部版は3.9と共有します。全33,868グラフ、4,458費用行列、1,130全整数フロー、Python任意精度1,099ケース・5ジャッジ形式320ケースと、大規模入力・5モードの図解を検証します。

3.24 はNim・Misère Nim、mexとDAGのGrundy数、独立ゲームのXORと勝ち手を提供します。19,531山列・21,845 mex列・1,100 DAGと積ゲーム、Python独立比較1,028ケース・4ジャッジ形式256入力を検証。64ビット上位・Misèreの終局・閉路拒否、20万頂点と100万重複辺、4モードの操作できる図解も整備しています。

3.25 は固定seedの64ビット乱数、steady_clockのTimer、最良解を保持する焼きなまし、安定した同点順のBeam Search、最初／最良改善の局所探索を提供します。反復・時間予算に対応し、24,060局所探索・2,000 Beam・800 SAとPython独立検証1,550ケースを比較。64ビット全域と差分、模擬時計、20万回の改善、5モードの図解を検証します。
