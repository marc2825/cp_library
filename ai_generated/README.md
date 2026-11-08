# cp_library

競技中に必要な機能だけ読み込む C++ ライブラリです。機能別に分離した新しいカテゴリは C++17 以降で利用できます。多倍長整数・有理数、およびそれらを含む `basic_utility/all.hpp` にはBoostヘッダが必要です。

**[競技中の使い方：include・コピペ・提出用1ファイル・VS Code補完](docs/usage.md)**

## 使い方（TL;DR）

`main.cpp` に必要なヘッダを読み込みます。例：`#include "union_find/disjoint_set_union.hpp"`。

```sh
CP_LIBRARY=/path/to/cp_library/ai_generated  # 自分の配置先へ変更
g++ -std=c++17 -O2 -I "$CP_LIBRARY" main.cpp -o main
python3 "$CP_LIBRARY/tools/cplib.py" bundle main.cpp -o submission.cpp --check
```

ローカルでは `-I` で依存ヘッダも読み込めます。提出時は、内部の依存を自動展開した **`submission.cpp`** を提出します。
生成ツールにはPython 3.10以上を使います。ヘッダの検索は `python3 "$CP_LIBRARY/tools/cplib.py" find dsu`、コピペ用の依存込み抽出は `python3 "$CP_LIBRARY/tools/cplib.py" snippet dsu --check` でできます。

## カテゴリ別ライブラリ

`3.xx` ごとに独立したカテゴリを設けます。実装はカテゴリフォルダ、説明は `docs/<カテゴリ>/`、インタラクティブな図解は `visualizations/<カテゴリ>/` に配置します。

| カテゴリ | 実装 | 説明・使用例・計算量・問題例 | インタラクティブ解説 |
|---|---|---|---|
| 追加 O/B 最適化・超現実数 | [combinatorial_optimization/](combinatorial_optimization/) / [game_theory/](game_theory/) | [docs](docs/optimization_extensions/README.md) | [図解](visualizations/optimization_extensions/index.html) |
| 追加 Y 数値解析 | [numerical_analysis/](numerical_analysis/) | [docs](docs/numerical_analysis/README.md) | [図解](visualizations/numerical_analysis/index.html) |
| 3.1 基本ユーティリティ・探索 | [basic_utility/](basic_utility/) | [docs](docs/basic_utility/README.md) | [図解](visualizations/a_priority/index.html#cyclic_sequence) |
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

[すべてのインタラクティブ解説](visualizations/index.html) / [ドキュメント索引](docs/README.md) / [グラフ・木のガイド](docs/graph_tree.md) / [検証方法](docs/development/testing.md)
