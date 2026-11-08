---
title: グラフ：組合せ構造・数え上げ
documentation_of: //graph_combinatorics/all.hpp
---

# 3.10 グラフ：組合せ構造・数え上げ

共有の `cp::Graph<T>` を使い、グラフ内の集合を列挙・最適化し、彩色や全域木を求めます。C++17、0 始まり、標準ライブラリのみ。まとめて使う入口は `graph_combinatorics/all.hpp` です。

| やりたいこと | 説明・usage | 実装 | 操作できる図解 |
|---|---|---|---|
| 三頂点が全て隣接する組を列挙 | [三角形列挙](triangle_enumeration.md) | [triangle_enumeration.hpp](../../graph_combinatorics/triangle_enumeration.hpp) | [次数による向きとマーク](../../visualizations/graph_combinatorics/index.html#triangles) |
| 全クリーク／極大クリークを列挙 | [クリーク列挙](clique_enumeration.md) | [clique_enumeration.hpp](../../graph_combinatorics/clique_enumeration.hpp) | [候補の交差・P/X/pivot](../../visualizations/graph_combinatorics/index.html#cliques) |
| 頂点数が最大のクリークを一つ取得 | [最大クリーク](maximum_clique.md) | [maximum_clique.hpp](../../graph_combinatorics/maximum_clique.hpp) | [彩色上界と枝刈り](../../visualizations/graph_combinatorics/index.html#maximum) |
| 互いに非隣接な頂点を最大数選ぶ | [最大独立集合](maximum_independent_set.md) | [maximum_independent_set.hpp](../../graph_combinatorics/maximum_independent_set.hpp) | [補グラフとの対応](../../visualizations/graph_combinatorics/index.html#independent) |
| 最小色数と具体的な彩色を取得 | [彩色数](chromatic_number.md) | [chromatic_number.hpp](../../graph_combinatorics/chromatic_number.hpp) | [部分集合 DP と復元](../../visualizations/graph_combinatorics/index.html#coloring) |
| 全域木の個数／重みの積の和 | [行列木定理](matrix_tree.md) | [matrix_tree.hpp](../../graph_combinatorics/matrix_tree.hpp) | [ラプラシアン・余因子・消去](../../visualizations/graph_combinatorics/index.html#trees) |

行列木定理に使う [素数 mod の行列式](determinant_mod.md) も単独利用できます。

## 入力条件と選び方

- Graph の既定は無向です。全て無向グラフ用。三角形・クリーク・独立集合・彩色は **自己ループと多重辺を含まない単純グラフ**を渡します。重みは無視します。
- 単純性・無向・頂点番号等は assert による使用条件で、`NDEBUG` では利用者が守ります。自動で辺を消したり向きを変えたりはしません。
- 行列木定理だけは多重辺・自己ループに対応します。多重辺を別の辺として数え、ループは除外します。
- 全クリーク列挙には64頂点の制限がありません。極大クリーク列挙・最大クリーク・最大独立集合は64頂点まで。彩色は22頂点まで。表現上の上限を超えると release でも length_error です。
- クリーク・独立集合・彩色には指数時間の処理があります。上限内の全入力を短時間で解けるという保証ではありません。彩色はまず18〜20頂点程度を目安とし、密度や時間制限を考慮します。
- 列挙は bool を返すコールバック形式です。false で打ち切り、列挙関数自身も false を返します。結果の保持や積・和の集計は利用者側で行います。
- 最大／最小解は具体的な頂点・色も返します。同点の解の辞書順最小は保証しません。
- 行列木定理は素数 mod の値を返します。整数の真の個数や合成数 mod の API ではありません。単一頂点は1、空グラフは0と定義します。剰余が0でも連結な場合があります。

## 計算量

N=頂点数、M=元辺数、C=出力するクリーク数、D=頂点番号の大きい隣接先の最大数です。コールバックの追加処理は含みません。

| 処理 | 時間 | 作業領域 |
|---|---|---|
| 三角形列挙 | O(N+M√M) | O(N+M) |
| 全クリーク列挙 | O(N+M log(M+1)+(D+1)C) | O(N+M+D²) |
| 極大クリーク列挙 | 最悪指数時間。保守的上界 O(N² 2^N) | O(N²+N+M) |
| 最大クリーク／独立集合 | 最悪 O(N² 2^N+N+M)、彩色上界で枝刈り | O(N²+N+M) |
| 彩色数・彩色復元 | O(3^N+N+M) | O(2^N+N+M) |
| 行列木定理 | O(N³+M+N log p) | O(N²) |

p は固定の素数です。色 DP の主要配列は計約 `6×2^N` バイト（22頂点で約24MiB）。全列挙では、完全グラフの空でないクリークは `2^N−1` 個になるため、出力自体が巨大になります。

## 動く例・問題・検証

- [三角形・全クリーク・極大クリーク](../../examples/graph_combinatorics/enumeration.cpp)
- [最大クリークと最大独立集合](../../examples/graph_combinatorics/optimization.cpp)
- [彩色数と色列](../../examples/graph_combinatorics/coloring.cpp)
- [通常／重み付きの全域木](../../examples/graph_combinatorics/matrix_tree.cpp)

```sh
python3 verify/run_graph_combinatorics.py
python3 verify/run_graph_combinatorics.py --sanitize
node verify/visualizations/graph_combinatorics_model_test.js
node verify/visualizations/graph_combinatorics_browser_test.mjs --browser /path/to/chromium
```

9 ヘッダ（内部補助を含む）の単独 include・debug/release・逆順 include・既存カテゴリとの混在・複数翻訳単位・4 使用例を確認します。Library Checker 5形式と AtCoder 1形式のラッパーに対して、計288ケースを独立した Python オラクルで確認します。**オンライン AC 済みという意味ではありません。**

C++ では5頂点までの全1,100単純グラフと240ランダムグラフを、全頂点部分集合・彩色の直接探索と比較します。240多重グラフ／行列では、全辺部分集合による全域木数と全順列による行列式を比較します。全ての余因子、負の重み、符号付き／符号なし極値、素数2・101・998244353・2147483647、行交換・特異行列、64ビット境界、22頂点の彩色、20万頂点の疎グラフを含みます。

図解は6モード。小グラフの入力から履歴を再生成し、列挙上限・極大のみ・空集合・補グラフ・重み・素数mod・除く行列の頂点を切り替えられます。

[参考資料・設計判断](references.md) / [図解一覧](../../visualizations/index.html)

3.7〜3.11 A補完時にも、3.10のA全6項目は既存実装を保持し、`python3 verify/run_graph_combinatorics.py`で回帰検証します。[今回の検証範囲](../graph_a/README.md)。

## B項目

[全体の対応範囲・比較検証](../b_7_11/README.md) / [一次資料・設計判断](../b_7_11/references.md)。`make verify-b-7-11`でまとめて検証します。

| API | 説明・条件 | 操作できる図解 |
|---|---|---|
| [chromatic_polynomial.hpp](../../graph_combinatorics/chromatic_polynomial.hpp) | [彩色多項式](chromatic_polynomial.md) | [図解](../../visualizations/b_7_11/index.html#chromatic_polynomial) |
| [count_four_cycles.hpp](../../graph_combinatorics/count_four_cycles.hpp) | [C4の個数](count_four_cycles.md) | [図解](../../visualizations/b_7_11/index.html#count_four_cycles) |
| [maximal_clique_enumeration.hpp](../../graph_combinatorics/maximal_clique_enumeration.hpp) | [極大クリーク列挙](maximal_clique_enumeration.md) | [図解](../../visualizations/b_7_11/index.html#maximal_clique_enumeration) |
| [steiner_tree.hpp](../../graph_combinatorics/steiner_tree.hpp) | [Steiner Tree DP](steiner_tree.md) | [図解](../../visualizations/b_7_11/index.html#steiner_tree) |
| [dag_minimum_path_cover.hpp](../../graph_combinatorics/dag_minimum_path_cover.hpp) | [DAGの最小Path Cover](dag_minimum_path_cover.md) | [図解](../../visualizations/b_7_11/index.html#dag_minimum_path_cover) |
| [disjoint_paths.hpp](../../graph_combinatorics/disjoint_paths.hpp) | [辺素・頂点素パス](disjoint_paths.md) | [図解](../../visualizations/b_7_11/index.html#disjoint_paths) |
| [graph_isomorphism_filter.hpp](../../graph_combinatorics/graph_isomorphism_filter.hpp) | [グラフ同型の簡易判定](graph_isomorphism_filter.md) | [図解](../../visualizations/b_7_11/index.html#graph_isomorphism_filter) |

## G-01〜G-15の追加

[対応範囲・検証](../graph_extensions/README.md) / [出典](../graph_extensions/references.md)。

| API | 説明 | 図解 |
|---|---|---|
| [minimum_edge_cover.hpp](../../graph_combinatorics/minimum_edge_cover.hpp) | [最小辺被覆](minimum_edge_cover.md) | [操作](../../visualizations/graph_extensions/index.html#minimum_edge_cover) |
| [degree_sequence.hpp](../../graph_combinatorics/degree_sequence.hpp) | [次数列の判定と構成](degree_sequence.md) | [操作](../../visualizations/graph_extensions/index.html#degree_sequence) |
| [dag_maximum_antichain.hpp](../../graph_combinatorics/dag_maximum_antichain.hpp) | [DAGの最大反鎖](dag_maximum_antichain.md) | [操作](../../visualizations/graph_extensions/index.html#dag_maximum_antichain) |
| [vizing_edge_coloring.hpp](../../graph_combinatorics/vizing_edge_coloring.hpp) | [一般単純グラフのΔ+1辺彩色](vizing_edge_coloring.md) | [操作](../../visualizations/graph_extensions/index.html#vizing_edge_coloring) |
| [min_cost_disjoint_paths.hpp](../../graph_combinatorics/min_cost_disjoint_paths.hpp) | [最小費用の辺素・頂点素パス](min_cost_disjoint_paths.md) | [操作](../../visualizations/graph_extensions/index.html#min_cost_disjoint_paths) |

## G-16・G-20〜G-33の追加

[対応範囲・検証](../graph_advanced/README.md) / [出典](../graph_advanced/references.md)。

| API | 説明 | 図解 |
|---|---|---|
| [connected_graph_count.hpp](../../graph_combinatorics/connected_graph_count.hpp) | [ラベル付き連結グラフ数](connected_graph_count.md) | [操作](../../visualizations/graph_advanced/index.html#connected_graph_count) |
| [strongly_connected_graph_count.hpp](../../graph_combinatorics/strongly_connected_graph_count.hpp) | [強連結有向グラフ数](strongly_connected_graph_count.md) | [操作](../../visualizations/graph_advanced/index.html#strongly_connected_graph_count) |
| [euler_circuit_count.hpp](../../graph_combinatorics/euler_circuit_count.hpp) | [Euler閉路数（BEST定理）](euler_circuit_count.md) | [操作](../../visualizations/graph_advanced/index.html#euler_circuit_count) |
| [graph_isomorphism.hpp](../../graph_combinatorics/graph_isomorphism.hpp) | [一般グラフの厳密同型判定](graph_isomorphism.md) | [操作](../../visualizations/graph_advanced/index.html#graph_isomorphism) |
| [series_parallel_decomposition.hpp](../../graph_combinatorics/series_parallel_decomposition.hpp) | [二端子直並列グラフ分解](series_parallel_decomposition.md) | [操作](../../visualizations/graph_advanced/index.html#series_parallel_decomposition) |
