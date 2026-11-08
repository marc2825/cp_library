---
title: オフラインクエリ・二次元データ構造
documentation_of: //offline_query/all.hpp
---

# 3.6 オフラインクエリ・二次元データ構造

問い合わせを先に集め、処理順や状態の構築を共有する基本形です。C++17、`namespace cp`、0 始まり・半開区間を使います。

| 状況 | 実装 | 説明 | 図解 |
|---|---|---|---|
| 静的な配列で、区間の両端を追加・削除できる | [Mo](../../offline_query/mo.hpp) | [API・usage](mo.md) | [種類数と区間の移動](../../visualizations/offline_query/index.html#mo) |
| 更新数に対して単調な判定を一括で二分探索する | [parallel_binary_search](../../offline_query/parallel_binary_search.hpp) | [API・usage](parallel_binary_search.md) | [連結判定・mid の共有](../../visualizations/offline_query/index.html#pbs) |
| 静的な重み付き点の長方形和を求める | [Sweep Line](../../offline_query/sweep_line.hpp) | [API・usage](sweep_line.md) | [走査線と y の BIT](../../visualizations/offline_query/index.html#sweep) |
| 時系列の点追加・長方形和を後からまとめて解く | [CDQRectangleSum](../../offline_query/cdq_rectangle_sum.hpp) | [API・usage](cdq_rectangle_sum.md) | [時刻分割・寄与・巻き戻し](../../visualizations/offline_query/index.html#cdq) |

全体の入口は `#include "offline_query/all.hpp"`。点・長方形の型は [rectangle.hpp](../../offline_query/rectangle.hpp) / [型の説明](rectangle.md) に分けています。

Sweep Line は静的問題、CDQ は時刻を含む問題の具体的な基本形です。平面全体を確保せず、3.5 の [座標圧縮](../ordered_structure/coordinate_compression.md) と 3.3 の [Fenwick Tree](../range_query/fenwick_tree.md) を再利用します。任意の幾何イベント処理、長方形の面積、動的二次元木、CDQ による DP 最適化は今回の範囲には含みません。

## 最初の使い方

- [Mo による種類数](../../examples/offline_query/mo_distinct.cpp)
- [並列二分探索＋Union-Find による初回連結時刻](../../examples/offline_query/first_connected.cpp)
- [静的な長方形和](../../examples/offline_query/static_rectangle.cpp)
- [点追加を含む長方形和](../../examples/offline_query/dynamic_rectangle.cpp)

長方形は引数順を `left, down, right, up` とし、`[left,right) × [down,up)` を表します。空・逆向きの長方形の答えは 0。同一座標の点は重みを加算し、負の重みも扱います。座標境界から 1 を引きません。

## ローカル検証

```sh
python3 verify/run_offline_query.py
python3 verify/run_offline_query.py --sanitize
node verify/visualizations/offline_query_model_test.js
node verify/visualizations/offline_query_browser_test.mjs --browser /path/to/chromium
```

6 ヘッダの単独 include・逆順 include・複数翻訳単位、4 使用例、4 ジャッジ形式、debug/release の独立オラクル比較を行います。

- Mo: 三値配列 1,093 個の全探索と全区間。左右別コールバックの連続性、空区間、複数ブロック幅。
- 並列二分探索: M=0..80 のすべての成立境界、200 グラフ履歴を BFS と比較。初期成立・不成立・更新順・コールバック回数。
- Sweep/CDQ: 300 履歴・48,000 操作、625 長方形の格子全探索、同一座標、負の重み、LLONG_MIN/MAX の座標、再 solve・追記・clear。
- 20 万要素／状態／操作、CDQ の展開後 50 万イベント。独自座標型（既定構築・== 不要）と加法群型。
- ジャッジ形式は各 600 クエリを独立した Python 解と比較。**オンライン提出・AC を実施したという意味ではありません。**

図解は配列・辺列・点・問い合わせ・操作列を編集し、戻る／次へ／再生できます。入力を反映するとバッチ全体から履歴を作り直します。モデルは 647 バッチ・53,519 スナップショットを素朴解と比較します。

[参考資料と設計判断](references.md) / [すべての図解](../../visualizations/index.html) / [カテゴリ一覧](../README.md)

## 高度なアルゴリズムの追加

- [Offline Dynamic Connectivity](offline_dynamic_connectivity.md) / [操作できる図解](../../visualizations/advanced/index.html#connectivity)
- [Mo with Updates](mo_with_updates.md) / [操作できる図解](../../visualizations/advanced/index.html#mo)

追加分の検証は `python3 verify/run_advanced.py`（`--sanitize`対応）。[全追加項目と既存項目の対応表](../advanced/README.md)。

## A項目の追加（2026-10-17）

| 機能 | 実装 | 説明・usage・計算量 | 図解 |
|---|---|---|---|
| 二次元BIT（座圧版） | [hpp](../../offline_query/compressed_fenwick_tree_2d.hpp) | [説明](compressed_fenwick_tree_2d.md) | [操作](../../visualizations/a_priority/index.html#compressed_fenwick_tree_2d) |
| Range Set | [hpp](../../offline_query/range_set.hpp) | [説明](range_set.md) | [操作](../../visualizations/a_priority/index.html#range_set) |
| Offline RMQ | [hpp](../../offline_query/offline_rmq.hpp) | [説明](offline_rmq.md) | [操作](../../visualizations/a_priority/index.html#offline_rmq) |
| Rollback Mo | [hpp](../../offline_query/rollback_mo.hpp) | [説明](rollback_mo.md) | [操作](../../visualizations/a_priority/index.html#rollback_mo) |
| Mo on Tree | [hpp](../../offline_query/mo_on_tree.hpp) | [説明](mo_on_tree.md) | [操作](../../visualizations/a_priority/index.html#mo_on_tree) |

今回の追加分は `python3 verify/run_a_priority.py` / `--sanitize`（`make verify-a-priority`）で検証します。[検証範囲・共有設計](../a_priority/README.md) / [一次資料・問題例](../a_priority/references.md)。オンラインACは未確認です。

## B項目の整備（2026-10-17）

| 機能 | 実装 | 詳細 | 図解 |
|---|---|---|---|
| 長方形加算・長方形和 | [hpp](../../offline_query/rectangle_add_rectangle_sum.hpp) | [説明・usage](rectangle_add_rectangle_sum.md) | [操作](../../visualizations/b_1_6/index.html#rectangle_add_rectangle_sum) |
| 長方形加算・点取得 | [hpp](../../offline_query/rectangle_add_point_get.hpp) | [説明・usage](rectangle_add_point_get.md) | [操作](../../visualizations/b_1_6/index.html#rectangle_add_point_get) |
| Kd-Tree | [hpp](../../offline_query/kd_tree.hpp) | [説明・usage](kd_tree.md) | [操作](../../visualizations/b_1_6/index.html#kd_tree) |
| Range Tree（共通入口） | [hpp](../../offline_query/range_tree.hpp) | [説明・usage](range_tree.md) | [操作](../../visualizations/b_1_6/index.html#range_tree) |
| Hilbert Order Mo | [hpp](../../offline_query/hilbert_order_mo.hpp) | [説明・usage](hilbert_order_mo.md) | [操作](../../visualizations/b_1_6/index.html#hilbert_order_mo) |
| Static Range Count Distinct | [hpp](../../offline_query/static_range_count_distinct.hpp) | [説明・usage](static_range_count_distinct.md) | [操作](../../visualizations/b_1_6/index.html#static_range_count_distinct) |
| Dynamic Rectangle Affine Sum | [hpp](../../offline_query/dynamic_rectangle_affine_sum.hpp) | [説明・usage](dynamic_rectangle_affine_sum.md) | [操作](../../visualizations/b_1_6/index.html#dynamic_rectangle_affine_sum) |

検証: `python3 verify/run_b_1_6.py` / `--sanitize` / `make verify-b-1-6`。[対応範囲・検証](../b_1_6/README.md) / [一次資料](../b_1_6/references.md)。

## D項目の追加（2026-10-24）

- [D-09：Segment Tree over Time](segment_tree_over_time.md) / [図解](../../visualizations/data_structure_extensions/index.html#segment_tree_over_time)

[共通の検証・共有設計](../data_structure_extensions/README.md)。`make verify-data-structure-extensions`。
- [静的転倒数クエリ](./static_range_inversions.md)（D-27）
- [多次元直交範囲クエリ](./orthogonal_range_query.md)（D-33）
