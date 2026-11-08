# データ構造の追加 D-02〜D-10

2026-10-24 の「D2〜19」依頼に対応する9項目です。統合一覧には D-11〜D-19 の行がなく、次に掲載されるD-20〜D-35も[続くバッチ](../data_structure_advanced/README.md)で整備済みです。次はG-01です。

| ID | 説明・API・計算量 | 実装 | 使用例 | 図解 |
|---|---|---|---|---|
| D-02 | [Double-Ended Priority Queue](../ordered_structure/double_ended_priority_queue.md) | [hpp](../../ordered_structure/double_ended_priority_queue.hpp) | [例](../../examples/ordered_structure/double_ended_priority_queue.cpp) | [操作](../../visualizations/data_structure_extensions/index.html#double_ended_priority_queue) |
| D-03 | [Static Range MEX](../static_query/static_range_mex.md) | [hpp](../../static_query/static_range_mex.hpp) | [例](../../examples/static_query/static_range_mex.cpp) | [操作](../../visualizations/data_structure_extensions/index.html#static_range_mex) |
| D-04 | [Range Majority](../static_query/range_majority.md) | [hpp](../../static_query/range_majority.hpp) | [例](../../examples/static_query/range_majority.cpp) | [操作](../../visualizations/data_structure_extensions/index.html#range_majority) |
| D-05 | [Point Set Range Frequency](../range_query/point_set_range_frequency.md) | [hpp](../../range_query/point_set_range_frequency.hpp) | [例](../../examples/range_query/point_set_range_frequency.cpp) | [操作](../../visualizations/data_structure_extensions/index.html#point_set_range_frequency) |
| D-06 | [Persistent Leftist Heap](../ordered_structure/persistent_leftist_heap.md) | [hpp](../../ordered_structure/persistent_leftist_heap.hpp) | [例](../../examples/ordered_structure/persistent_leftist_heap.cpp) | [操作](../../visualizations/data_structure_extensions/index.html#persistent_leftist_heap) |
| D-07 | [Dynamic Segtree merge/split](../range_query/dynamic_segtree_merge_split.md) | [hpp](../../range_query/dynamic_segtree.hpp) | [例](../../examples/range_query/dynamic_segtree_merge_split.cpp) | [操作](../../visualizations/data_structure_extensions/index.html#dynamic_segtree_merge_split) |
| D-08 | [Segment Tree Cover](../range_query/segment_tree_cover.md) | [hpp](../../range_query/segment_tree_cover.hpp) | [例](../../examples/range_query/segment_tree_cover.cpp) | [操作](../../visualizations/data_structure_extensions/index.html#segment_tree_cover) |
| D-09 | [Segment Tree over Time](../offline_query/segment_tree_over_time.md) | [hpp](../../offline_query/segment_tree_over_time.hpp) | [例](../../examples/offline_query/segment_tree_over_time.cpp) | [操作](../../visualizations/data_structure_extensions/index.html#segment_tree_over_time) |
| D-10 | [DSU callback / Rollback data](../union_find/rollback_disjoint_set_union_with_data.md) | [hpp](../../union_find/rollback_disjoint_set_union_with_data.hpp) | [例](../../examples/union_find/rollback_disjoint_set_union_with_data.cpp) | [操作](../../visualizations/data_structure_extensions/index.html#rollback_disjoint_set_union_with_data) |

既存 API を保ちながら、永続 heap を k_shortest_walks と共有し、時間軸セグ木を OfflineDynamicConnectivity と共有しました。動的セグ木は不変ノードによるコピー・merge/split、通常/rollback DSU は併合 callback を追加しています。Point Set Range Frequency はオンラインの値変更に対応する O(log² n) 版です。

## 検証

```sh
make verify-data-structure-extensions
python3 -m tools.cp_tools.suite --runners data_structure_extensions
python3 -m tools.cp_tools.suite --runners data_structure_extensions --stdlib-debug
python3 -m tools.cp_tools.suite --runners data_structure_extensions --sanitize
node verify/visualizations/data_structure_extensions_model_test.js
node verify/visualizations/data_structure_extensions_browser_test.mjs --browser /path/to/chromium
```

単独・重複 include の debug/release、全カテゴリ混在と2翻訳単位、本文と完全一致する9使用例、C++/図解9サンプル、2ジャッジ形式×12入力を検証します。

- 両端キュー10万操作、静的クエリ401,041全探索区間、動的頻度142,000操作、全区間の標準ノード被覆、20万要素。
- 永続 heap 12,000分岐・20万要素構築/削除・共有版の自己 meld・size overflow。動的セグ木6,000分岐と非可換な文字列積、例外時保持、巨大座標。
- 付随情報 DSU 80,000分岐操作、101時刻長の時間軸を BFS と比較。古い枝/別個体 snapshot、callback/apply/visit の例外、move-only Event。
- 既存 graph_a/paths、a_priority/range、a_priority/dsu、b_1_6/spatial の比較検証も debug/release で回帰実行します。
- 図解モデルは2,718比較。ブラウザは9モード・5変更入力・390px幅・再生/移動/エラーを検証します。

[一次資料・設計判断](references.md) / [図解の範囲](../../visualizations/data_structure_extensions/README.md)
