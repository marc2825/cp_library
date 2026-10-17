# 3.2 Union-Find・連結性管理

| ランク | 機能 | 実装 | 詳細 | 操作できる解説 |
|---|---|---|---|---|
| S | 通常版・成分サイズ・個数 | [hpp](../../union_find/disjoint_set_union.hpp) | [説明](disjoint_set_union.md) | [図解](../../visualizations/union_find/index.html#ordinary) |
| S | 重み付き版・差分制約・矛盾判定 | [hpp](../../union_find/potentialized_disjoint_set_union.hpp) | [説明](potentialized_disjoint_set_union.md) | [図解](../../visualizations/union_find/index.html#potential) |

必要なヘッダだけ include できます。一括利用は `union_find/all.hpp`。C++17 以降、外部依存なしです。

[参考資料と設計](references.md) / [通常版の使用例](../../examples/union_find/connectivity.cpp) / [重み付き版の使用例](../../examples/union_find/potential_constraints.cpp)

```sh
python3 verify/run_union_find.py
python3 verify/run_union_find.py --sanitize
```

## 高度なアルゴリズムの追加

- [Rollback DSU](rollback_disjoint_set_union.md) / [操作できる図解](../../visualizations/advanced/index.html#rollback)

追加分の検証は `python3 verify/run_advanced.py`（`--sanitize`対応）。[全追加項目と既存項目の対応表](../advanced/README.md)。

## A項目の追加（2026-10-17）

| 機能 | 実装 | 説明・usage・計算量 | 図解 |
|---|---|---|---|
| 部分永続Union-Find | [hpp](../../union_find/partially_persistent_disjoint_set_union.hpp) | [説明](partially_persistent_disjoint_set_union.md) | [操作](../../visualizations/a_priority/index.html#partially_persistent_disjoint_set_union) |
| 完全永続Union-Find | [hpp](../../union_find/persistent_disjoint_set_union.hpp) | [説明](persistent_disjoint_set_union.md) | [操作](../../visualizations/a_priority/index.html#persistent_disjoint_set_union) |
| 連結成分のモノイド集約 | [hpp](../../union_find/aggregate_disjoint_set_union.hpp) | [説明](aggregate_disjoint_set_union.md) | [操作](../../visualizations/a_priority/index.html#aggregate_disjoint_set_union) |
| 二部グラフ判定付きUnion-Find | [hpp](../../union_find/bipartite_disjoint_set_union.hpp) | [説明](bipartite_disjoint_set_union.md) | [操作](../../visualizations/a_priority/index.html#bipartite_disjoint_set_union) |
| 区間Union-Find | [hpp](../../union_find/interval_disjoint_set_union.hpp) | [説明](interval_disjoint_set_union.md) | [操作](../../visualizations/a_priority/index.html#interval_disjoint_set_union) |
| マージ履歴木 | [hpp](../../union_find/merge_tree.hpp) | [説明](merge_tree.md) | [操作](../../visualizations/a_priority/index.html#merge_tree) |

今回の追加分は `python3 verify/run_a_priority.py` / `--sanitize`（`make verify-a-priority`）で検証します。[検証範囲・共有設計](../a_priority/README.md) / [一次資料・問題例](../a_priority/references.md)。オンラインACは未確認です。
