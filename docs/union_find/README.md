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
