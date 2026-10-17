# 3.1 基本ユーティリティ・探索

C++17、namespace cp。共通入口は `basic_utility/all.hpp`。各ヘッダを単独でincludeできます。

## A項目の追加（2026-10-17）

| 機能 | 実装 | 説明・usage・計算量 | 図解 |
|---|---|---|---|
| 循環列の操作 | [hpp](../../basic_utility/cyclic_sequence.hpp) | [説明](cyclic_sequence.md) | [操作](../../visualizations/a_priority/index.html#cyclic_sequence) |
| 区間のマージ・分割 | [hpp](../../basic_utility/interval_operations.hpp) | [説明](interval_operations.md) | [操作](../../visualizations/a_priority/index.html#interval_operations) |
| 商列挙 | [hpp](../../basic_utility/quotient_enumeration.hpp) | [説明](quotient_enumeration.md) | [操作](../../visualizations/a_priority/index.html#quotient_enumeration) |
| 範囲内の倍数列挙 | [hpp](../../basic_utility/multiple_enumeration.hpp) | [説明](multiple_enumeration.md) | [操作](../../visualizations/a_priority/index.html#multiple_enumeration) |
| Gray Code | [hpp](../../basic_utility/gray_code.hpp) | [説明](gray_code.md) | [操作](../../visualizations/a_priority/index.html#gray_code) |
| 部分集合の部分集合列挙 | [hpp](../../basic_utility/subset_pairs.hpp) | [説明](subset_pairs.md) | [操作](../../visualizations/a_priority/index.html#subset_pairs) |
| 順列のランク・アンランク | [hpp](../../basic_utility/permutation_rank.hpp) | [説明](permutation_rank.md) | [操作](../../visualizations/a_priority/index.html#permutation_rank) |

既存のA項目は実装を共有します。

- [ダブリング](../tree/binary_lifting.md)
- [一つのmaskの部分集合列挙](../set_function/submask_enumeration.md)
- [Cartesian Tree（旧API）](../../data_structure/cartesian_tree.hpp)
- [XOR基底](../linear_algebra/xor_basis.md)
- [約数列挙](../number_theory_basic/divisors.md)

今回の追加分は `python3 verify/run_a_priority.py` / `--sanitize`（`make verify-a-priority`）で検証します。[検証範囲・共有設計](../a_priority/README.md) / [一次資料・問題例](../a_priority/references.md)。オンラインACは未確認です。
