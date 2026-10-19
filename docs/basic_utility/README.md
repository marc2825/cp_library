# 3.1 基本ユーティリティ・探索

C++17、namespace cp。共通入口は `basic_utility/all.hpp`。各ヘッダを単独でincludeできます。

[V-05：デバッグ出力](debug.md)を追加しました。コンテナ・tuple・任意型の整形と、提出時に引数評価ごと消すCP_DEBUG。[図解](../../visualizations/development_runtime/index.html#debug)。`make verify-development-runtime`。

## 共通基盤 F-12〜F-17（2026-10-19）

[追加範囲・検証](../foundation_tools/README.md)。`make verify-foundation-tools`。

| ID | 説明・API | 図解 |
|---|---|---|
| F-12 | [多倍長有理数](big_rational.md) | [操作](../../visualizations/foundation_tools/index.html#big_rational) |
| F-13 | [型付きArena](arena.md) | [操作](../../visualizations/foundation_tools/index.html#arena) |
| F-14 | [ビット演算・固定popcount列挙](bit_operations.md) | [操作](../../visualizations/foundation_tools/index.html#bit_operations) |
| F-15 | [グリッド共通処理](grid.md) | [操作](../../visualizations/foundation_tools/index.html#grid) |
| F-16 | [Rollback配列](rollback_array.md) | [操作](../../visualizations/foundation_tools/index.html#rollback_array) |
| F-17 | [安定基数ソート](radix_sort.md) | [操作](../../visualizations/foundation_tools/index.html#radix_sort) |

## 共通基盤 F-11（2026-10-19）

[多倍長整数BigInt](big_integer.md) / [図解](../../visualizations/big_integer/index.html)。Boostヘッダが必要な符号付き任意精度整数です。加減乗除・剰余・GCD・LCM、厳密な10進／16進変換、128bitまでの範囲検査付き変換を扱います。入力1万桁・積2万桁まで独立比較。`make verify-big-integer`。

## 共通基盤 F-08（2026-10-19）

[seed付き複合キーハッシュ](seeded_hash.md) / [Open Addressing Hash Map](hash_map.md) / [操作できる図解](../../visualizations/foundation_structures/index.html#hash_map)。整数・pair・tuple・列のhash、衝突・削除・再構築、固定seedでの再現に対応します。[検証範囲](../foundation_structures/README.md)。`make verify-foundation-structures`。

## 共通基盤 F-01〜F-07（2026-10-19）

[対応範囲・検証](foundation.md) / [一次資料](references.md)。`make verify-foundation` / `python3 verify/run_foundation.py --sanitize`。

| ID | 機能 | 実装 | 図解 |
|---|---|---|---|
| F-01 | [固定法modint](static_modint.md) | [hpp](../../basic_utility/static_modint.hpp) | [操作](../../visualizations/foundation/index.html#static_modint) |
| F-02 | [実行時法modint](dynamic_modint.md) | [hpp](../../basic_utility/dynamic_modint.hpp) | [操作](../../visualizations/foundation/index.html#dynamic_modint) |
| F-03 | [Barrett Reduction](barrett_reduction.md) | [hpp](../../basic_utility/barrett_reduction.hpp) | [操作](../../visualizations/foundation/index.html#barrett_reduction) |
| F-04 | [Montgomery Reduction・64bit modint](montgomery_modint.md) | [hpp](../../basic_utility/montgomery_modint.hpp) | [操作](../../visualizations/foundation/index.html#montgomery_modint) |
| F-05 | [2^61−1 modint](modint61.md) | [hpp](../../basic_utility/modint61.hpp) | [操作](../../visualizations/foundation/index.html#modint61) |
| F-06 | [共通FastIO](fast_io.md) | [hpp](../../basic_utility/fast_io.hpp) | [操作](../../visualizations/foundation/index.html#fast_io) |
| F-07 | [安全な整数演算](safe_integer.md) | [hpp](../../basic_utility/safe_integer.hpp) | [操作](../../visualizations/foundation/index.html#safe_integer) |

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

## B項目の整備（2026-10-17）

| 機能 | 実装 | 詳細 | 図解 |
|---|---|---|---|
| Fractional Cascading | [hpp](../../basic_utility/fractional_cascading.hpp) | [説明・usage](fractional_cascading.md) | [操作](../../visualizations/b_1_6/index.html#fractional_cascading) |
| 整数k乗根 | [hpp](../../basic_utility/integer_kth_root.hpp) | [説明・usage](integer_kth_root.md) | [操作](../../visualizations/b_1_6/index.html#integer_kth_root) |
| Stern–Brocot Tree | [hpp](../../basic_utility/stern_brocot_tree.hpp) | [説明・usage](stern_brocot_tree.md) | [操作](../../visualizations/b_1_6/index.html#stern_brocot_tree) |
| 分数二分探索 | [hpp](../../basic_utility/fraction_binary_search.hpp) | [説明・usage](fraction_binary_search.md) | [操作](../../visualizations/b_1_6/index.html#fraction_binary_search) |
| 平方分割テンプレート | [hpp](../../basic_utility/sqrt_decomposition.hpp) | [説明・usage](sqrt_decomposition.md) | [操作](../../visualizations/b_1_6/index.html#sqrt_decomposition) |
| 非単調関数の探索 | [hpp](../../basic_utility/nonmonotone_search.hpp) | [説明・usage](nonmonotone_search.md) | [操作](../../visualizations/b_1_6/index.html#nonmonotone_search) |
| 連続区間集合の管理 | [hpp](../../basic_utility/range_set.hpp) | [説明・usage](range_set.md) | [操作](../../visualizations/b_1_6/index.html#range_set) |

検証: `python3 verify/run_b_1_6.py` / `--sanitize` / `make verify-b-1-6`。[対応範囲・検証](../b_1_6/README.md) / [一次資料](../b_1_6/references.md)。
