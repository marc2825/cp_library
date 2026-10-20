# U-01〜U-22の原理・出典

[対応表・検証](README.md) / [出典管理の規約](../development_records/provenance.md)

2026-10-20に確認。アルゴリズムの参照、コードを改変して利用したもの、既存部品への依存を区別します。

- **順列木**：[hitonanode本人の公開実装と解説](https://hitonanode.github.io/cplib-cpp/other_algorithms/permutation_tree.hpp.html)を改変しました。MIT、Copyright (c) 2019 Ryotaro Sato。[上流ライセンス](https://raw.githubusercontent.com/hitonanode/cplib-cpp/master/LICENSE)、[同梱した全文](../../verification/licenses/hitonanode-MIT.txt)。既存LazySegtreeを使うよう変更し、cp名前空間・入力検査・空順列・値域の半開区間・constアクセサ・区間数APIを追加しました。ページに掲載された構築コードを参照しており、取り込みcommitは未確定です。公開ページと閲覧日、ローカルソースのSHA256を[台帳](../../verification/provenance.json)へ記録しました。上流のAC表記をこの改変版のAC実績として引き継ぎません。
- **黄金分割**：[SciPyの公式golden説明](https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.golden.html)も関数評価と探索区間の縮小を説明しています。こちらのAPIは有限の閉区間内を探索し、SciPyのブラケット自動拡張・相対誤差による終了とは異なります。コードは独自実装です。
- **フィボナッチ探索**：J. Kiefer, “Sequential minimax search for a maximum”, Proceedings of the AMS 4 (1953), 502–506, [DOI](https://doi.org/10.1090/S0002-9939-1953-0055639-3)。本実装は整数領域をフィボナッチ長へ拡張し、範囲外を仮想の+∞とする独自の整数版です。評価回数の契約は[個別説明](../basic_utility/unimodal_search.md)とローカルテストによります。
- **尺取り・Kadane・半分全列挙・順列演算**：区間の部分区間閉性、右端DP、ソート済み部分和のマージ、サイクル分解を直接実装しました。各説明に不変条件と計算量を記載しています。
- **括弧・整数分割・集合分割**：補完数DP、非増加整数分割、restricted-growth表現の定義に基づく独自実装です。巨大な個数は既存 [BigInt](../basic_utility/big_integer.md) を利用します。
- **動的MEX**：既存 [RangeSet](../basic_utility/range_set.md) で非負値の被覆を管理します。**集合hash**：既存 [SeededHash](../basic_utility/seeded_hash.md) を使用します。**部分永続の探索**：既存 [部分永続DSU](../../union_find/partially_persistent_disjoint_set_union.hpp) を参照するアダプタを追加し、Rollbackとは別の版履歴配列を用意しました。

問題例：[Library Checker / Common Interval Decomposition Tree](https://judge.yosupo.jp/problem/common_interval_decomposition_tree)。他の各ページにも具体的な使用場面を記載しています。オンライン提出による検証は実施していません。

- **過去操作編集**：[Demaine・Iacono・Langerman, Retroactive Data Structures](https://erikdemaine.org/papers/Retroactive_SODA2004/)を概念の参考にしました。固定時刻上のMonoid合成に限定した独自ラッパで、既存[Segtree](../../range_query/segtree.hpp)を共有します。Segtree自体のACL由来の記録は既存台帳を保持します。
- **特殊整数列挙**：[ABC363公式解説](https://atcoder.jp/contests/abc363/editorial/10488)の回文の前半表現、[Jayadev Misra, Programming in Orc, §6.3.2.4](https://www.cs.utexas.edu/~misra/OrcBook.pdf)のHamming列を原理の参考にしました。数字制限は10進prefix木の直接走査です。3方式ともコードは独自実装です。
