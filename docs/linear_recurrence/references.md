# 一次資料・設計判断

2026-10-16に以下の一次資料・公式仕様を確認しました。外部のコード・説明文を転載せず、式と不変条件から独立に実装しています。

| 一次資料 | 確認した内容と今回の設計 |
|---|---|
| [CP-Algorithms: Binary Exponentiation](https://cp-algorithms.com/algebra/binary-exp.html) | 行列によるFibonacci・状態遷移と二分累乗。列ベクトル、uint64、形の例外を明示し、合成数法と法1にも対応 |
| [KACTL: LinearRecurrence](https://github.com/kth-competitive-programming/kactl/blob/main/content/numerical/LinearRecurrence.h) | 特性多項式の剰余による初期項への還元。NTTを使わないKitamasaを用意し、重み自体を公開 |
| [KACTL: BerlekampMassey](https://github.com/kth-competitive-programming/kactl/blob/main/content/numerical/BerlekampMassey.h)・[Nyaan BM](https://nyaannyaan.github.io/library/fps/berlekamp-massey.hpp.html) | 接続式と不一致量、2d観測の前提。空・全0、任意有限列の最短長、末尾0を残す規約を明示。独立したGaussian消去で最短性を検証 |
| [Bostan・Mori 原論文](https://arxiv.org/abs/2008.08822)・[Nyaan 線形漸化式](https://nyaannyaan.github.io/library/fps/kitamasa.hpp.html) | 有理母関数と遠い項の高速抽出。3.17の実装を共通入口から再利用 |
| [Library Checker: Find Linear Recurrence公式仕様](https://github.com/yosupo06/library-checker-problems/blob/master/other/find_linear_recurrence/task.md) | 最短次数と複数解を許す出力形式 |
| [Library Checker: Kth Term公式仕様](https://github.com/yosupo06/library-checker-problems/blob/master/other/kth_term_of_linearly_recurrent_sequence/task.md) | 初期項の昇順、直前からの係数順、0始まりの項番号 |
| [Library Checker: Pow of Matrix](https://judge.yosupo.jp/problem/pow_of_matrix) | 行列累乗の使用問題と入出力ラッパ |

KACTLの参照ファイルはCC0と記載されています。今回そのコードは取り込んでいません。CP-Algorithmsの文章はCC BY-SA 4.0です。既存依存の出典は[3.16](../convolution/references.md)・[3.17](../fps/references.md)も参照してください。

一般の行列クラスへ既存の旧APIを置換せず、このカテゴリに必要な正方行列の累乗・状態遷移をcp名前空間へ追加します。Kitamasaと行列は剰余環上、BMは有限体上、Bostan–MoriはNTT容量のある有限体上という違いを明示します。観測列の最短規則と、未知の無限列の保証を区別します。オンラインACは未確認です。
