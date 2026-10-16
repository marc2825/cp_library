# 一次資料・設計判断

2026-10-16に次の一次資料と問題仕様を確認しました。コードや説明文の転載はせず、標準的な代数式から独立に実装しています。

| 一次資料 | 確認した内容・設計への反映 |
|---|---|
| [Nyaan: FPS](https://nyaannyaan.github.io/library/fps/formal-power-series.hpp.html) | inv/log/exp/pow、反転による除算、精度倍増。今回は既存の整数vector APIと揃え、継承クラスやmodintを導入せず個別ヘッダの関数として提供 |
| [Nyaan: 平方根](https://nyaannyaan.github.io/library/fps/fps-sqrt.hpp.html) | 先頭0の次数と定数の平方根。nulloptで不成立を表し、最小先頭根と未確定な末尾0の規約を明記 |
| [Nyaan: Taylor Shift](https://nyaannyaan.github.io/library/fps/taylor-shift.hpp) | 階乗による畳み込みへの変形。既存のFactorialTableは実行時法なので、固定法の内部前計算を使用 |
| [Nyaan: 線形漸化式](https://nyaannyaan.github.io/library/fps/nth-term.hpp.html)・[kitamasa](https://nyaannyaan.github.io/library/fps/kitamasa.hpp.html) | 母関数P/Qと偶奇抽出。既知の係数を受け取り、Berlekamp–Masseyは今回の対象に含めない |
| [CP-Algorithms: Polynomials](https://cp-algorithms.com/algebra/polynomial.html) | 積木による評価・補間。点の順序・重複条件を明示し、補間はバッチ逆元を使用 |
| [Library Checker公式 polynomial問題群](https://github.com/yosupo06/library-checker-problems/tree/master/polynomial) | inv/log/exp/pow/sqrt・除算・多点評価・補間・平行移動の入出力。10ラッパを独立オラクルのローカルケースで検証 |
| [Library Checker: 線形漸化式](https://judge.yosupo.jp/problem/kth_term_of_linearly_recurrent_sequence) | 初期項と直前からの係数順、0始まりの項番号 |

[3.16の畳み込み](../convolution/references.md)をリポジトリ内依存として再利用します。新規コードは外部実装のコピーではありません。Nyaanのコードのライセンスは[公式LICENSE](https://github.com/NyaanNyaan/library/blob/master/LICENSE)、CP-Algorithmsの文章はCC BY-SA 4.0です。

精度上限を明示的なNTT作業容量の半分に制限します。高速なNTTの再利用・周回畳み込みを使って上限まで拡張する実装よりも、各補正を通常の多項式積で確認しやすい構成を選んでいます。多点評価は一括APIで、積木の永続的な再利用APIは公開しません。sqrtの不成立と不正入力・容量超過を区別します。オンラインジャッジのACは未確認です。
