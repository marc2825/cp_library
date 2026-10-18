# 3.12〜3.15 B：一次資料と設計判断

2026-10-18に仕様と原理を確認しました。[対応表](README.md) / [図解の範囲](../../visualizations/b_12_15/README.md)。実行時の外部ライブラリ依存はありません。

| 一次資料 | 参照内容・この実装での判断 |
|---|---|
| [noshi91 LARSCH](https://noshi91.github.io/Library/algorithm/larsch.cpp.html) / [CC0の明記](https://github.com/noshi91/Library#readme) | 下三角行列の行縮約・列縮約とオンラインで参照可能な列の条件を確認。CC0コードをC++17の公開クラスへ移植し、reset・空入力・超過検出を追加。ヘッダにも帰属を記載。 |
| [maspypy Monge辺数制約](https://maspypy.github.io/library/convex/monge/monge_shortest_path_d_edge.hpp.html) | Monge最短路の辺数制約という問題を確認。こちらは既存SMAWKを各段へ使い、罰則探索の範囲や同点処理なしでO(KN)・経路復元に対応。 |
| [Library Checker 凹×任意Min-Plus](https://judge.yosupo.jp/problem/min_plus_convolution_concave_arbitrary) / [凸×任意](https://judge.yosupo.jp/problem/min_plus_convolution_convex_arbitrary) | 有限列の境界も含む畳み込み仕様を確認。凸版は既存実体、凹版は長さM以下への分割と前後2つの部分包絡を実装。下記の証明と直接全組合せで検証。 |
| [Communication-Efficient Private Protocols for Longest Common Subsequence](https://eprint.iacr.org/2009/019) | Four Russiansの小ブロック表引きという考え方と原法の文献を確認。今回はBoolean行列積のM4RMを実装し、LCS専用版と区別。 |
| [Nyaan Tonelli–Shanks](https://nyaannyaan.github.io/library/modulo/mod-sqrt.hpp.html) | 2冪位数の誤差修正を確認。こちらは既存の64bit素数検査と安全なmod演算を共有し、決定的な非剰余探索と小さい方の根を返す。 |
| [Library Checker 離散対数](https://judge.yosupo.jp/problem/discrete_logarithm_mod) / [Tetration Mod](https://judge.yosupo.jp/problem/tetration_mod) | 最小非負指数、0^0、有限高さの規約を確認。BSGSはgcd除去で一般の法へ対応。テトレーションは実指数がφ以上かを別に判定。 |
| [rsk0315 素数の数え上げと乗法的関数の和](https://rsk0315.github.io/slides/prime-counting.pdf) | Lucy DPの商状態と素数ごとの更新順を確認。今回は素数個数だけを扱うO(N^(3/4))上界の基本版で、一般化Min_25はCに残す。 |
| [Nyaan 商列挙](https://nyaannyaan.github.io/library/math/enumerate-quotient.hpp.html) | 同一商の範囲をまとめる考え方を確認。このリポジトリでは既存for_each_quotientの閉区間[l,r]・商の降順を保持。外部版の(l,r]や走査順と混同しない。 |
| [Library Checker φの総和](https://judge.yosupo.jp/problem/sum_of_totient_function) | 問題の数学的総和を確認。こちらは整数の正確な128bit結果を返し、剰余は呼出側が取る。 |
| [Gessel, Lagrange Inversion](https://arxiv.org/abs/1609.05988) | Lagrange–Bürmannの係数公式を確認。g自体を全復元せず、既存FPS冪からF(g)の指定係数を得る。 |
| [Library Checker Bernoulli Number](https://judge.yosupo.jp/problem/bernoulli_number) / [標本点シフト](https://judge.yosupo.jp/problem/shift_of_sampling_points_of_polynomial) | 規約と問題仕様の参照。BernoulliはB1=-1/2。標本点シフトは既存補間とProductTree評価を共有し、専用連続点アルゴリズムではない計算量を明記。 |

## 凹列と任意列のMin-Plus

bの長さをMとし、任意列aを長さM以下のblockへ分けます。1blockの前半k=0..M−1では有効な列jが0..min(k,L−1)なので、候補の寿命切れがありません。j<j'について、凹性からb[k−j]−b[k−j']はkに対して非増加です。そのため、新しい候補j'が既存の最小包絡に勝つ範囲は現在以降のprefixです。

dequeに各候補と担当区間の右端を置きます。区間全体で負ける先頭候補を除き、最初に既存候補が勝つ境界を二分探索して新候補を先頭へ追加します。各候補の追加・除去は一度ずつ。後半はaのblockとbを反転して同じ処理を行い、元の順へ戻します。各blockの結果をminで重ねることで元の畳み込みが得られ、全体でO(N log(M+1)+M)です。符号反転を使わず、凹×凹の線形端点版も別に提供します。

## 共有・適用範囲

整数根・商列挙は3.1の既存実装、素数カウントは3.13/3.14で同じLucy実体を共有。Factorial/Pascal/Stirling・SMAWK・NTT・FPS・SPF・φ/μ・Pollard–Rhoも共有します。

乗法的関数の総和は全prefixを作る線形篩版、二項変換・Stirling変換・階乗基底・二項prefixは一般法に対応する二乗表版です。一般の巨大Nの乗法的総和、合成数法のLucas拡張、LCS用Four Russians、特殊な高速二項変換を実装済みとは数えません。外部ライブラリのACをこの実装の検証実績に転用しません。
