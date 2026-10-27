# L追加項目の一次資料・設計判断

2026-10-27確認。公開APIと数学的な規約を参照し、本リポジトリのMatrix/BigInt/有限体多項式の部品から構成しました。外部ライブラリのソースは取り込んでいません。

| 対象 | 一次資料 | 今回の設計 |
|---|---|---|
| Smith・Hermite・Frobenius | [Magma Canonical Forms](https://magma.maths.usyd.edu.au/magma/handbook/text/280) | 整数はD=UAVと行側H=UAを規約にする。Magmaの最適化された標準形実装とは異なる基本Euclid法。FrobeniusはxI−Aの多項式Smith形と加群の生成元から実際の相似変換を作る |
| GF(2)多項式 | [NTL GF2X](https://libntl.org/doc/GF2X.cpp.html) | ゼロ次数−1、係数表現・剰余演算を参照。Boostの任意長bit列上の独立したshift/XOR実装 |
| 拡大有限体 | [NTL ZZ_pE](https://libntl.org/doc/ZZ_pE.cpp.html) | 多項式剰余による基底表現を参照。今回は素数・既約法を検査し、globalなcurrent modulusは使わない。Qで実装した有限体多項式を共有 |
| Pfaffian | [Wimmer原論文](https://arxiv.org/abs/1102.3440) | 交代行列の合同変形という考え方を参照。実数の数値安定化ではなく、素数体の2×2 Schur消去を実装 |
| 高速特性多項式 | [Dumas–Pernet–Wan原論文](https://arxiv.org/abs/cs/0501074)、[新しい一般決定的手法の論文](https://arxiv.org/abs/2010.04662) | Krylov倍増の計算量と一般法を区別する。今回は単一の左右射影・BM・次数nの証明条件とHessenberg fallbackを組み合わせる。一般のKeller–Gehrig branching全体や新しいO(n^ω)一般法の実装ではない |
| ジャッジ仕様 | [Characteristic Polynomialの公式仕様](https://github.com/yosupo06/library-checker-problems/blob/master/linear_algebra/characteristic_polynomial/task.md) | 係数の昇冪順と空行列式1。オンラインACとローカルの入出力比較を区別 |

整数行列式はBareissの恒等式、合成数法は整数Euclid行操作、PermanentはRyserの包除式、Hafnianは最小未使用頂点との組合せDPから構成しました。BigIntのBoost依存と既存部品の出典は各元ドキュメントを引き継ぎます。
