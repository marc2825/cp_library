# 一次資料・設計判断

2026-10-16に次の一次資料を確認しました。外部のコードや説明文を転載せず、定義式と既存APIに合わせて実装しています。

| 一次資料 | 確認した内容・今回の設計 |
|---|---|
| [CP-Algorithms: Binary Exponentiation](https://cp-algorithms.com/algebra/binary-exp.html) | 結合的な積、行列による状態遷移と2冪。矩形の積と正方行列の累乗を分け、最後の不要な二乗と整数超過を避ける |
| [CP-Algorithms: Gauss method](https://cp-algorithms.com/linear_algebra/linear-system-gauss.html) | 行交換・消去、実数のpivot、長方形と自由変数。特解に加えて全零空間基底を返し、素数法と実数の条件を分ける |
| [USACO Guide: XOR Basis](https://usaco.guide/adv/xor-basis?lang=cpp) | GF(2)上の独立性と張る空間。最上位bitごとの固定配列、数値順のmin/maxと所属判定、64ビット全域を提供 |
| [Library Checker: System of Linear Equations公式仕様](https://raw.githubusercontent.com/yosupo06/library-checker-problems/master/linear_algebra/system_of_linear_equations/task.md) | mod998244353で特解と独立な基底を返す出力仕様。ローカルでは出力値の一致だけでなく、残差・次元・全解空間を検証 |
| [AtCoder ABC141 F: Xor Sum 3公式問題](https://atcoder.jp/contests/abc141/tasks/abc141_f) | 60ビットの非負整数と、両側を非空にする分割の条件。XOR基底を使うラッパを全分割と比較 |

Matrixは行優先の配列に次元を別に保持し、0×mを失わない設計です。旧matrixのvector<vector<T>>を受け取るグローバル関数は保持し、3.18の漸化式用APIにも変更を加えません。

標準整数の積・加減算検査は[3.14の算術補助](../../number_theory_convolution/detail/arithmetic.hpp)、素数法版の正規化・積・逆元は[3.18の内部補助](../../linear_recurrence/detail/common.hpp)を明示includeして再利用します。行列演算の独自型は利用者が渡すため、外部modintへの実行依存はありません。ジャッジ用の小さなmod型はverify内にだけ置きます。

実数のGauss–Jordanは絶対epsilonと最大絶対値pivotを使う数値的な解法で、任意のスケール・悪条件で正しい階数や高精度を保証するものではありません。厳密な整数除算や合成数法の一般の方程式には使わず、必要なら適切な体や別の解法を選びます。CP-Algorithmsの文章はCC BY-SA 4.0。今回外部コードは取り込んでいません。オンラインACは未確認です。
