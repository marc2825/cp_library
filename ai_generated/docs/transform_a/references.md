# 一次資料と設計判断

2026-10-17に以下の著者資料・実装・公式問題仕様を確認しました。コード・説明文は転載せず、定義と既存APIに合わせて実装しています。

| 一次資料 | 確認内容と今回の設計 |
|---|---|
| [Cheng: Space-Efficient Karatsuba Multiplication](https://arxiv.org/abs/1605.06760) | 通常のKaratsubaの時間・領域計算量。今回は係数列の3再帰積で、論文のin-place整数実装は取り込まない |
| [Nyaan: Walsh Hadamard Transform](https://nyaannyaan.github.io/library/set-function/walsh-hadamard-transform.hpp.html) | 自然順butterflyと逆正規化。今回は整数を段ごとにexact divisionし、超過と割り切れを検査 |
| [Nyaan: Subset Convolution](https://nyaannyaan.github.io/library/set-function/subset-convolution.hpp.html) | 要素数のrankと部分集合変換。今回は[rank][mask]の公開APIに切り出し、既存Subset Convolutionから共有 |
| [hitonanode: Bitset Linear Algebra](https://hitonanode.github.io/cplib-cpp/linear_algebra_matrix/linalg_bitset.hpp) | bitsetの行XOR、実際の変数数、特解・自由変数。今回はRHSをpivotにせず、係数の階数と矛盾を分離 |
| [CP-Algorithms: Gauss](https://cp-algorithms.com/linear_algebra/linear-system-gauss.html) | 行操作とpivot、自由変数。逆行列では右側の単位行列を同時に変形し、交差は係数の同次方程式へ帰着 |
| [hitonanode: Hessenberg Reduction](https://hitonanode.github.io/cplib-cpp/linear_algebra_matrix/hessenberg_reduction.hpp) | 行操作と逆向きの列操作による相似変形。今回は体版だけを提供し、合成数環版は含めない |
| [hitonanode: Characteristic Polynomial](https://hitonanode.github.io/cplib-cpp/linear_algebra_matrix/characteristic_poly.hpp.html) | 主小行列の多項式の漸化式。Hessenberg化と組み合わせ、次数を割らずに小標数でも扱う |
| [Library Checker XOR公式仕様](https://raw.githubusercontent.com/yosupo06/library-checker-problems/master/convolution/bitwise_xor_convolution/task.md) | XORで集計する添字と自然な出力順。ORはANDの補集合ラッパも確認 |
| [Library Checker GF(2)公式仕様](https://raw.githubusercontent.com/yosupo06/library-checker-problems/master/linear_algebra/system_of_linear_equations_mod_2/task.md) | 行・ベクトルの01文字列形式と、特解・解空間の基底の出力 |
| [Library Checker 特性多項式公式仕様](https://raw.githubusercontent.com/yosupo06/library-checker-problems/master/linear_algebra/characteristic_polynomial/task.md) | det(xI−A)、定数項からの出力順、0次元の{1} |

線形空間の交差は、両側を独立基底にしてから[Uᵀ|-Vᵀ]の零空間を求める設計です。GF(2)の整数版は消去にU側の成分を随伴させます。共有XorBasis・Gauss–JordanのAPIを保持し、空間の和と交差を別の操作として提供します。

標準整数の超過検査は既存の[算術補助](../../number_theory_convolution/detail/arithmetic.hpp)、素数法は[既存の体型](../../linear_algebra/detail/prime_field.hpp)を共有します。外部modintの実行依存はありません。テスト用mod型はverify内のみ。CP-Algorithmsの文章はCC BY-SA 4.0ですが、本文・コードは転載していません。全てローカル検証で、オンラインACは未確認です。
