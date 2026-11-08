# 数値解析の一次資料・設計判断

[カテゴリ](README.md) / [図解](../../visualizations/numerical_analysis/README.md)

公開API・数式の確認に使った資料です。外部の実装コードは移植していません。既存のBigRational・二進変換・図解のBigInt有理数を共有しています。

- [GNU GSL: One dimensional root finding](https://www.gnu.org/software/gsl/doc/html/roots.html)：区間法と局所改良法、二分・Brent・Newton、残差／区間幅の停止条件。C++では評価予算と停止理由を明示し、Newtonの小さい更新だけでは成功にしません。
- [SciPy: Simpson](https://docs.scipy.org/doc/scipy/reference/generated/scipy.integrate.simpson.html)：等間隔の奇数個の点で三次以下を積分する性質。本APIのnは点数ではなく正の偶数の小区間数です。適応版はSimpsonの4次誤差から導くRichardson補正を使い、推定を厳密保証とは扱いません。
- [Higham: The Accuracy of Floating Point Summation](https://epubs.siam.org/doi/10.1137/0914050)：集計順序と補償和の精度。KahanとNeumaierを選べるAPIにし、任意入力で正確とする契約は置きません。
- [Blanchard・Higham・Higham: Accurately Computing the Log-Sum-Exp and Softmax Functions](https://eprints.maths.manchester.ac.uk/2728/)：最大値を引く変形とその数値安定性。最大項以外は補償和とlog1pで評価します。
- [Moreno Maza: Real Root Isolation tutorial](https://www.csd.uwo.ca/~mmorenom/Publications/RS-SAV-Tutorial.pdf)：平方因子除去・Sturm列・符号変化による実根分離。浮動係数の表現値を厳密有理数として扱い、近い根もcountを保って分離します。
- [LAPACK: DGTSV](https://www.netlib.org/lapack/explore-html/d6/ddb/group__gtsv_ga6b06a11b8e2dced10bc9cd3c20a09484.html)：部分pivotによる三重対角消去と第2上対角。本APIは因子を保持して複数右辺へ再利用でき、特異pivotの閾値は呼び出し側で指定します。

絶対＋相対誤差比較は数値停止用です。幾何のトポロジーには[厳密predicate](../computational_geometry/robust_predicates.md)を使います。二次方程式の判別式の符号も同じ二進有理変換で確定します。
