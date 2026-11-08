# 3.20 線形代数

[ライブラリ一覧](../../README.md) / [全図解](../../visualizations/index.html) / [一次資料・設計判断](references.md)

S項目を独立した`linear_algebra/`にまとめています。[all.hpp](../../linear_algebra/all.hpp)が入口です。

| S項目 | API・usage | 条件・出力 | 図解 |
|---|---|---|---|
| Matrix | [Matrix<T>](matrix.md) | 長方形・行優先、0行／0列も次元を保持 | [転置と保存順](../../visualizations/linear_algebra/index.html#matrix) |
| 行列積 | [matrix_multiply / matrix_vector_multiply](matrix_product.md) | 内側の次元を一致、列ベクトル、整数・独自mod型・実数等 | [行と列の内積](../../visualizations/linear_algebra/index.html#product) |
| 行列累乗 | [matrix_power](matrix_power.md) | 正方行列、uint64指数、0乗は単位行列 | [二分累乗](../../visualizations/linear_algebra/index.html#power) |
| Gauss–Jordan | [gauss_jordan / gauss_jordan_mod](gauss_jordan.md) | 厳密な体／実数／素数法、特解と零空間の基底 | [行基本変形と解空間](../../visualizations/linear_algebra/index.html#gauss) |
| XOR基底 | [XorBasis](xor_basis.md) | 符号なし整数、挿入・所属・min/max・merge | [ビット消去](../../visualizations/linear_algebra/index.html#xor) |

## 共通仕様と選び方

- C++17、namespace cp、標準ライブラリと明示したリポジトリ内includeだけで使えます。各ヘッダは単独include可能。旧行列はリポジトリ直下の `depricated/matrix/matrix_basic.hpp` に保存し、3.18のAPIは引き続き利用できます。
- Matrix<T>は形と行優先のvector<T>を保持します。要素は`A(row,col)`、0始まり。0×mとn×0も区別。次元の容量超過はlength_error、形の違反はinvalid_argumentです。
- 行列積と累乗はT(0)、積・加算、累乗ではT(1)が必要。独自mod型なら合成数法・法1も可。標準整数は各積・部分和の範囲超過を検出します。標準浮動小数点では非有限値を検出し、有限の誤差やアンダーフローは許容します。独自型の演算条件は利用者が管理します。boolは未対応です。
- Gauss–Jordanは除算できる体専用で、整数の通常の割り算は拒否。実数版は部分ピボットと絶対epsilon、`gauss_jordan_mod<Mod>`は素数法2〜2×10^9です。積・累乗が合成数法でも使えることと、方程式の解法の条件を区別してください。
- 解はno_solution / unique / multiple。解があれば`particular + Σ t[j]*nullspace_basis[j]`で全てを表し、rank・pivot列・拡大行列の簡約結果も返します。0方程式では全ての変数が自由です。
- XOR基底はGF(2)上のビットベクトルを扱います。既定uint64全域、0と重複は挿入しません。min/maxはseed XOR spanの符号なし数値順で、0も含む空の選択を許します。
- [3.18 線形漸化式](../linear_recurrence/README.md)のmatrix_power_mod / matrix_power_applyは、vector<vector<int>>を入力とする既存の用途別APIです。一般の行列と方程式はこちらを使います。

## ローカル検証

```sh
python3 verify/run_linear_algebra.py
python3 verify/run_linear_algebra.py --sanitize
node verify/visualizations/linear_algebra_model_test.js
node verify/visualizations/linear_algebra_browser_test.mjs --browser /path/to/chromium
```

`--compiler clang++`で切り替えられます。8ヘッダ、5使用例、4ジャッジ形式・256ケース、Python任意精度1,361クエリ。500長方形積・1,950反復累乗比較、有限体5,915系の解の全列挙、実数160系・有理数・複素数・法1・合成数法、65,536 int8積、37,449 XOR系列・100全幅系列を確認します。128変数の消去、256自由変数の出力、32×32巡回行列のuint64最大乗、64個の独立pivotと100,000回の挿入、整数超過と非有限値を含みます。通常・サニタイザのdebug/releaseを実行します。オンラインACは未確認です。

図解は5モードで、各次元5まで、XOR入力12個まで。行優先の保存と転置、内積の各項と部分和、二分累乗の不変条件、pivot・行交換・正規化・消去、特解と零空間、64ビットの基底を操作できます。2,413入力・27,172スナップショット・17拒否条件を独立に検証します。

## 3.16〜3.20 A項目の補完

- [逆行列](inverse.md) / [操作できる図解](../../visualizations/transform_a/index.html#inverse)
- [GF(2)上のBitset Gauss](gf2_gauss.md) / [操作できる図解](../../visualizations/transform_a/index.html#gf2)
- [線形空間の交差](space_intersection.md) / [操作できる図解](../../visualizations/transform_a/index.html#intersection)
- [特性多項式](characteristic_polynomial.md) / [操作できる図解](../../visualizations/transform_a/index.html#charpoly)

[追加分のAPI条件・使用例・検証範囲](../transform_a/README.md)。`python3 verify/run_transform_a.py` / `--sanitize` / `make verify-transform-a`。

## 3.16〜3.20 B項目

- [Sparse Matrix](sparse_matrix.md) / [操作できる図解](../../visualizations/b_16_20/index.html#sparse_matrix)
- [Black-Box Linear Algebra](black_box.md) / [操作できる図解](../../visualizations/b_16_20/index.html#black_box)
- [Wiedemann Algorithm](wiedemann.md) / [操作できる図解](../../visualizations/b_16_20/index.html#wiedemann)
- [行列の最小多項式](minimal_polynomial.md) / [操作できる図解](../../visualizations/b_16_20/index.html#minimal_polynomial)
- [Matrix Tree Theorem（共通入口）](matrix_tree.md) / [操作できる図解](../../visualizations/b_16_20/index.html#matrix_tree)
- [Low-Rank Update](low_rank_update.md) / [操作できる図解](../../visualizations/b_16_20/index.html#low_rank_update)

[全31項目のAPI・条件・比較検証](../b_16_20/README.md)。`python3 verify/run_b_16_20.py` / `--sanitize` / `make verify-b-16-20`。

## L全11項目の追加

[12公開ヘッダ・API条件・検証](../linear_algebra_extensions/README.md)。既存Matrixと体演算を共有します。

- L-01 [任意modの行列式](determinant_any_mod.md) / [図解](../../visualizations/linear_algebra_extensions/index.html#det_mod)
- L-02 [整数行列の厳密行列式](determinant_integer.md) / [図解](../../visualizations/linear_algebra_extensions/index.html#det_integer)
- L-03 [半環行列](semiring_matrix.md) / [図解](../../visualizations/linear_algebra_extensions/index.html#semiring)
- L-04 [Pfaffian](pfaffian.md) / [図解](../../visualizations/linear_algebra_extensions/index.html#pfaffian)
- L-05 [Permanent](permanent.md) / [図解](../../visualizations/linear_algebra_extensions/index.html#permanent)
- L-06 [Smith標準形・整数／合同線形系](smith_normal_form.md) / [図解](../../visualizations/linear_algebra_extensions/index.html#smith)
- L-07 [Hermite標準形](hermite_normal_form.md) / [図解](../../visualizations/linear_algebra_extensions/index.html#hermite)
- L-08 [GF(2)多項式](gf2_polynomial.md) / [図解](../../visualizations/linear_algebra_extensions/index.html#gf2)
- L-08 [汎用拡大有限体](finite_extension_field.md) / [図解](../../visualizations/linear_algebra_extensions/index.html#extension)
- L-20 [Frobenius標準形](frobenius_normal_form.md) / [図解](../../visualizations/linear_algebra_extensions/index.html#frobenius)
- L-21 [Hafnian](hafnian.md) / [図解](../../visualizations/linear_algebra_extensions/index.html#hafnian)
- L-22 [高速経路付き特性多項式](characteristic_polynomial_fast.md) / [図解](../../visualizations/linear_algebra_extensions/index.html#charpoly)
