# 線形代数・専門的代数 L全11項目

[一次資料](references.md) / [12モードの図解](../../visualizations/linear_algebra_extensions/index.html)

L-01〜L-08・L-20〜L-22の全11項目。L-08はGF(2)多項式と汎用拡大体の2ヘッダに分け、合計12公開ヘッダ・12使用例・12図解を追加しています。既存Matrix・BigInt・体演算・多項式Euclid・BM・Hessenbergの部品を共有します。

| ID | API・説明・使用例 | ヘッダ | 図解 |
|---|---|---|---|
| L-01 | [任意modの行列式](../linear_algebra/determinant_any_mod.md) | [determinant_any_mod.hpp](../../linear_algebra/determinant_any_mod.hpp) | [図解](../../visualizations/linear_algebra_extensions/index.html#det_mod) |
| L-02 | [整数行列の厳密行列式](../linear_algebra/determinant_integer.md) | [determinant_integer.hpp](../../linear_algebra/determinant_integer.hpp) | [図解](../../visualizations/linear_algebra_extensions/index.html#det_integer) |
| L-03 | [半環行列](../linear_algebra/semiring_matrix.md) | [semiring_matrix.hpp](../../linear_algebra/semiring_matrix.hpp) | [図解](../../visualizations/linear_algebra_extensions/index.html#semiring) |
| L-04 | [Pfaffian](../linear_algebra/pfaffian.md) | [pfaffian.hpp](../../linear_algebra/pfaffian.hpp) | [図解](../../visualizations/linear_algebra_extensions/index.html#pfaffian) |
| L-05 | [Permanent](../linear_algebra/permanent.md) | [permanent.hpp](../../linear_algebra/permanent.hpp) | [図解](../../visualizations/linear_algebra_extensions/index.html#permanent) |
| L-06 | [Smith標準形・整数／合同線形系](../linear_algebra/smith_normal_form.md) | [smith_normal_form.hpp](../../linear_algebra/smith_normal_form.hpp) | [図解](../../visualizations/linear_algebra_extensions/index.html#smith) |
| L-07 | [Hermite標準形](../linear_algebra/hermite_normal_form.md) | [hermite_normal_form.hpp](../../linear_algebra/hermite_normal_form.hpp) | [図解](../../visualizations/linear_algebra_extensions/index.html#hermite) |
| L-08 | [GF(2)多項式](../linear_algebra/gf2_polynomial.md) | [gf2_polynomial.hpp](../../linear_algebra/gf2_polynomial.hpp) | [図解](../../visualizations/linear_algebra_extensions/index.html#gf2) |
| L-08 | [汎用拡大有限体](../linear_algebra/finite_extension_field.md) | [finite_extension_field.hpp](../../linear_algebra/finite_extension_field.hpp) | [図解](../../visualizations/linear_algebra_extensions/index.html#extension) |
| L-20 | [Frobenius標準形](../linear_algebra/frobenius_normal_form.md) | [frobenius_normal_form.hpp](../../linear_algebra/frobenius_normal_form.hpp) | [図解](../../visualizations/linear_algebra_extensions/index.html#frobenius) |
| L-21 | [Hafnian](../linear_algebra/hafnian.md) | [hafnian.hpp](../../linear_algebra/hafnian.hpp) | [図解](../../visualizations/linear_algebra_extensions/index.html#hafnian) |
| L-22 | [高速経路付き特性多項式](../linear_algebra/characteristic_polynomial_fast.md) | [characteristic_polynomial_fast.hpp](../../linear_algebra/characteristic_polynomial_fast.hpp) | [図解](../../visualizations/linear_algebra_extensions/index.html#charpoly) |

整数標準形は変換行列まで返し、整数解の格子基底と、一般の正整数法に対する全剰余解の生成元・位数を公開します。Frobenius形は不変因子だけでなく実際の相似基底とその逆も返します。

L-22はcyclic行列の次数nの射影が得られるとき、Strassen積・Krylov倍増でO(n^log₂7 log n)を目標にする追加APIです。固定試行数の失敗時にはHessenberg法に戻り、一般の最悪計算量はO(n³)です。全入力での実測高速化や最先端の一般行列アルゴリズムを意味しません。標準形の素朴Euclid法は途中の係数増大、Permanent/Hafnianは指数時間・領域、拡大体は多項式の基底の指定に注意してください。

## 検証

```sh
make verify-linear-algebra-extensions
python3 -m tools.cp_tools.suite --runners linear_algebra_extensions
python3 -m tools.cp_tools.suite --stdlib-debug --runners linear_algebra_extensions
python3 -m tools.cp_tools.suite --sanitize --runners linear_algebra_extensions
node verify/visualizations/linear_algebra_extensions_model_test.js
node verify/visualizations/linear_algebra_extensions_browser_test.mjs --browser /path/to/chromium
```

- [整数](../../verify/linear_algebra_extensions/integer_test.cpp)：180長方形行列の全小行列式のgcdをSmith不変因子へ照合。UAV=D、H=UA、unimodular性、Hermiteの一意性、整数解の格子基底。360合同系の全3,423解を列挙し、生成元の直積が過不足・重複なしで一致。160行列式の順列和、uint64最大法・負係数・50桁以上の整数も確認します。
- [有限体](../../verify/linear_algebra_extensions/finite_test.cpp)：16,384組のGF(2)多項式の積・除算・Bézout係数、510既約性判定を試し割りと照合。64bitを超える積、F8/F9/F25等の全乗算表、逆元、Frobenius則を確認。F2の全3×3行列・F3の全2×2行列を含む673行列で相似変換・companion形・因子の割切りを検証します。
- [行列演算](../../verify/linear_algebra_extensions/matrix_test.cpp)：512個のPermanentを全単射で、100組のPfaffian/Hafnianを完全マッチングで比較。180半環行列を辺数別DP、独自非可換半環を文字列の結合で比較。300小標数行列と256×256の高速特性多項式、非cyclic入力のfallbackを確認します。
- 全12ヘッダの単独・重複include、全カテゴリのall.hpp混在、2翻訳単位、12使用例を確認。Python有理数消去による48行列式と、集合DPによる24特性多項式を独立比較します。1つのLibrary Checker形式ラッパはローカル検証です。
- 図解は12の実C++参照値、独立した順列・全解・直接積のモデル検証、入力編集・前後移動・再生・エラー復帰・390pxのブラウザ検証を用意しています。

実行記録の保存先と集計方法は[検証台帳の使い方](../development_records/verification.md)を参照してください。
