# 3.16〜3.20 A項目の補完

3.16〜3.20の不足9掲載箇所（8系統）を補完しました。対象のAは全29掲載箇所が実装済みです。3.17・3.18の既存APIは共有し、3.16と3.19の集合畳み込みも共通実装です。10公開ヘッダ・10個別説明・10使用例を用意しています。

[実装予定リスト](../implementation_plan.md#3-16) / [操作できる8モード](../../visualizations/transform_a/index.html) / [一次資料](references.md)

| 項目 | 公開APIと内容 |
|---|---|
| [Karatsuba](../convolution/karatsuba.md) | convolution_karatsuba、除算不要の環上の通常の積 |
| [FWHT](../set_function/fwht.md) | fwht、自然順の変換と各段で正規化する逆変換。[3.16共通入口](../convolution/fwht.md) |
| [AND/OR/XOR Convolution](../set_function/bitwise_convolution.md) | bitwise_and/or/xor_convolution、同じ正の2冪長。[3.16共通入口](../convolution/bitwise_convolution.md) |
| [Ranked Zeta](../set_function/ranked_zeta_transform.md) | ranked_zeta_transform・ranked_mobius_transform。Subset Convolutionからも利用 |
| [逆行列](../linear_algebra/inverse.md) | matrix_inverse / matrix_inverse_mod、特異ならnullopt |
| [GF(2) Bitset Gauss](../linear_algebra/gf2_gauss.md) | gf2_gauss_jordan・gf2_rank、特解・零空間・簡約行列 |
| [線形空間の交差](../linear_algebra/space_intersection.md) | linear_space_intersection / _modとxor_space_intersection |
| [特性多項式](../linear_algebra/characteristic_polynomial.md) | characteristic_polynomial / _mod、Hessenberg相似変形、昇冪順det(xI−A) |

## 条件と設計

- Karatsubaは一般の環で使え、標準整数の中間超過は例外。短い側32以下は直接積です。空入力なら空を返します。
- 集合変換と集合畳み込みは同じ正の2冪長を要求し、自動paddingは行いません。AND/ORは除算不要、XORの独自mod型は2の逆元が必要。整数の逆FWHTは各段の割り切れを検査します。標準整数の途中超過も拒否します。
- Ranked Zetaは[rank][mask]。逆は全rank層へメビウスを適用し、一般のrank表にも使用できます。既存Subset Convolutionの内部変換をこの公開APIへ置き換えました。
- 線形代数の汎用版は体型、mod版は既存の素数法補助を共有します。逆行列と交差の実数版は既存Gauss–Jordanと同じepsilon規約。実数の厳密な階数や悪条件での精度は保証しません。
- GF(2)のbitset容量はコンパイル時指定、実際の変数数は別に指定できます。RHSの位置はその変数数、未使用high bitは0。XOR空間の交差は既存XorBasisを使用します。
- 特性多項式は1〜nの除算を使わず、Mod<=nでも計算できます。合成数法の一般版は含めません。

## 検証

```sh
python3 verify/run_transform_a.py
python3 verify/run_transform_a.py --sanitize
make verify-transform-a
node verify/visualizations/transform_a_model_test.js
node verify/visualizations/transform_a_browser_test.mjs --browser /path/to/chromium
```

10入口の単独・重複include（debug/release）、全カテゴリ混在の複数翻訳単位、10使用例と説明中usageの完全一致、8ジャッジ形式各24ケースを検証します。

C++では300組の通常積、再帰乗算回数、除算のないmod12環、直接Hadamard和と逆、3種bitwiseの二重ループ、rank逆変換、Subset Convolutionの再利用、65,536項のXOR単位元、129ビットのGF(2)独立行・全自由変数、実数・複素数、空次元・不正形状・整数超過を確認します。

Python任意精度の7,336ケースをdebug/releaseの両方で比較します。通常積・Hadamardの符号付き和・bitwiseとrankの定義、素数2/3/5/998244353での置換和による行列式と多項式、逆行列の左右の積、素数2/3の生成空間全列挙、3変数3方程式の全4,096系と追加系の全解、64ビットXOR空間の列挙を使います。オンラインACは未確認です。

図解モデルは1,924入力・31,226状態・15拒否条件を独立に検証します。rank各段、[A|I]の行同値性、相似変形後の多項式不変性、主小行列の多項式、全生成空間と全二値解を含みます。[教材モデルとの差](../../visualizations/transform_a/README.md)も明記しています。

回帰検証は既存の `run_convolution.py`、`run_fps.py`、`run_linear_recurrence.py`、`run_set_function.py`、`run_linear_algebra.py`、`run_advanced.py` を使用します。サニタイザとChromiumは環境のptrace/socket制限により、制限外実行が必要になる場合があります。

検証結果（2026-10-17）：上記の通常・ASan/UBSan/LeakSanitizer（debug/release）、6カテゴリの既存回帰、全図解モデル、ブラウザ8モードと390px表示が成功しました。関連ローカルリンク1,001件に欠落なし。デスクトップ・モバイルの画像も確認しました。
