# 3.17 多項式・形式的冪級数（FPS）

[ライブラリ一覧](../../README.md) / [全図解](../../visualizations/index.html) / [一次資料・設計判断](references.md)

S・A項目を `fps/` にまとめています。全体の入口は [all.hpp](../../fps/all.hpp)。3.16のNTTを明示的に再利用し、外部ライブラリ・modintの実行依存はありません。C++17の標準ライブラリだけで使えます。

| 優先度 | 説明・usage・API | ヘッダ | 操作できる解説 |
|---|---|---|---|
| S | [多項式の基本演算・微積分・切り詰め](polynomial.md) | [実装](../../fps/polynomial.hpp) | [図解](../../visualizations/fps/index.html#polynomial) |
| A | [FPS逆数 inv](inv.md) | [実装](../../fps/inv.hpp) | [図解](../../visualizations/fps/index.html#inv) |
| A | [FPS対数 log](log.md) | [実装](../../fps/log.hpp) | [図解](../../visualizations/fps/index.html#log) |
| A | [FPS指数 exp](exp.md) | [実装](../../fps/exp.hpp) | [図解](../../visualizations/fps/index.html#exp) |
| A | [FPS非負整数乗 pow](pow.md) | [実装](../../fps/pow.hpp) | [図解](../../visualizations/fps/index.html#pow) |
| A | [FPS平方根 sqrt](sqrt.md) | [実装](../../fps/sqrt.hpp) | [図解](../../visualizations/fps/index.html#sqrt) |
| A | [多項式除算・剰余](division.md) | [実装](../../fps/division.hpp) | [図解](../../visualizations/fps/index.html#division) |
| A | [多点評価](multipoint_evaluation.md) | [実装](../../fps/multipoint_evaluation.hpp) | [図解](../../visualizations/fps/index.html#multipoint) |
| A | [多項式補間](interpolation.md) | [実装](../../fps/interpolation.hpp) | [図解](../../visualizations/fps/index.html#interpolation) |
| A | [Taylor Shift（多項式の平行移動）](taylor_shift.md) | [実装](../../fps/taylor_shift.hpp) | [図解](../../visualizations/fps/index.html#taylor) |
| A | [Bostan–Mori・線形漸化式の第n項](bostan_mori.md) | [実装](../../fps/bostan_mori.hpp) | [図解](../../visualizations/fps/index.html#bostan) |

基本演算の説明には加減乗・微分・積分・係数切り詰めに加え、末尾0除去と一点評価も含みます。[微積分・切り詰め専用の図解](../../visualizations/fps/index.html#calculus)もあります。

## 共通仕様

- `namespace cp`。係数は昇冪順の `vector<int>`。全APIは入力不変で、係数・点を[0,Mod)へ正規化します。INT_MIN/MAXも扱います。標準整数のまま計算する多項式や浮動小数点FPSはこのカテゴリに含めません。
- 関数テンプレートの既定値はMod=998244353、Root=3。微積分・加減算・切り詰め等はModだけを指定します。Modは2以上2×10^9以下の素数。乗算を使うAPIはRootが原始根であることもコンパイル時に検査します。[NTTの仕様](../convolution/ntt.md)と同じです。例: `<17,3>`、`<97,5>`、`<1224736769,3>`。
- `fps_max_size<Mod,Root>()` はconstexprの共通精度上限L。NTT容量Cに対しC/2（C=1のみL=1）です。既定で2^22、法17では8、法97では16。全長の積を計算してから切り詰めるNewton法に必要な作業容量を確保するため、NTT自体の容量より小さくしています。上限超過はlength_errorです。
- inv/log/exp/pow/sqrtは出力項数nを必須で指定し、正確にn項を返します。不足入力は0、n次以上は無視。n=0は代数条件を調べず成功した空列を返します。powの指数0は1、sqrtの不成立はnulloptです。
- 基本演算とTaylor Shiftは末尾0を保持。除算の商・剰余は末尾0を除き、零多項式は空。補間は点数と同じ長さで返します。`fps_truncate` は不足分を0で補い、`polynomial_trim` は長さを縮めます。
- 不正な代数条件はdomain_error、入力長の不一致はinvalid_argument。メモリ確保自体が不可能な場合は通常のbad_alloc等です。負の長さ・指数・項番号を符号なし引数へ渡さないでください。

## 選び方

| 必要な処理 | 使うAPI・注意 |
|---|---|
| a+b / a−b / a*b | polynomial_add / polynomial_subtract / polynomial_multiply |
| a/bの低次n項 | a*fps_inv(b,n)をn項で切る。b₀≠0 |
| 多項式としての商・余り | polynomial_divmod。bが非零ならb₀=0も可 |
| 一点での値 | polynomial_evaluate。Horner法O(n) |
| 多数の任意の点 | multipoint_evaluation。重複点も可 |
| 点と値から係数を復元 | polynomial_interpolation。法で異なる点が必要 |
| f(x+c) | taylor_shift。有限多項式の高次も寄与 |
| P/Qの遠い係数 | bostan_mori。uint64の項番号 |
| 既知の線形漸化式の遠い項 | linear_recurrence_nth。係数順は直前の項から |

## ローカル検証

```sh
python3 verify/run_fps.py
python3 verify/run_fps.py --sanitize
node verify/visualizations/fps_model_test.js
node verify/visualizations/fps_browser_test.mjs --browser /path/to/chromium
```

`--compiler clang++`でコンパイラを変更できます。ASan/UBSanはdebug/releaseで実行します。この開発環境ではサンドボックス内のptrace制約を避けるため、ASan/UBSan/LeakSanitizerを有効にした同じ検証を、承認手順でサンドボックス外実行し成功しています。

14ヘッダ、11使用例、10ジャッジ形式・480ケース、Python任意精度4,352クエリを検証します。逆数の係数漸化式、expの微分方程式、二分累乗、Horner、直接Lagrange、階乗を使わない平行移動、特性多項式の剰余と比較します。小さい系列の全探索、ランダム、65,536項、8,192点、4,096次の漸化式を含みます。詳細は各APIの検証範囲を参照してください。オンラインACは未確認です。

図解は12モード。入力変更・戻る／次へ／再生・境界例に対応します。係数の表、Newton法の補正と確定精度、積木の構築／剰余／補間、Bostan–Moriの偶奇抽出を表示します。図の積と除算は定義式・筆算を使い、C++のNTTと反転による高速化は再現しません。

行列累乗・Kitamasa・Berlekamp–Masseyによる推定との比較は[3.18 線形漸化式・母関数](../linear_recurrence/README.md)にまとめています。Bostan–Moriは両カテゴリの入口から同じ実装を使えます。

## 3.16〜3.20 B項目

- [FPS合成](composition.md) / [操作できる図解](../../visualizations/b_16_20/index.html#composition)
- [FPS合成逆関数](compositional_inverse.md) / [操作できる図解](../../visualizations/b_16_20/index.html#compositional_inverse)
- [Sparse FPS](sparse_fps.md) / [操作できる図解](../../visualizations/b_16_20/index.html#sparse_fps)
- [Online FPS](online_fps.md) / [操作できる図解](../../visualizations/b_16_20/index.html#online_fps)
- [Polynomial GCD](polynomial_gcd.md) / [操作できる図解](../../visualizations/b_16_20/index.html#polynomial_gcd)
- [Polynomial Mod Pow](polynomial_mod_pow.md) / [操作できる図解](../../visualizations/b_16_20/index.html#polynomial_mod_pow)
- [Subproduct Tree](subproduct_tree.md) / [操作できる図解](../../visualizations/b_16_20/index.html#subproduct_tree)
- [Partial Fraction Decomposition](partial_fraction.md) / [操作できる図解](../../visualizations/b_16_20/index.html#partial_fraction)

[全31項目のAPI・条件・比較検証](../b_16_20/README.md)。`python3 verify/run_b_16_20.py` / `--sanitize` / `make verify-b-16-20`。
