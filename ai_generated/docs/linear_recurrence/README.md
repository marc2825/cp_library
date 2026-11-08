# 3.18 線形漸化式・母関数

[ライブラリ一覧](../../README.md) / [全図解](../../visualizations/index.html) / [一次資料・設計判断](references.md)

A項目の行列累乗、Kitamasa、Berlekamp–Massey、Bostan–Moriを `linear_recurrence/` にまとめています。[all.hpp](../../linear_recurrence/all.hpp)が入口です。Bostan–Moriは[3.17 FPS](../fps/README.md)の実装を再利用し、どちらの入口からも同じAPIを使えます。

| 手法 | API・usage | 時間 | 条件 | 図解 |
|---|---|---|---|---|
| 行列累乗 | [linear_recurrence_matrix / matrix_power_mod / matrix_power_apply](matrix_recurrence.md) | O(d³ log(n+1)+d²) | 合成数法・法1も可、一般の状態遷移も可 | [行列と状態](../../visualizations/linear_recurrence/index.html#matrix) |
| Kitamasa | [kitamasa / kitamasa_coefficients](kitamasa.md) | O(d² log(n+1)+d) | 合成数法・法1も可 | [多項式の還元と初期項の重み](../../visualizations/linear_recurrence/index.html#kitamasa) |
| Berlekamp–Massey | [berlekamp_massey / linear_recurrence_from_prefix](berlekamp_massey.md) | O(S²+S log Mod)、延長費用は別 | 素数法、有限観測の最短規則 | [不一致と次数の更新](../../visualizations/linear_recurrence/index.html#berlekamp) |
| Bostan–Mori | [bostan_mori / linear_recurrence_nth](bostan_mori.md) | O(M(d) log(n+1)+log Mod) | NTT素数・原始根・作業容量 | [母関数と偶奇抽出](../../visualizations/linear_recurrence/index.html#bostan) |

## 共通仕様

- C++17、namespace cp、vector<int>、入力不変。既定法998244353。全入力を法で正規化し、係数の負数・INT_MIN/MAXを扱います。内部の乗算はlong longの範囲内、和はその都度剰余を取ります。
- 既知の漸化式は `a[t]=Σ_(j=0..d−1)c[j]*a[t−1−j]` (t≥d)。initialはa[0]から昇順、c[0]は直前の項です。全手法で同じ順序です。既知の漸化式のinitialとcは同じ非空長が必要です。
- 項番号／回数は0始まりのuint64_tで、最大値2^64−1も扱います。負数を符号なし引数へ渡さないでください。n<dの既知漸化式は初期項だけを正規化して返します。
- 行列累乗・Kitamasaの法は1〜2×10^9。逆元を使わず、合成数法・法1に対応。BMは2〜2×10^9の素数、Bostan–Moriはさらに原始根とNTT作業容量を要求します。不適切な法はコンパイル時に拒否します。
- BMの戻り値は最短の**長さ**L。末尾0も保持します。空・全0観測の次数0と、{1,0,0,…}の次数1を区別します。観測の先頭2d項から将来を再現するには、真の無限列が次数d以下の線形漸化式に従う前提が必要です。
- 形や長さの違反はinvalid_argument、Bostan–Moriの零分母定数項はdomain_error、作業長の超過はlength_error。メモリ確保そのものの失敗は通常のbad_alloc等です。

## 母関数との対応

A(x)=Σa[t]x^t、Q(x)=1−c[0]x−…−c[d−1]x^dとすると、

```text
P(x) = (initial(x) * Q(x)) mod x^d
A(x) = P(x)/Q(x)
```

同じ規則を状態遷移として扱うのが行列、X^n mod Fから初期項の重みを得るのがKitamasa、観測から最短の規則を求めるのがBM、P/Qから遠い係数を直接抽出するのがBostan–Moriです。BMから延長する補助APIはKitamasaを使うため、法1000000007でも使えます。

## ローカル検証

```sh
python3 verify/run_linear_recurrence.py
python3 verify/run_linear_recurrence.py --sanitize
node verify/visualizations/linear_recurrence_model_test.js
node verify/visualizations/linear_recurrence_browser_test.mjs --browser /path/to/chromium
```

`--compiler clang++`で切り替えられます。6ヘッダ・4使用例・5ジャッジ形式・320ケース、Python任意精度3,504クエリ。BMの9,233列を連立方程式で、600漸化式と160任意行列を逐次計算で比較します。各Kitamasa重みは初期値を単位ベクトルにした独立の数列で照合します。大きな次数、uint64最大値、法1・合成数・非NTT素数、空・遅い不一致・末尾0・整数極値を含みます。オンラインACは未確認です。

図解は4モードで、同じ漸化式を切り替えて理解できます。初期値・係数・法・項番号の変更、任意の小行列、観測列、直接P/Qに対応。3,446入力・55,104スナップショット、15拒否条件を独立に検証します。旧matrixヘッダと3.17 APIの互換性も確認します。

一般の長方形Matrix、行列積・累乗、Gauss–Jordan・XOR基底は[3.20 線形代数](../linear_algebra/README.md)を参照してください。3.18の既存APIはそのまま使用できます。

## 3.16〜3.20 B項目

- [線形漸化式の推定と第N項](infer_nth.md) / [操作できる図解](../../visualizations/b_16_20/index.html#infer_nth)
- [P-recursive数列](p_recursive.md) / [操作できる図解](../../visualizations/b_16_20/index.html#p_recursive)
- [指数型母関数（EGF）の演算](egf.md) / [操作できる図解](../../visualizations/b_16_20/index.html#egf)
- [母関数の係数抽出](coefficient_extraction.md) / [操作できる図解](../../visualizations/b_16_20/index.html#coefficient_extraction)
- [有理型母関数](rational_generating_function.md) / [操作できる図解](../../visualizations/b_16_20/index.html#rational_generating_function)
- [等比数列の多項式重み付き和](polynomial_geometric_sum.md) / [操作できる図解](../../visualizations/b_16_20/index.html#polynomial_geometric_sum)

[全31項目のAPI・条件・比較検証](../b_16_20/README.md)。`python3 verify/run_b_16_20.py` / `--sanitize` / `make verify-b-16-20`。

## Q追加項目（2026-10-26）

[Q全31項目と検証](../polynomial_extensions/README.md)。既存APIを保持し、以下を追加しました。

- Q-36 [Holonomic数列の推定と大きな添字](holonomic.md) / [図解](../../visualizations/polynomial_extensions/index.html#holonomic)
- Q-37 [Reeds–Sloane](reeds_sloane.md) / [図解](../../visualizations/polynomial_extensions/index.html#reeds_sloane)
- Q-38 [SET・SEQ・CYCの母関数合成](combinatorial_composition.md) / [図解](../../visualizations/polynomial_extensions/index.html#labelled)
- Q-39 [線形漸化式の連続大量項](recurrence_slice.md) / [図解](../../visualizations/polynomial_extensions/index.html#recurrence_slice)
