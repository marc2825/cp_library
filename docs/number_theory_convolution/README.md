# 3.14 数論：乗法的関数・高度な整数問題

今回のA項目は、正整数の添字を持つ算術関数の変換と畳み込みです。入力の係数列が乗法的関数である必要はありません。C++17・`namespace cp`、[all.hpp](../../number_theory_convolution/all.hpp)からまとめて使えます。

| 欲しい値 | 実装・API・usage | 時間 |
|---|---|---|
| 約数からの集計 Σ_{d∣n} f[d] | [約数ゼータ変換](divisor_zeta_transform.md) | O(N log log N) |
| 約数和から元の値へ | [約数メビウス変換](divisor_mobius_transform.md) | O(N log log N) |
| 倍数からの集計 Σ_{n∣m} f[m] | [倍数ゼータ変換](multiple_zeta_transform.md) | O(N log log N) |
| 倍数和から元の値へ | [倍数メビウス変換](multiple_mobius_transform.md) | O(N log log N) |
| 積ij=nに集計 | [Dirichlet Convolution](dirichlet_convolution.md) | O(N log N) |
| GCD(i,j)=nに集計 | [GCD Convolution](gcd_convolution.md) | O(N log log N) |
| LCM(i,j)=nに集計 | [LCM Convolution](lcm_convolution.md) | O(N log log N) |

[操作できる図解：7モード](../../visualizations/number_theory_convolution/index.html) / [動く使用例](../../examples/number_theory_convolution/) / [一次資料・設計判断](references.md)

正整数1〜Nに対して長さN+1の `vector<T>` を用意します。変換はin-placeで添字0を保持し、畳み込みは入力の添字0を無視して結果の0をゼロにします。空も扱えます。畳み込みの出力長は両入力の最大長、短い入力は0で補い、DirichletとLCMで上限を超えた組は捨てます。GCDは大きい入力の添字が小さい結果にも寄与するため、入力を先に切らないでください。

係数型は整数やmod型等を使います。割り算・逆元・素数法の条件はありません。乗算順は `a[i]*b[j]` で、加算が可換な群・乗算が分配的なら行列等も扱えます。zetaだけなら加算だけ、mobiusだけなら減算だけのAPI条件です。標準整数の全ての途中値は表現可能である必要があり、超過は例外です。符号なしの折り返しも行いません。独自型の算術条件は利用者が管理します。

素数列は [3.13のPrimeSieve](../number_theory_basic/prime_sieve.md) を再利用します。既存の数論・部分集合の変換とは別のAPIです。整数のμやφの表を構築する機能は今回の項目には含めず、φの例は約数メビウス変換で計算します。

## 検証

```sh
python3 verify/run_number_theory_convolution.py
python3 verify/run_number_theory_convolution.py --sanitize
node verify/visualizations/number_theory_convolution_model_test.js
node verify/visualizations/number_theory_convolution_browser_test.mjs --browser /path/to/chromium
```

`make verify-number-theory-convolution`、全カテゴリの図解モデルは `make verify-visualizations`。追加のPython/npmパッケージは不要です。

10ヘッダの単独・逆順・混在includeと複数翻訳単位、7使用例、3ジャッジ形式・288ケース。三値列、全添字対、直接の約数／倍数和・μ、符号付き・mod・合成数法・行列・整数極値を照合します。100万上限で全出力と更新回数も確認します。オンラインACは未確認です。

## 高度なアルゴリズムの追加

- [64bit Miller–Rabin](miller_rabin.md) / [操作できる図解](../../visualizations/advanced/index.html#primality)
- [64bit Pollard’s Rho](pollard_rho.md) / [操作できる図解](../../visualizations/advanced/index.html#rho)

追加分の検証は `python3 verify/run_advanced.py`（`--sanitize`対応）。[全追加項目と既存項目の対応表](../advanced/README.md)。

3.14のA全5項目は既存実装を共有します。[3.12〜3.15の補完](../math_a/README.md)ではφ/μ・CRTなどを3.13へ追加し、Miller–Rabin/Pollard–Rhoを再利用。今回も既存カテゴリ検証を回帰実行します。

## B項目（2026-10-18）

[30項目の対応範囲・比較検証](../b_12_15/README.md) / [一次資料と設計判断](../b_12_15/references.md)。`make verify-b-12-15`でまとめて検証します。

| API | 説明・条件 | 操作できる図解 |
|---|---|---|
| [divisor_count_sum.hpp](../../number_theory_convolution/divisor_count_sum.hpp) | [約数個数和](divisor_count_sum.md) | [図解](../../visualizations/b_12_15/index.html#divisor_count_sum) |
| [summatory_totient.hpp](../../number_theory_convolution/summatory_totient.hpp) | [Euler φの総和](summatory_totient.md) | [図解](../../visualizations/b_12_15/index.html#summatory_totient) |
| [mertens.hpp](../../number_theory_convolution/mertens.hpp) | [Mertens Function](mertens.md) | [図解](../../visualizations/b_12_15/index.html#mertens) |
| [prime_counting.hpp](../../number_theory_convolution/prime_counting.hpp) | [素数個数計算（Lucy DP）](prime_counting.md) | [図解](../../visualizations/b_12_15/index.html#prime_counting) |
| [multiplicative_function_sum.hpp](../../number_theory_convolution/multiplicative_function_sum.hpp) | [乗法的関数の総和](multiplicative_function_sum.md) | [図解](../../visualizations/b_12_15/index.html#multiplicative_function_sum) |
| [square_free_counting.hpp](../../number_theory_convolution/square_free_counting.hpp) | [Square-Free Counting](square_free_counting.md) | [図解](../../visualizations/b_12_15/index.html#square_free_counting) |

## N追加項目（2026-10-26）

[N全19項目の条件・検証](../number_theory_extensions/README.md) / [19モードの図解](../../visualizations/number_theory_extensions/index.html)。`make verify-number-theory-extensions`。

- [商prefix表のDirichlet積](dirichlet_prefix_product.md)（N-09）
- [Dirichlet逆元](dirichlet_inverse.md)（N-10）
- [高速素数和](prime_sum.md)（N-25）
- [一般化Min_25 Sieve](min25_sieve.md)（N-26）
- [任意乗法的関数の大規模総和](multiplicative_summatory.md)（N-27）
