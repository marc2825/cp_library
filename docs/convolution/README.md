# 3.16 畳み込み・高速変換

C++17・`namespace cp`。[all.hpp](../../convolution/all.hpp)でまとめて、個別ヘッダでも使えます。

| 用途 | 実装・説明 | 条件・結果 |
|---|---|---|
| 有限体で変換 | [ntt / ntt_max_size](ntt.md) | 既定998244353、素数と原始根を検査。2冪長 |
| 通常のmod畳み込み | [convolution](convolution.md) | 整数入力を正規化し、NTTで正確に計算 |
| 通常の整数畳み込み | [convolution_ll](convolution_ll.md) | 3法CRT、64ビット整数。絶対値の十分条件を検査 |
| 実数・複素数のFFT | [fft / fft_real](fft.md) | float/double/long double、全スペクトル、近似値 |
| 実数・複素数の畳み込み | [convolution_fft](convolution_fft.md) | 自動0埋め、実数または複素数の近似値 |

[操作できる図解：6モード](../../visualizations/convolution/index.html) / [使用例](../../examples/convolution/) / [一次資料・設計判断](references.md)

通常の畳み込みは **c[k]=Σ_(i+j=k) a[i]b[j]** で、係数は定数項から昇冪順です。空を含む場合は空、それ以外は|a|+|b|−1個を返し、末尾の0も残します。畳み込み関数は自動0埋め、変換関数は2冪の長さを利用者が用意します。

NTTとFFTはどちらも順変換に正の指数、逆変換に負の指数と1/nを使い、自然順→自然順です。FFTを他ライブラリと比較するときは符号と正規化を確認してください。

mod版はNTTに適した素数が対象です。任意法への高速CRT版は[convolution_mod](convolution_mod.md)を使います。整数版は保守的な絶対値上限を満たす場合に正確な値を返し、打ち消しで上限を超える入力も明示的に拒否します。実数・複素数版は近似値なので、整数の正確さが必要な場合と使い分けます。

3.14の[約数・倍数による畳み込み](../number_theory_convolution/README.md)とは添字の集計規則が異なり、別カテゴリです。既存カテゴリと3.15の変更は保持しています。標準C++17のみで、外部ライブラリや__int128に依存しません。

## 検証

```sh
python3 verify/run_convolution.py
python3 verify/run_convolution.py --sanitize
node verify/visualizations/convolution_model_test.js
node verify/visualizations/convolution_browser_test.mjs --browser /path/to/chromium
```

`make verify-convolution`、全図解モデルは`make verify-visualizations`。Python/npmの追加パッケージは不要です。

7ヘッダの単独／逆順／混在includeと複数翻訳単位、5使用例、5ジャッジ形式・360ケースを検証します。14,641組の三値配列、8組の法／根、直接の有限体DFT・複素DFT、400ランダム整数積・300複素数積、Python任意精度858組・2,838出力と比較します。空・1要素・2冪境界・符号付き／符号なし極値・CRTの符号・保守的上限・NaN/Inf・非有限の途中値も含みます。

2^20点NTT、10万×10万の整数／mod積、65,537×65,537の実数FFTも確認します。doubleの大きい実数例では全出力の絶対誤差を測定しますが、この試験の誤差を一般の保証値とはしません。

[検証入口](../../verify/run_convolution.py)：通常／ASan・UBSan、debug/release。オンラインACは未確認です。

## 高度なアルゴリズムの追加

- [任意mod畳み込み](convolution_mod.md) / [操作できる図解](../../visualizations/advanced/index.html#crt)

追加分の検証は `python3 verify/run_advanced.py`（`--sanitize`対応）。[全追加項目と既存項目の対応表](../advanced/README.md)。

## 3.16〜3.20 A項目の補完

- [Karatsuba畳み込み](karatsuba.md) / [操作できる図解](../../visualizations/transform_a/index.html#karatsuba)
- [FWHT / Walsh–Hadamard変換](fwht.md) / [操作できる図解](../../visualizations/transform_a/index.html#fwht)
- [AND / OR / XOR Convolution](bitwise_convolution.md) / [操作できる図解](../../visualizations/transform_a/index.html#bitwise)

[追加分のAPI条件・使用例・検証範囲](../transform_a/README.md)。`python3 verify/run_transform_a.py` / `--sanitize` / `make verify-transform-a`。

## 3.16〜3.20 B項目

- [Relaxed / Online Convolution](relaxed_convolution.md) / [操作できる図解](../../visualizations/b_16_20/index.html#relaxed_convolution)
- [GCD / LCM Convolution（共通入口）](gcd_lcm_convolution.md) / [操作できる図解](../../visualizations/b_16_20/index.html#gcd_lcm_convolution)
- [Min-Plus Convolution（共通入口）](min_plus_convolution.md) / [操作できる図解](../../visualizations/b_16_20/index.html#min_plus_convolution)
- [多次元畳み込み](multidimensional_convolution.md) / [操作できる図解](../../visualizations/b_16_20/index.html#multidimensional_convolution)
- [巨大配列の畳み込み](convolution_large.md) / [操作できる図解](../../visualizations/b_16_20/index.html#convolution_large)
- [Middle Product](middle_product.md) / [操作できる図解](../../visualizations/b_16_20/index.html#middle_product)
- [Chirp-Z Transform](chirp_z.md) / [操作できる図解](../../visualizations/b_16_20/index.html#chirp_z)

[全31項目のAPI・条件・比較検証](../b_16_20/README.md)。`python3 verify/run_b_16_20.py` / `--sanitize` / `make verify-b-16-20`。

## Q追加項目（2026-10-26）

[Q全31項目と検証](../polynomial_extensions/README.md)。既存APIを保持し、以下を追加しました。

- Q-04 [mod 2^64 畳み込み](convolution_u64.md) / [図解](../../visualizations/polynomial_extensions/index.html#wrap_convolution)
- Q-05 [GF(2^64) 畳み込み](convolution_nimber.md) / [図解](../../visualizations/polynomial_extensions/index.html#nimber_convolution)
- Q-06 [転置原理・固定畳み込みの共有前計算](transposed_convolution.md) / [図解](../../visualizations/polynomial_extensions/index.html#transpose)
- Q-07 [NTT Doubling](ntt_doubling.md) / [図解](../../visualizations/polynomial_extensions/index.html#doubling)
- Q-20 [Rader FFT](rader_fft.md) / [図解](../../visualizations/polynomial_extensions/index.html#rader)
- Q-21 [Bluestein FFT](bluestein_fft.md) / [図解](../../visualizations/polynomial_extensions/index.html#bluestein)
- Q-22 [Goldilocks 64bit NTT](ntt64.md) / [図解](../../visualizations/polynomial_extensions/index.html#ntt64)
- Q-23 [AVX2 NTT](ntt_avx2.md) / [図解](../../visualizations/polynomial_extensions/index.html#avx2)
- Q-25 [多変数巡回畳み込み](multidimensional_cyclic_convolution.md) / [図解](../../visualizations/polynomial_extensions/index.html#cyclic_tensor)
- Q-26 [乗法剰余添字の畳み込み](multiplicative_convolution.md) / [図解](../../visualizations/polynomial_extensions/index.html#multiplicative)
- Q-35 [高速中間積](fast_middle_product.md) / [図解](../../visualizations/polynomial_extensions/index.html#middle)
