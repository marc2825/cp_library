# 畳み込み・多項式・母関数 Q全31項目

[統合一覧](../remaining_work.md) / [実装計画](../implementation_plan.md) / [一次資料](references.md) / [31モードの図解](../../visualizations/polynomial_extensions/index.html)

Q-01〜Q-09、Q-20〜Q-31、Q-33〜Q-42の31項目。Q-10〜Q-19とQ-32は欠番です。実装は既存の `convolution/`、`fps/`、`linear_recurrence/`、`set_function/` に置き、通常NTT・任意mod畳み込み・FPS・Kitamasa・集合畳み込みを共有します。

Q-30は一次分数置換と逆変換、Q-38はラベル付きSET/SEQ/CYC、Q-42は交差サイズ別の和集合畳み込みに具体化しました。Q-36は次数・order上限付き推定とblock法による大引数計算です。任意の「特殊変換」や全てのHolonomic閉包演算を意味しません。

| ID | 説明・API・使用例 | ヘッダ | 図解 |
|---|---|---|---|
| Q-01 | [多項式拡張GCD・剰余逆元](../fps/polynomial_extended_gcd.md) | [fps/polynomial_extended_gcd.hpp](../../fps/polynomial_extended_gcd.hpp) | [図解](../../visualizations/polynomial_extensions/index.html#extended_gcd) |
| Q-02 | [終結式・判別式](../fps/polynomial_resultant.md) | [fps/polynomial_resultant.hpp](../../fps/polynomial_resultant.hpp) | [図解](../../visualizations/polynomial_extensions/index.html#resultant) |
| Q-03 | [有限体上の多項式因数分解](../fps/polynomial_factorization.md) | [fps/polynomial_factorization.hpp](../../fps/polynomial_factorization.hpp) | [図解](../../visualizations/polynomial_extensions/index.html#factorization) |
| Q-04 | [mod 2^64 畳み込み](../convolution/convolution_u64.md) | [convolution/convolution_u64.hpp](../../convolution/convolution_u64.hpp) | [図解](../../visualizations/polynomial_extensions/index.html#wrap_convolution) |
| Q-05 | [GF(2^64) 畳み込み](../convolution/convolution_nimber.md) | [convolution/convolution_nimber.hpp](../../convolution/convolution_nimber.hpp) | [図解](../../visualizations/polynomial_extensions/index.html#nimber_convolution) |
| Q-06 | [転置原理・固定畳み込みの共有前計算](../convolution/transposed_convolution.md) | [convolution/transposed_convolution.hpp](../../convolution/transposed_convolution.hpp) | [図解](../../visualizations/polynomial_extensions/index.html#transpose) |
| Q-07 | [NTT Doubling](../convolution/ntt_doubling.md) | [convolution/ntt_doubling.hpp](../../convolution/ntt_doubling.hpp) | [図解](../../visualizations/polynomial_extensions/index.html#doubling) |
| Q-08 | [Sharp P Subset Sum](../fps/sharp_p_subset_sum.md) | [fps/sharp_p_subset_sum.hpp](../../fps/sharp_p_subset_sum.hpp) | [図解](../../visualizations/polynomial_extensions/index.html#subset_sum) |
| Q-09 | [多項式の不定和分](../fps/polynomial_prefix_sum.md) | [fps/polynomial_prefix_sum.hpp](../../fps/polynomial_prefix_sum.hpp) | [図解](../../visualizations/polynomial_extensions/index.html#prefix_sum) |
| Q-20 | [Rader FFT](../convolution/rader_fft.md) | [convolution/rader_fft.hpp](../../convolution/rader_fft.hpp) | [図解](../../visualizations/polynomial_extensions/index.html#rader) |
| Q-21 | [Bluestein FFT](../convolution/bluestein_fft.md) | [convolution/bluestein_fft.hpp](../../convolution/bluestein_fft.hpp) | [図解](../../visualizations/polynomial_extensions/index.html#bluestein) |
| Q-22 | [Goldilocks 64bit NTT](../convolution/ntt64.md) | [convolution/ntt64.hpp](../../convolution/ntt64.hpp) | [図解](../../visualizations/polynomial_extensions/index.html#ntt64) |
| Q-23 | [AVX2 NTT](../convolution/ntt_avx2.md) | [convolution/ntt_avx2.hpp](../../convolution/ntt_avx2.hpp) | [図解](../../visualizations/polynomial_extensions/index.html#avx2) |
| Q-25 | [多変数巡回畳み込み](../convolution/multidimensional_cyclic_convolution.md) | [convolution/multidimensional_cyclic_convolution.hpp](../../convolution/multidimensional_cyclic_convolution.hpp) | [図解](../../visualizations/polynomial_extensions/index.html#cyclic_tensor) |
| Q-26 | [乗法剰余添字の畳み込み](../convolution/multiplicative_convolution.md) | [convolution/multiplicative_convolution.hpp](../../convolution/multiplicative_convolution.hpp) | [図解](../../visualizations/polynomial_extensions/index.html#multiplicative) |
| Q-27 | [Power Projection](../fps/power_projection.md) | [fps/power_projection.hpp](../../fps/power_projection.hpp) | [図解](../../visualizations/polynomial_extensions/index.html#projection) |
| Q-28 | [有限体の根の列挙](../fps/polynomial_roots.md) | [fps/polynomial_roots.hpp](../../fps/polynomial_roots.hpp) | [図解](../../visualizations/polynomial_extensions/index.html#roots) |
| Q-29 | [高速FPS合成](../fps/fast_composition.md) | [fps/fast_composition.hpp](../../fps/fast_composition.hpp) | [図解](../../visualizations/polynomial_extensions/index.html#composition) |
| Q-30 | [FPSの一次分数変数変換](../fps/fractional_linear_transform.md) | [fps/fractional_linear_transform.hpp](../../fps/fractional_linear_transform.hpp) | [図解](../../visualizations/polynomial_extensions/index.html#fractional) |
| Q-31 | [多変数FPS](../fps/multivariate_fps.md) | [fps/multivariate_fps.hpp](../../fps/multivariate_fps.hpp) | [図解](../../visualizations/polynomial_extensions/index.html#multivariate) |
| Q-33 | [FPSの微分方程式](../fps/differential_equation.md) | [fps/differential_equation.hpp](../../fps/differential_equation.hpp) | [図解](../../visualizations/polynomial_extensions/index.html#ode) |
| Q-24 | [多項式行列式・評価行列Prefix Product](../fps/polynomial_matrix.md) | [fps/polynomial_matrix.hpp](../../fps/polynomial_matrix.hpp) | [図解](../../visualizations/polynomial_extensions/index.html#polynomial_matrix) |
| Q-34 | [Composed Sum / Product](../fps/composed_polynomial.md) | [fps/composed_polynomial.hpp](../../fps/composed_polynomial.hpp) | [図解](../../visualizations/polynomial_extensions/index.html#composed) |
| Q-35 | [高速中間積](../convolution/fast_middle_product.md) | [convolution/fast_middle_product.hpp](../../convolution/fast_middle_product.hpp) | [図解](../../visualizations/polynomial_extensions/index.html#middle) |
| Q-36 | [Holonomic数列の推定と大きな添字](../linear_recurrence/holonomic.md) | [linear_recurrence/holonomic.hpp](../../linear_recurrence/holonomic.hpp) | [図解](../../visualizations/polynomial_extensions/index.html#holonomic) |
| Q-37 | [Reeds–Sloane](../linear_recurrence/reeds_sloane.md) | [linear_recurrence/reeds_sloane.hpp](../../linear_recurrence/reeds_sloane.hpp) | [図解](../../visualizations/polynomial_extensions/index.html#reeds_sloane) |
| Q-38 | [SET・SEQ・CYCの母関数合成](../linear_recurrence/combinatorial_composition.md) | [linear_recurrence/combinatorial_composition.hpp](../../linear_recurrence/combinatorial_composition.hpp) | [図解](../../visualizations/polynomial_extensions/index.html#labelled) |
| Q-39 | [線形漸化式の連続大量項](../linear_recurrence/recurrence_slice.md) | [linear_recurrence/recurrence_slice.hpp](../../linear_recurrence/recurrence_slice.hpp) | [図解](../../visualizations/polynomial_extensions/index.html#recurrence_slice) |
| Q-40 | [SPS Power Projection](../set_function/sps_power_projection.md) | [set_function/sps_power_projection.hpp](../../set_function/sps_power_projection.hpp) | [図解](../../visualizations/polynomial_extensions/index.html#sps_projection) |
| Q-41 | [オンライン集合関数畳み込み](../set_function/online_subset_convolution.md) | [set_function/online_subset_convolution.hpp](../../set_function/online_subset_convolution.hpp) | [図解](../../visualizations/polynomial_extensions/index.html#online_subset) |
| Q-42 | [交差サイズ別の集合畳み込み](../set_function/intersection_convolution.md) | [set_function/intersection_convolution.hpp](../../set_function/intersection_convolution.hpp) | [図解](../../visualizations/polynomial_extensions/index.html#intersection) |

高速合成はO(n log² n)、専用中間積は長さceil_pow2(|a|)の巡回積を使用します。GF(2^64)はConway nimber＋Karatsuba、64bit NTTはGoldilocks素数専用、AVX2は実行時dispatch付きです。有限体因数分解は素朴な剰余演算を使うLas Vegas法で、最先端の因数分解計算量を主張していません。多変数FPSのパディング体積、Holonomicのblock予算、係数法と添字法の違いを各説明に明記しています。

## 検証

```sh
make verify-polynomial-extensions
python3 -m tools.cp_tools.suite --runners polynomial_extensions
python3 -m tools.cp_tools.suite --stdlib-debug --runners polynomial_extensions
python3 -m tools.cp_tools.suite --sanitize --runners polynomial_extensions
node verify/visualizations/polynomial_extensions_model_test.js
node verify/visualizations/polynomial_extensions_browser_test.mjs --browser /path/to/chromium
```

- [代数](../../verify/polynomial_extensions/algebra_test.cpp): F2/F3/F5の1,656多項式を全列挙し、因子の積・試し割り既約性・全根を照合。Bézout恒等式、Sylvester行列式、離散差分、根の対からのcomposed演算、行列式の順列和と直接prefix積を比較します。
- [変換](../../verify/polynomial_extensions/transform_test.cpp): Goldilocks演算10万組とuint64境界を128bit剰余に照合。2^15までのAVX2/scalar/倍増/逆変換、直接DFT、full-width畳み込みと1万項wraparound、nimber、転置の内積、巡回／乗法添字を確認します。
- [FPS](../../verify/polynomial_extensions/fps_test.cpp): 220組の通常合成・Power Projection、部分和の全mask列挙、矩形商環の直接積と幾何級数、ODEの残差と初期値、8,192項の高速合成を別のFPS冪と照合します。
- [漸化式・集合](../../verify/polynomial_extensions/sequence_set_test.cpp): 法4/6/8/9の12,977列を全探索しReeds–Sloaneの最小次数を検査。階乗10万番目、巨大開始位置からの1万項、推定式の未使用項への適合、出力依存のonline入力、交差サイズを直接列挙で確認。12変数の全online出力も確認します。
- 全31公開ヘッダの単独・重複include、全カテゴリのall.hpp混在と2翻訳単位、31使用例と図解の出力を確認します。チェックはNDEBUGでも無効になりません。
- 図解は31サンプル、150ランダムバッチ・60集合比較・不正入力を検証。ブラウザでは全モード、31変更入力、前後移動・再生・エラー復帰・390px表示を確認します。

実行結果は共通suiteの [検証記録フォルダ](../../verification/evidence/) の `polynomial_extensions/` に保存します。ソース変更で記録は失効します。オンライン提出・GitHub Actions上の実行は未実施です。

2026-10-27の最終ソースで、[通常](../../verification/evidence/polynomial_extensions/standard.json)（91.3秒）・[stdlib-debug](../../verification/evidence/polynomial_extensions/stdlib-debug.json)（108.9秒）・[ASan/UBSan/LeakSanitizer](../../verification/evidence/polynomial_extensions/sanitize.json)（139.5秒）のDebug/NDEBUG検証が成功しました。[既存3.16〜3.20 Bの回帰](../../verification/evidence/b_16_20/standard.json)（82.7秒）も成功し、4記録は同じソースfingerprintに一致しています。図解モデル・31モード／31編集入力のブラウザ検証・390px表示・画像目視も完了しています。
