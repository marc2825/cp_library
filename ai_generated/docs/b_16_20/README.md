# 3.16〜3.20 B項目

対象全31掲載箇所を整備しました。[実装予定リスト](../implementation_plan.md) / [31モードの図解](../../visualizations/b_16_20/index.html) / [一次資料と設計判断](references.md)。既存のGCD/LCM・Min-Plus・BMと第N項・Matrix Tree・Subset Convolutionは共通利用し、公開入口と不足機能を補っています。

| カテゴリ | B掲載数 | 主な対応 |
|---|---:|---|
| 3.16 畳み込み | 7 | 両列online、GCD/LCM共有、形状付きMin-Plus共有、多次元通常積、ブロック積、中間積、等比点評価 |
| 3.17 FPS | 8 | BSGS合成、Newton合成逆、Sparse、Online inv/exp、Euclid GCD、剰余冪、再利用積木、重根付き部分分数 |
| 3.18 漸化式・母関数 | 6 | BMとKitamasa共有、既知P-recursiveのprefix、EGF、係数抽出、有理式演算、多項式重み付き等比和 |
| 3.19 集合関数 | 4 | SPS inv/log/exp、有限多項式合成、分割DP、既存ranked-zeta積の素数mod入口 |
| 3.20 線形代数 | 6 | CSR、Krylovと冪作用・求解、Wiedemann、決定的最小多項式、Matrix Tree共有、Woodbury |

## 適用条件

FPS系は既存の素数法・原始根・NTT容量の契約を引き継ぎます。Sparse FPS、SPS、CSR、Black-Box、Wiedemann、決定的最小多項式、低ランク更新にはNTT原始根の条件はありません。各APIの法・除算・空入力・容量条件は個別説明を参照してください。

合成はbaby-step/giant-step、中間積は通常積からの抽出、巨大配列積はブロック対ごとの再変換方式です。最小多項式は決定的ですが最悪O(n⁴)。Wiedemannの射影多項式を行列全体の最小多項式とは扱いません。求解で返した値は残差を検証し、nulloptは解なしの証明に使いません。P-recursiveは既知の式の逐次生成です。C項目の高速合成・高速中間積・Holonomic推定・Frobenius標準形などへ範囲を広げません。

## 公開API

| 項目 | 実装 | 説明 | 実行例 |
|---|---|---|---|
| Relaxed / Online Convolution | [convolution/relaxed_convolution.hpp](../../convolution/relaxed_convolution.hpp) | [API・条件・計算量](../convolution/relaxed_convolution.md) | [使用例](../../examples/convolution/relaxed_convolution_b.cpp) |
| GCD / LCM Convolution（共通入口） | [convolution/gcd_lcm_convolution.hpp](../../convolution/gcd_lcm_convolution.hpp) | [API・条件・計算量](../convolution/gcd_lcm_convolution.md) | [使用例](../../examples/convolution/gcd_lcm_convolution_b.cpp) |
| Min-Plus Convolution（共通入口） | [convolution/min_plus_convolution.hpp](../../convolution/min_plus_convolution.hpp) | [API・条件・計算量](../convolution/min_plus_convolution.md) | [使用例](../../examples/convolution/min_plus_convolution_b.cpp) |
| 多次元畳み込み | [convolution/multidimensional_convolution.hpp](../../convolution/multidimensional_convolution.hpp) | [API・条件・計算量](../convolution/multidimensional_convolution.md) | [使用例](../../examples/convolution/multidimensional_convolution_b.cpp) |
| 巨大配列の畳み込み | [convolution/convolution_large.hpp](../../convolution/convolution_large.hpp) | [API・条件・計算量](../convolution/convolution_large.md) | [使用例](../../examples/convolution/convolution_large_b.cpp) |
| Middle Product | [convolution/middle_product.hpp](../../convolution/middle_product.hpp) | [API・条件・計算量](../convolution/middle_product.md) | [使用例](../../examples/convolution/middle_product_b.cpp) |
| Chirp-Z Transform | [convolution/chirp_z.hpp](../../convolution/chirp_z.hpp) | [API・条件・計算量](../convolution/chirp_z.md) | [使用例](../../examples/convolution/chirp_z_b.cpp) |
| FPS合成 | [fps/composition.hpp](../../fps/composition.hpp) | [API・条件・計算量](../fps/composition.md) | [使用例](../../examples/fps/composition_b.cpp) |
| FPS合成逆関数 | [fps/compositional_inverse.hpp](../../fps/compositional_inverse.hpp) | [API・条件・計算量](../fps/compositional_inverse.md) | [使用例](../../examples/fps/compositional_inverse_b.cpp) |
| Sparse FPS | [fps/sparse_fps.hpp](../../fps/sparse_fps.hpp) | [API・条件・計算量](../fps/sparse_fps.md) | [使用例](../../examples/fps/sparse_fps_b.cpp) |
| Online FPS | [fps/online_fps.hpp](../../fps/online_fps.hpp) | [API・条件・計算量](../fps/online_fps.md) | [使用例](../../examples/fps/online_fps_b.cpp) |
| Polynomial GCD | [fps/polynomial_gcd.hpp](../../fps/polynomial_gcd.hpp) | [API・条件・計算量](../fps/polynomial_gcd.md) | [使用例](../../examples/fps/polynomial_gcd_b.cpp) |
| Polynomial Mod Pow | [fps/polynomial_mod_pow.hpp](../../fps/polynomial_mod_pow.hpp) | [API・条件・計算量](../fps/polynomial_mod_pow.md) | [使用例](../../examples/fps/polynomial_mod_pow_b.cpp) |
| Subproduct Tree | [fps/subproduct_tree.hpp](../../fps/subproduct_tree.hpp) | [API・条件・計算量](../fps/subproduct_tree.md) | [使用例](../../examples/fps/subproduct_tree_b.cpp) |
| Partial Fraction Decomposition | [fps/partial_fraction.hpp](../../fps/partial_fraction.hpp) | [API・条件・計算量](../fps/partial_fraction.md) | [使用例](../../examples/fps/partial_fraction_b.cpp) |
| 線形漸化式の推定と第N項 | [linear_recurrence/infer_nth.hpp](../../linear_recurrence/infer_nth.hpp) | [API・条件・計算量](../linear_recurrence/infer_nth.md) | [使用例](../../examples/linear_recurrence/infer_nth_b.cpp) |
| P-recursive数列 | [linear_recurrence/p_recursive.hpp](../../linear_recurrence/p_recursive.hpp) | [API・条件・計算量](../linear_recurrence/p_recursive.md) | [使用例](../../examples/linear_recurrence/p_recursive_b.cpp) |
| 指数型母関数（EGF）の演算 | [linear_recurrence/egf.hpp](../../linear_recurrence/egf.hpp) | [API・条件・計算量](../linear_recurrence/egf.md) | [使用例](../../examples/linear_recurrence/egf_b.cpp) |
| 母関数の係数抽出 | [linear_recurrence/coefficient_extraction.hpp](../../linear_recurrence/coefficient_extraction.hpp) | [API・条件・計算量](../linear_recurrence/coefficient_extraction.md) | [使用例](../../examples/linear_recurrence/coefficient_extraction_b.cpp) |
| 有理型母関数 | [linear_recurrence/rational_generating_function.hpp](../../linear_recurrence/rational_generating_function.hpp) | [API・条件・計算量](../linear_recurrence/rational_generating_function.md) | [使用例](../../examples/linear_recurrence/rational_generating_function_b.cpp) |
| 等比数列の多項式重み付き和 | [linear_recurrence/polynomial_geometric_sum.hpp](../../linear_recurrence/polynomial_geometric_sum.hpp) | [API・条件・計算量](../linear_recurrence/polynomial_geometric_sum.md) | [使用例](../../examples/linear_recurrence/polynomial_geometric_sum_b.cpp) |
| SPSのexp / log / inv | [set_function/set_power_series.hpp](../../set_function/set_power_series.hpp) | [API・条件・計算量](../set_function/set_power_series.md) | [使用例](../../examples/set_function/set_power_series_b.cpp) |
| SPSの合成 | [set_function/sps_composition.hpp](../../set_function/sps_composition.hpp) | [API・条件・計算量](../set_function/sps_composition.md) | [使用例](../../examples/set_function/sps_composition_b.cpp) |
| 集合分割DP | [set_function/set_partition.hpp](../../set_function/set_partition.hpp) | [API・条件・計算量](../set_function/set_partition.md) | [使用例](../../examples/set_function/set_partition_b.cpp) |
| 高速な部分集合畳み込み | [set_function/fast_subset_convolution.hpp](../../set_function/fast_subset_convolution.hpp) | [API・条件・計算量](../set_function/fast_subset_convolution.md) | [使用例](../../examples/set_function/fast_subset_convolution_b.cpp) |
| Sparse Matrix | [linear_algebra/sparse_matrix.hpp](../../linear_algebra/sparse_matrix.hpp) | [API・条件・計算量](../linear_algebra/sparse_matrix.md) | [使用例](../../examples/linear_algebra/sparse_matrix_b.cpp) |
| Black-Box Linear Algebra | [linear_algebra/black_box.hpp](../../linear_algebra/black_box.hpp) | [API・条件・計算量](../linear_algebra/black_box.md) | [使用例](../../examples/linear_algebra/black_box_b.cpp) |
| Wiedemann Algorithm | [linear_algebra/wiedemann.hpp](../../linear_algebra/wiedemann.hpp) | [API・条件・計算量](../linear_algebra/wiedemann.md) | [使用例](../../examples/linear_algebra/wiedemann_b.cpp) |
| 行列の最小多項式 | [linear_algebra/minimal_polynomial.hpp](../../linear_algebra/minimal_polynomial.hpp) | [API・条件・計算量](../linear_algebra/minimal_polynomial.md) | [使用例](../../examples/linear_algebra/minimal_polynomial_b.cpp) |
| Matrix Tree Theorem（共通入口） | [linear_algebra/matrix_tree.hpp](../../linear_algebra/matrix_tree.hpp) | [API・条件・計算量](../linear_algebra/matrix_tree.md) | [使用例](../../examples/linear_algebra/matrix_tree_b.cpp) |
| Low-Rank Update | [linear_algebra/low_rank_update.hpp](../../linear_algebra/low_rank_update.hpp) | [API・条件・計算量](../linear_algebra/low_rank_update.md) | [使用例](../../examples/linear_algebra/low_rank_update_b.cpp) |

## ローカル検証

```sh
python3 verify/run_b_16_20.py
python3 verify/run_b_16_20.py --sanitize
python3 verify/run_b_16_20.py --links-only
make verify-b-16-20
node verify/visualizations/b_16_20_model_test.js
node verify/visualizations/b_16_20_browser_test.mjs --browser /path/to/chromium
```

31公開入口の単独・重複include（debug/release）、全カテゴリall.hppを混在させる複数翻訳単位、31使用例と文書usageの一致を検証します。CHECKはNDEBUGでも有効。--sanitizeはASan/UBSan/LeakSanitizerです。

- [畳み込み比較](../../verify/b_16_20/convolution_test.cpp)：各追加時点の直接和、容量16を超える法17のブロック積、全添字対の3次元tensor積、直接窓内積、Hornerによる等比点評価。32768項のstreamingも確認。
- [FPS・母関数比較](../../verify/b_16_20/fps_recurrence_test.cpp)：逐次冪合成、左右の合成逆、直接係数漸化式、重根付き部分分数の分子復元、Horner・積木の再利用、PascalによるEGF積、直接有限和、有理式の和・積、uint64最大指数と項数。
- [集合・代数比較](../../verify/b_16_20/set_algebra_test.cpp)：restricted-growth stringで全分割を独立列挙、全submask積、標数2/3のSPS、CSRと密行列、射影列の消去、法3で低次数monic多項式を全探索、密行列逆と置換和の行列式によるWoodbury比較。
- [ジャッジ形式](../../verify/b_16_20/)：Convolution、Composition of FPS、Exp of SPS、Online Exp of FPS。各24入力をPython整数の定義から生成し、debug/releaseで比較します。

オンライン提出・ACと、全ジャッジ最大制約での時間内完了は未確認です。図解の簡略化は[図解README](../../visualizations/b_16_20/README.md)へ記載しています。

## 今回の検証結果（2026-10-18）

通常・ASan/UBSan/LeakSanitizerのdebug/release、31使用例、31件のC++・図解参照値、4ジャッジ形式×24入力が成功。追加の4096項Online FPS、256項合成逆、128次巡回作用とuint64最大冪、16次の決定的最小多項式、P-recursiveの非定数係数とEGF容量境界も確認しました。既存convolution/FPS/linear_recurrence/set_function/linear_algebra/transform_a/number_theory_convolution/b_12_15/advanced/graph_combinatoricsの10系統の通常回帰が成功。`make verify-visualizations`による全図解モデルの回帰も成功。図解モデルは2,394入力・16,582状態・22拒否条件、31モードの全フレームのSVG描画範囲・ブラウザ操作と390px表示が成功。デスクトップ・モバイル画像を確認し、関連ローカル文書リンク1,390件に欠落なし。Chromiumのsocket制限・LeakSanitizerのptrace制限を避けるため、同じローカル検証を承認レビュー付きで制限外から実行しました。 今回の変更は未コミットです。
