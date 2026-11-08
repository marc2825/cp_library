# 3.12〜3.15 B項目

対象全30掲載箇所を整備しました。[30モードの図解](../../visualizations/b_12_15/index.html) / [一次資料と設計判断](references.md)。既存Min-Plusを拡張し、29公開ヘッダを追加。30個別説明・30使用例・独立した比較検証を用意しています。整数根・商列挙・素数カウントは既存実体や共通実装を共有します。

## 対応範囲

| カテゴリ | B掲載数 | 実装 |
|---|---:|---|
| 3.12 DP・最適化 | 7 | 凸／凹Min-Plus、Monge最短路・ちょうどK辺、LARSCH、CDQ変換、分枝限定0-1、Boolean行列積Four Russians |
| 3.13 基本数論 | 8 | 一般BSGS、Tonelli–Shanks、共有整数根、有限modテトレーション、共有商列挙、合同式全解、Lucas、共有素数カウント |
| 3.14 高度な整数 | 6 | 約数個数和、Du Jiaoのφ総和・Mertens、Lucy DP、線形篩の乗法的prefix、平方因子なし個数 |
| 3.15 組合せ | 9 | Eulerian、Bernoulli、Narayana、Lagrange係数、Stirling／二項変換、階乗基底、標本点シフト、二項prefix |

Mongeは前向き完全DAGとMonge条件を前提にし、辺数制約版はO(KN)のSMAWK各段方式で復元します。オンライン・オフライン変換は左を確定して右へ寄与を転送できるCDQ、Four RussiansはBoolean行列積です。分枝限定は最悪指数時間です。

素数個数は3.13/3.14で共通Lucy DP。任意の乗法的関数の総和は全prefixを線形に作る版です。一般化Min_25や任意の巨大N総和はCに残します。Stirling・二項・基底変換と二項prefixは一般法の二乗表、標本点シフトは既存補間と多点評価を使います。適用条件・数値範囲・メモリ予算は個別説明を参照してください。

## ローカル検証

```sh
python3 verify/run_b_12_15.py
python3 verify/run_b_12_15.py --sanitize
python3 verify/run_b_12_15.py --links-only
make verify-b-12-15
node verify/visualizations/b_12_15_model_test.js
node verify/visualizations/b_12_15_browser_test.mjs --browser /path/to/chromium
```

30ヘッダの単独・重複includeをdebug/releaseで検査します。全カテゴリall.hppを混在させた複数翻訳単位、30例と文書usage、以下のC++比較、4ジャッジ形式×24入力を実行します。CHECKはNDEBUGでも有効。--sanitizeはASan/UBSan/LeakSanitizerを使います。

| 検証 | 独立した比較方法 |
|---|---|
| [dp_test.cpp](../../verify/b_12_15/dp_test.cpp) | 全組合せのMin-Plus、二乗距離＋頂点費用の素朴DPと全辺数段、LARSCHの依存参照順、CDQの確定順と直接畳み込み、全品物subset、直接Boolean積。10万状態のLARSCH評価回数も検査。 |
| [number_test.cpp](../../verify/b_12_15/number_test.cpp) | 小さい法の全剰余軌道・平方根・合同式、PascalとLucas、表現できる整数塔、素数篩と全Lucy商、gcdによるφ、直接素因数／約数／平方因子。pi(10^9)、phi/Mertens/square-freeの既知小値、uint64根境界、範囲超過も確認。 |
| [combinatorics_test.cpp](../../verify/b_12_15/combinatorics_test.cpp) | 全順列の上昇、Dyck全列挙のpeak、直接べき和、形式的固定点の反復合成、集合分割、二項係数の直接積、上昇／下降多項式評価、標本点Horner評価、直接二項区間和。mod1/12/素数と不可逆条件も確認。 |
| [ジャッジ形式](../../verify/b_12_15/) | 一般Discrete Log、Tetration Mod、Counting Primes、凹×任意Min-Plus。Pythonの有限軌道・任意精度pow・篩・全添字対で各24入力をdebug/release比較。テトレーションは64bit底と法、非互いに素、巨大な正確な指数も含む。 |

問題URLは入出力仕様の参照先です。オンライン提出・ACや、全ジャッジ最大制約での時間内完了は未確認です。実装の計算量と予算に合わせて使用してください。

## 公開API一覧

| 項目 | 実装 | 説明 | 実行例 |
|---|---|---|---|
| Min-Plus Convolution（凸・凹） | [dp_optimization/min_plus_convolution.hpp](../../dp_optimization/min_plus_convolution.hpp) | [API・条件・計算量](../dp_optimization/min_plus_convolution_b.md) | [使用例](../../examples/dp_optimization/min_plus_convolution_b.cpp) |
| Monge Shortest Path | [dp_optimization/monge_shortest_path.hpp](../../dp_optimization/monge_shortest_path.hpp) | [API・条件・計算量](../dp_optimization/monge_shortest_path.md) | [使用例](../../examples/dp_optimization/monge_shortest_path_b.cpp) |
| Mongeの辺数制約付き最短路 | [dp_optimization/monge_shortest_path_k_edges.hpp](../../dp_optimization/monge_shortest_path_k_edges.hpp) | [API・条件・計算量](../dp_optimization/monge_shortest_path_k_edges.md) | [使用例](../../examples/dp_optimization/monge_shortest_path_k_edges_b.cpp) |
| LARSCH | [dp_optimization/larsch.hpp](../../dp_optimization/larsch.hpp) | [API・条件・計算量](../dp_optimization/larsch.md) | [使用例](../../examples/dp_optimization/larsch_b.cpp) |
| オンライン・オフライン変換（CDQ） | [dp_optimization/online_offline_dp.hpp](../../dp_optimization/online_offline_dp.hpp) | [API・条件・計算量](../dp_optimization/online_offline_dp.md) | [使用例](../../examples/dp_optimization/online_offline_dp_b.cpp) |
| 分枝限定ナップサック | [dp_optimization/branch_bound_knapsack.hpp](../../dp_optimization/branch_bound_knapsack.hpp) | [API・条件・計算量](../dp_optimization/branch_bound_knapsack.md) | [使用例](../../examples/dp_optimization/branch_bound_knapsack_b.cpp) |
| Four Russians法（Boolean行列積） | [dp_optimization/four_russians.hpp](../../dp_optimization/four_russians.hpp) | [API・条件・計算量](../dp_optimization/four_russians.md) | [使用例](../../examples/dp_optimization/four_russians_b.cpp) |
| Discrete Log（BSGS） | [number_theory_basic/discrete_log.hpp](../../number_theory_basic/discrete_log.hpp) | [API・条件・計算量](../number_theory_basic/discrete_log.md) | [使用例](../../examples/number_theory_basic/discrete_log_b.cpp) |
| mod平方根（Tonelli–Shanks） | [number_theory_basic/mod_sqrt.hpp](../../number_theory_basic/mod_sqrt.hpp) | [API・条件・計算量](../number_theory_basic/mod_sqrt.md) | [使用例](../../examples/number_theory_basic/mod_sqrt_b.cpp) |
| 整数k乗根（数論入口） | [number_theory_basic/integer_kth_root.hpp](../../number_theory_basic/integer_kth_root.hpp) | [API・条件・計算量](../number_theory_basic/integer_kth_root.md) | [使用例](../../examples/number_theory_basic/integer_kth_root_b.cpp) |
| modテトレーション | [number_theory_basic/mod_tetration.hpp](../../number_theory_basic/mod_tetration.hpp) | [API・条件・計算量](../number_theory_basic/mod_tetration.md) | [使用例](../../examples/number_theory_basic/mod_tetration_b.cpp) |
| 商列挙（数論入口） | [number_theory_basic/quotient_enumeration.hpp](../../number_theory_basic/quotient_enumeration.hpp) | [API・条件・計算量](../number_theory_basic/quotient_enumeration.md) | [使用例](../../examples/number_theory_basic/quotient_enumeration_b.cpp) |
| 線形合同方程式 | [number_theory_basic/linear_congruence.hpp](../../number_theory_basic/linear_congruence.hpp) | [API・条件・計算量](../number_theory_basic/linear_congruence.md) | [使用例](../../examples/number_theory_basic/linear_congruence_b.cpp) |
| Lucasの定理 | [number_theory_basic/lucas.hpp](../../number_theory_basic/lucas.hpp) | [API・条件・計算量](../number_theory_basic/lucas.md) | [使用例](../../examples/number_theory_basic/lucas_b.cpp) |
| 素数カウント（共通入口） | [number_theory_basic/prime_count.hpp](../../number_theory_basic/prime_count.hpp) | [API・条件・計算量](../number_theory_basic/prime_count.md) | [使用例](../../examples/number_theory_basic/prime_count_b.cpp) |
| 約数個数和 | [number_theory_convolution/divisor_count_sum.hpp](../../number_theory_convolution/divisor_count_sum.hpp) | [API・条件・計算量](../number_theory_convolution/divisor_count_sum.md) | [使用例](../../examples/number_theory_convolution/divisor_count_sum_b.cpp) |
| Euler φの総和 | [number_theory_convolution/summatory_totient.hpp](../../number_theory_convolution/summatory_totient.hpp) | [API・条件・計算量](../number_theory_convolution/summatory_totient.md) | [使用例](../../examples/number_theory_convolution/summatory_totient_b.cpp) |
| Mertens Function | [number_theory_convolution/mertens.hpp](../../number_theory_convolution/mertens.hpp) | [API・条件・計算量](../number_theory_convolution/mertens.md) | [使用例](../../examples/number_theory_convolution/mertens_b.cpp) |
| 素数個数計算（Lucy DP） | [number_theory_convolution/prime_counting.hpp](../../number_theory_convolution/prime_counting.hpp) | [API・条件・計算量](../number_theory_convolution/prime_counting.md) | [使用例](../../examples/number_theory_convolution/prime_counting_b.cpp) |
| 乗法的関数の総和 | [number_theory_convolution/multiplicative_function_sum.hpp](../../number_theory_convolution/multiplicative_function_sum.hpp) | [API・条件・計算量](../number_theory_convolution/multiplicative_function_sum.md) | [使用例](../../examples/number_theory_convolution/multiplicative_function_sum_b.cpp) |
| Square-Free Counting | [number_theory_convolution/square_free_counting.hpp](../../number_theory_convolution/square_free_counting.hpp) | [API・条件・計算量](../number_theory_convolution/square_free_counting.md) | [使用例](../../examples/number_theory_convolution/square_free_counting_b.cpp) |
| Eulerian数 | [combinatorics/eulerian_numbers.hpp](../../combinatorics/eulerian_numbers.hpp) | [API・条件・計算量](../combinatorics/eulerian_numbers.md) | [使用例](../../examples/combinatorics/eulerian_numbers_b.cpp) |
| Bernoulli数 | [combinatorics/bernoulli_numbers.hpp](../../combinatorics/bernoulli_numbers.hpp) | [API・条件・計算量](../combinatorics/bernoulli_numbers.md) | [使用例](../../examples/combinatorics/bernoulli_numbers_b.cpp) |
| Narayana数 | [combinatorics/narayana.hpp](../../combinatorics/narayana.hpp) | [API・条件・計算量](../combinatorics/narayana.md) | [使用例](../../examples/combinatorics/narayana_b.cpp) |
| Lagrange Inversion | [combinatorics/lagrange_inversion.hpp](../../combinatorics/lagrange_inversion.hpp) | [API・条件・計算量](../combinatorics/lagrange_inversion.md) | [使用例](../../examples/combinatorics/lagrange_inversion_b.cpp) |
| Stirling Transform | [combinatorics/stirling_transform.hpp](../../combinatorics/stirling_transform.hpp) | [API・条件・計算量](../combinatorics/stirling_transform.md) | [使用例](../../examples/combinatorics/stirling_transform_b.cpp) |
| 二項変換 | [combinatorics/binomial_transform.hpp](../../combinatorics/binomial_transform.hpp) | [API・条件・計算量](../combinatorics/binomial_transform.md) | [使用例](../../examples/combinatorics/binomial_transform_b.cpp) |
| 階乗べきとの基底変換 | [combinatorics/factorial_basis.hpp](../../combinatorics/factorial_basis.hpp) | [API・条件・計算量](../combinatorics/factorial_basis.md) | [使用例](../../examples/combinatorics/factorial_basis_b.cpp) |
| 標本点シフト | [combinatorics/shift_sample_points.hpp](../../combinatorics/shift_sample_points.hpp) | [API・条件・計算量](../combinatorics/shift_sample_points.md) | [使用例](../../examples/combinatorics/shift_sample_points_b.cpp) |
| 二項係数のPrefix Sum | [combinatorics/binomial_prefix_sum.hpp](../../combinatorics/binomial_prefix_sum.hpp) | [API・条件・計算量](../combinatorics/binomial_prefix_sum.md) | [使用例](../../examples/combinatorics/binomial_prefix_sum_b.cpp) |
