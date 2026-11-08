# 数論 N全19項目

[一次資料・設計判断](references.md) / [入力を操作できる19モード](../../visualizations/number_theory_extensions/index.html)

C++17、namespace cp。N-01〜N-10、N-20〜N-28を追加しました。N-11〜N-19の行はありません。N-06/07は既存の組合せカテゴリ、商prefixと大規模総和は3.14、ほかは3.13に置き、各all.hppへ登録しています。

| ID | 公開APIの説明 | 使用例 | 図解 |
|---|---|---|---|
| N-01 | [区間篩](../number_theory_basic/segmented_sieve.md) | [C++](../../examples/number_theory_basic/segmented_sieve.cpp) | [操作](../../visualizations/number_theory_extensions/index.html#segmented_sieve) |
| N-02 | [乗法位数・Carmichael関数](../number_theory_basic/multiplicative_order.md) | [C++](../../examples/number_theory_basic/multiplicative_order.cpp) | [操作](../../visualizations/number_theory_extensions/index.html#multiplicative_order) |
| N-03 | [一次式modの最小値](../number_theory_basic/min_mod_linear.md) | [C++](../../examples/number_theory_basic/min_mod_linear.cpp) | [操作](../../visualizations/number_theory_extensions/index.html#min_mod_linear) |
| N-04 | [多項式Floor Sum・万能Euclid](../number_theory_basic/floor_sum_polynomial.md) | [C++](../../examples/number_theory_basic/floor_sum_polynomial.cpp) | [操作](../../visualizations/number_theory_extensions/index.html#floor_sum_polynomial) |
| N-05 | [Batch Inversion](../number_theory_basic/batch_inversion.md) | [C++](../../examples/number_theory_basic/batch_inversion.cpp) | [操作](../../visualizations/number_theory_extensions/index.html#batch_inversion) |
| N-06 | [大引数・任意mod二項係数](../combinatorics/prime_power_binomial.md) | [C++](../../examples/combinatorics/prime_power_binomial.cpp) | [操作](../../visualizations/number_theory_extensions/index.html#prime_power_binomial) |
| N-07 | [巨大なnの階乗 mod p](../combinatorics/large_factorial.md) | [C++](../../examples/combinatorics/large_factorial.cpp) | [操作](../../visualizations/number_theory_extensions/index.html#large_factorial) |
| N-08 | [再利用可能な離散対数表](../number_theory_basic/discrete_log_table.md) | [C++](../../examples/number_theory_basic/discrete_log_table.cpp) | [操作](../../visualizations/number_theory_extensions/index.html#discrete_log_table) |
| N-09 | [商prefix表のDirichlet積](../number_theory_convolution/dirichlet_prefix_product.md) | [C++](../../examples/number_theory_convolution/dirichlet_prefix_product.cpp) | [操作](../../visualizations/number_theory_extensions/index.html#dirichlet_prefix_product) |
| N-10 | [Dirichlet逆元](../number_theory_convolution/dirichlet_inverse.md) | [C++](../../examples/number_theory_convolution/dirichlet_inverse.cpp) | [操作](../../visualizations/number_theory_extensions/index.html#dirichlet_inverse) |
| N-20 | [mod k乗根](../number_theory_basic/mod_kth_root.md) | [C++](../../examples/number_theory_basic/mod_kth_root.cpp) | [操作](../../visualizations/number_theory_extensions/index.html#mod_kth_root) |
| N-21 | [Gaussian Integer](../number_theory_basic/gaussian_integer.md) | [C++](../../examples/number_theory_basic/gaussian_integer.cpp) | [操作](../../visualizations/number_theory_extensions/index.html#gaussian_integer) |
| N-22 | [連分数・有理数近似](../number_theory_basic/continued_fraction.md) | [C++](../../examples/number_theory_basic/continued_fraction.cpp) | [操作](../../visualizations/number_theory_extensions/index.html#continued_fraction) |
| N-23 | [Pell方程式](../number_theory_basic/pell_equation.md) | [C++](../../examples/number_theory_basic/pell_equation.cpp) | [操作](../../visualizations/number_theory_extensions/index.html#pell_equation) |
| N-24 | [二平方数和の全表示](../number_theory_basic/sum_two_squares.md) | [C++](../../examples/number_theory_basic/sum_two_squares.cpp) | [操作](../../visualizations/number_theory_extensions/index.html#sum_two_squares) |
| N-25 | [高速素数和](../number_theory_convolution/prime_sum.md) | [C++](../../examples/number_theory_convolution/prime_sum.cpp) | [操作](../../visualizations/number_theory_extensions/index.html#prime_sum) |
| N-26 | [一般化Min_25 Sieve](../number_theory_convolution/min25_sieve.md) | [C++](../../examples/number_theory_convolution/min25_sieve.cpp) | [操作](../../visualizations/number_theory_extensions/index.html#min25_sieve) |
| N-27 | [任意乗法的関数の大規模総和](../number_theory_convolution/multiplicative_summatory.md) | [C++](../../examples/number_theory_convolution/multiplicative_summatory.cpp) | [操作](../../visualizations/number_theory_extensions/index.html#multiplicative_summatory) |
| N-28 | [重み付き多項式mod和](../number_theory_basic/mod_sum_polynomial.md) | [C++](../../examples/number_theory_basic/mod_sum_polynomial.cpp) | [操作](../../visualizations/number_theory_extensions/index.html#mod_sum_polynomial) |

## 対応範囲

N-20は素数法のk乗根、N-07は2..2×10^9の実行時素数の階乗、N-06は素数冪ごとの表を確保する固定法二項係数です。前計算予算を明示し、非unit・0・空区間を区別します。N-27は任意の乗法的関数の素数prefixを呼出側が与えるAPI。N-28は区間上のΣ i^p·((ai+b) mod m)^qとして具体化しました。BigIntを使うAPIはBoostヘッダが必要です。

## 検証

```sh
make verify-number-theory-extensions
python3 -m tools.cp_tools.suite --runners number_theory_extensions
python3 -m tools.cp_tools.suite --stdlib-debug --runners number_theory_extensions
python3 -m tools.cp_tools.suite --sanitize --runners number_theory_extensions
node verify/visualizations/number_theory_extensions_model_test.js
node verify/visualizations/number_theory_extensions_browser_test.mjs --browser /path/to/chromium
```

| テスト | 独立比較・境界 |
|---|---|
| modular_test | 試し割りの区間篩、単位群の全周期からλ/位数、全べき列と再利用BSGS、一括逆元の1回callback、PascalとBigInt積による二項係数、線形階乗100万と5億付近の積恒等式、全候補のmod根。UINT64_MAX・大きな64bit素数・非unit・予算も確認。 |
| floor_test | 非可換文字列として操作列を全列挙、負係数のmomentを直接総和、最初の最小添字・mod多項式和をランダム比較。10^30長の区間を閉じた式と照合。 |
| algebraic_test | Gaussian除算の恒等式とnorm減少・gcd整除、全分母の有理近似、Pellのnormと小さい正の解、D=661の大きな解、3000までの二平方数和全表示と64bit端点。 |
| summatory_test | ランダム算術関数の約数列挙による積・配列/表の逆元、全gcdによるφ、約数列挙による個数/和、法12の冪和、合同類に依存する素数項。N=10^8の素数和・定数関数・Mertens総和。 |

21ヘッダの単独/重複includeをDebug/NDEBUGで確認し、全カテゴリ混在・2翻訳単位、19個のdocsと一致する実行例・図解の参照値を照合します。C++4テストはDebug/NDEBUG両方で実行。図解モデルは5,386比較、ブラウザは19モード・8変更入力・エラー・再生・リンク・390px表示を確認します。

実行記録は `verification/evidence/number_theory_extensions/`、ログは `build/suites/number_theory_extensions/` へ保存します。GitHub Actionsの実行とオンライン提出ACは未確認です。
