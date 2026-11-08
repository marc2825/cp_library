---
title: 分数計画・Dinkelbach・比率二分
documentation_of: //combinatorial_optimization/fractional_programming.hpp
---

# O-04 分数計画・Dinkelbach・比率二分

[カテゴリ](README.md) / [実装](../../combinatorial_optimization/fractional_programming.hpp) / [操作できる図解](../../visualizations/optimization_extensions/index.html#fractional_programming) / [一次資料](references.md)

比率の問題をf−λgの最適化に変え、選んだ点の比率または区間の中点へλを更新する。

## API・条件

最大化する比率は f(x)/g(x)。oracle(λ)は `optional<FractionalPoint>` を返し、全実行可能点の中でf−λgを大域最大化する必要があります。FractionalPointはnumerator、denominator、solution（任意の復元情報用のlong double列）を持ちます。局所解や単なる実行可能点では上下界の契約が成立しません。

全域でg≥denominator_lower_bound>0を呼び出し側が保証します。返された点でも下限を検査しますが、全域の正値性の証明は行いません。分母が常に負なら分子・分母を両方反転して正に直せます。0を跨ぐ場合はそのまま使えません。

Dinkelbachはxを得たらλ←f(x)/g(x)。F(λ)=max(f−λg)の符号と分母下限から上界を作り、返された点の比率を下界とします。F≥0なら最適比率≤λ+F/g_min、F≤0なら最適比率≤λです。任意の有限startから開始できます。

二分法は既知のlower≤最適比率≤upperを入力します。両端のoracleで符号を検査してから区間を縮めます。max_iterations=0でも両端を評価し、Dinkelbachの0回は評価しません。二分法の評価数は最大max_iterations+2です。

FractionalOptionsはtolerance（絶対＋相対、既定各1e−12）、denominator_lower_bound（既定1e−12）、max_iterations（既定128）。結果はstatus/has_solution/point/ratio/lower/upper/evaluations。statusはconverged/infeasible/invalid_bracket/iteration_limit/stagnation/non_finiteです。convergedは残差0または上下界の許容幅、同じ比率へ丸められた更新はstagnation。計算はlong doubleであり、返した数値区間を厳密な外向き丸めの証明とは扱いません。終了時は最良の実行可能候補を保持します。

有限候補の走査のほか、fとgが一次式なら[線形計画](linear_program.md)でc_f−λc_gを目的としてoracleを構成できます。LPが大域最適に終了したことと、分母下限・最適値の達成を確認してからFractionalPointへ変換してください。

## 計算量

I反復・oracle費用FでO(IF)時間。solutionのコピーを除きO(1)追加領域。

## 使用例

```cpp
#include "combinatorial_optimization/fractional_programming.hpp"
#include <iostream>
using namespace cp;
int main() {
    std::vector<FractionalPoint> points{{5, 2, {}}, {7, 3, {}}, {9, 4, {}}};
    auto oracle = [&](long double lambda) -> std::optional<FractionalPoint> {
        auto best = points[0];
        for (auto p : points)
            if (p.numerator - lambda * p.denominator > best.numerator - lambda * best.denominator)
                best = p;
        return best;
    };
    FractionalOptions options;
    options.denominator_lower_bound = 2;
    std::cout << fractional_dinkelbach(oracle, 0, options).ratio << " "
              << fractional_bisection(oracle, 0, 5, options).ratio << "\n";
}
```

出力:

```text
2.5 2.5
```

## 使用する問題・検証

利益/費用や総得点/総量の比率を最適化する問題。 オンラインACは未確認です。

[単体・全探索比較](../../verify/optimization_extensions/unit_test.cpp)、[独立Python比較](../../verify/run_optimization_extensions.py)、[使用例と参照値](../../verify/optimization_extensions/manifest.json)で検証します。検証コマンドと図解との相違は[カテゴリREADME](README.md)を参照してください。
