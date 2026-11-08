---
title: 劣モジュラ最適化
documentation_of: //combinatorial_optimization/submodular_optimization.hpp
---

# O-22 劣モジュラ最適化

[カテゴリ](README.md) / [実装](../../combinatorial_optimization/submodular_optimization.hpp) / [操作できる図解](../../visualizations/optimization_extensions/index.html#submodular_optimization) / [一次資料](references.md)

Lovász拡張のepigraphをLPで下から近似し、重み順の貪欲基を制約として追加する。

## API・条件

`submodular_minimize(n,F,SubmodularOptions)` は一般の劣モジュラ関数を制約なしで最小化します。Fの値はBigRational、空集合値は0でなくても構いません。内部で[基多面体の貪欲法](base_polyhedron.md)と[厳密Simplex](linear_program.md)を共用します。

初期の基sから L=Σmin(0,sᵢ) を下界とします。各貪欲基sに対してθ≥s·xを加え、0≤x≤1上でθを最小化するLPを解きます。LP解xの順序から新しい基を得て、違反する制約を追加します。prefix集合は実行可能な離散解の候補となります。下界と候補値が一致したときだけoptimalです。

結果はvalue（候補集合selectedの値）、lower_bound、status、iterations、cuts、oracle_calls。予算超過時もlower_bound≤真の最小値≤valueを保持します。既定max_iterations=256で、LP1回のpivot予算はoptions.simplex。statusはoptimal/iteration_limit/lp_limit。0反復でも初期基から上下界が一致すればoptimalです。

このcutting-planeは有限個の基を増やす厳密法ですが、最悪で指数時間です。大規模な一般劣モジュラ関数への強多項式アルゴリズムという契約は置きません。oracleの劣モジュラ性・決定性は前提で、一部の矛盾は検出しますが全ての違反は検出できません。予算は回数で、任意精度演算の時間・メモリ上限ではありません。

`submodular_greedy_cardinality(n,k,F)` は別の最大化APIです。F(空)=0、単調劣モジュラを前提とし、kをnで切り詰めて限界値最大の要素を順に選びます。k>0なら1−(1−1/k)^k以上、従って1−1/e以上の近似保証。厳密最適化ではありません。返り値はvalue/selected/oracle_callsです。

## 計算量

最小化は最大n!基を追加し得る指数時間。I回のLPとO(I(n log n+nF))の分離。最大化はO(kn(F+k))時間。

## 使用例

```cpp
#include "combinatorial_optimization/submodular_optimization.hpp"
#include <iostream>
using namespace cp;
int main() {
    auto f = [](const std::vector<int> &s) {
        std::vector<bool> in(3);
        int value = 0;
        const int weight[] = {2, -3, 1};
        for (int e : s) {
            in[e] = true;
            value += weight[e];
        }
        value += in[0] != in[1];
        value += in[1] != in[2];
        return BigRational(value);
    };
    auto r = submodular_minimize(3, f);
    std::cout << r.value << " " << r.lower_bound << "\n";
}
```

出力:

```text
-1/1 -1/1
```

## 使用する問題・検証

カットにモジュラ項を足した集合関数の最小化、被覆の個数制限付き近似最大化。 オンラインACは未確認です。

[単体・全探索比較](../../verify/optimization_extensions/unit_test.cpp)、[独立Python比較](../../verify/run_optimization_extensions.py)、[使用例と参照値](../../verify/optimization_extensions/manifest.json)で検証します。検証コマンドと図解との相違は[カテゴリREADME](README.md)を参照してください。
