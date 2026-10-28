---
title: 二段階Simplex・線形計画
documentation_of: //combinatorial_optimization/linear_program.hpp
---

# O-01 二段階Simplex・線形計画

[カテゴリ](README.md) / [実装](../../combinatorial_optimization/linear_program.hpp) / [操作できる図解](../../visualizations/optimization_extensions/index.html#linear_program) / [一次資料](references.md)

人工変数の総和を最小化し、実行可能基底を得て元の目的を改善する。

## API・条件

`linear_program<T>(A,b,c,SimplexOptions)` は c·x を最大化します。Aの各行とcの長さが変数数n、bの長さが制約数mです。通常は x≥0。`free_linear_program` は x=x⁺−x⁻ と変換して符号自由な変数を扱います。最小化はcの符号を反転してください。等式は互いに逆向きの2不等式で指定できます。

TはBigRationalまたは浮動型。整数型は除算が体にならないため拒否します。空制約・0変数・冗長な制約も扱います。入力は変更しません。

| 結果 | 意味・確認できる条件 |
|---|---|
| optimal | primalが実行可能、dual≥0、Aᵀdual≥c、b·dual=c·primal=value |
| infeasible | dualがFarkasベクトル。dual≥0、Aᵀdual≥0、b·dual<0 |
| unbounded | primalは実行可能点、ray≥0、A·ray≤0、c·ray>0 |
| iteration_limit | pivot予算を使い切った。has_primalなら実行可能点を保持 |
| numerical_failure | 浮動計算の非有限値・基底の実行可能性喪失を検出 |

free版では上のAᵀdual≥c/≥0がそれぞれ等式となり、primal/rayの非負条件はありません。原座標で返します。低次元LPとは証明の返却範囲が異なります。

`LinearProgramResult<T>` は status、has_primal、value、primal、dual、ray、pivots を持ちます。意味のない証明ベクトルは空です。Phase Iでの予算終了時は元の制約を満たすとは限らないためhas_primal=false。Phase IIの途中ならtrueです。

既定max_pivots=100000、epsilon=1e−12。BigRationalでは比較が厳密なのでepsilonは判定に使いません。浮動版のepsilonは絶対値で、入力のスケーリング・条件数に注意が必要です。数値的なoptimal/infeasible/unboundedを厳密証明とは扱わず、原入力への残差を確認してください。小さい係数を0と扱うことで分類が変わる場合もあります。

負の右辺には人工変数を置き、人工変数の総和を0にしてから元の目的へ移ります。実行不能判定に巨大な有限ペナルティを使いません。入る変数と比率同値で出る変数は添字の小さいものを選ぶBland規則です。厳密演算では循環を防ぎますが、指数回のpivotを要する入力があります。

## 計算量

m制約・n変数・P pivotでO((P+1)(m+1)(n+m+1))体演算、O((m+1)(n+m+1))領域。Bland規則でも最悪指数時間。

## 使用例

```cpp
#include "combinatorial_optimization/linear_program.hpp"
#include <iostream>
using namespace cp;
int main() {
    auto r = linear_program<BigRational>({{1, 1}, {1, 0}, {0, 1}}, {4, 2, 3}, {3, 2});
    std::cout << lp_status_name(r.status) << " " << r.value << "\n";
    for (std::size_t i = 0; i < r.primal.size(); i++)
        std::cout << (i ? " " : "") << r.primal[i];
    std::cout << "\n";
}
```

出力:

```text
optimal 10/1
2/1 2/1
```

## 使用する問題・検証

資源配分、混合戦略のLP、連続緩和と双対証明。 オンラインACは未確認です。

[単体・全探索比較](../../verify/optimization_extensions/unit_test.cpp)、[独立Python比較](../../verify/run_optimization_extensions.py)、[使用例と参照値](../../verify/optimization_extensions/manifest.json)で検証します。検証コマンドと図解との相違は[カテゴリREADME](README.md)を参照してください。
