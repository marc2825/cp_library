---
title: 有限上下限付き整数線形計画
documentation_of: //combinatorial_optimization/integer_program.hpp
---

# O-23 有限上下限付き整数線形計画

[カテゴリ](README.md) / [実装](../../combinatorial_optimization/integer_program.hpp) / [操作できる図解](../../visualizations/optimization_extensions/index.html#integer_program) / [一次資料](references.md)

LP緩和の目的値を上界とし、分数の変数をfloor/ceilで分けて探索する。

## API・条件

ユーザー承認済みの範囲は、有限の整数上下限がある線形計画です。係数A,b,cはBigRational、lower/upperはBigInt。c·xを最大化し、A·x≤b、lower≤x≤upper、全成分整数を要求します。下限は負でも構いません。

各探索ノードでx=lower+zとして非負変数へ移し、[厳密Simplex](linear_program.md)で連続緩和を解きます。緩和が実行不能なら枝を捨て、目的上界が既存整数解以下なら打ち切ります。分数の変数を見つけたらxⱼ≤floor(v)とxⱼ≥ceil(v)へ分岐します。負の分数にも数学的なfloor/ceilを使います。深さ優先探索は明示stackで行います。

IntegerProgramOptionsはmax_nodes（既定10000）とsimplex。結果はstatus/has_solution/value/solution/upper_bound/nodes。statusはoptimal/infeasible/node_limit/lp_limit。has_solution=falseのvalueは意味を持ちません。upper_boundは未探索部分も含む上界で、根の緩和すら完了していない場合などはnulloptです。予算終了時の候補を最適解とは扱いません。optimalならupper_bound=valueです。

lower>upperの成分があればinfeasible。空変数系も扱います。ノード予算は緩和LPを解いた回数で、上界による枝刈りは数えません。LP自身も予算で停止できます。一般の混合整数計画・無限上下限・カット生成や商用ソルバ並みの性能は対象外です。

## 計算量

探索ノードNでN回の有理Simplex。最悪指数時間。境界と有理演算のbit成長は別途必要。

## 使用例

```cpp
#include "combinatorial_optimization/integer_program.hpp"
#include <iostream>
using namespace cp;
int main() {
    auto r = bounded_integer_program({{1, 1}}, {BigRational(7, 2)}, {3, 2}, {0, 0}, {4, 4});
    std::cout << r.value << "\n";
    for (std::size_t i = 0; i < r.solution.size(); i++)
        std::cout << (i ? " " : "") << r.solution[i];
    std::cout << "\n";
}
```

出力:

```text
9/1
3 0
```

## 使用する問題・検証

小規模な整数資源配分、有限範囲の線形制約、LP緩和で枝を削れる探索。 オンラインACは未確認です。

[単体・全探索比較](../../verify/optimization_extensions/unit_test.cpp)、[独立Python比較](../../verify/run_optimization_extensions.py)、[使用例と参照値](../../verify/optimization_extensions/manifest.json)で検証します。検証コマンドと図解との相違は[カテゴリREADME](README.md)を参照してください。
