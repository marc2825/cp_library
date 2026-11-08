---
title: 安定な二次方程式
documentation_of: //numerical_analysis/quadratic.hpp
---

# Y-01 安定な二次方程式

[カテゴリ](README.md) / [実装](../../numerical_analysis/quadratic.hpp) / [操作できる図解](../../visualizations/numerical_analysis/index.html#quadratic) / [参考資料](references.md)

符号の確定した判別式とq=−(b+sign(b)√D)/2から、q/aとc/qを求めて桁落ちを避ける。

## API・使用条件

`quadratic_roots(a,b,c)` → status / all_real / roots。

a=0の一次・定数へ退化可。重根は1回、昇順。判別式の符号は入力の二進値をBigRationalへ変換して厳密判定。根は近似値で、normalizationのunderflow・表現範囲外はnon_finite。

`quadratic_roots(a,b,c)` は a x²+b x+c=0 の実根を昇順で返します。一次式・定数式へ退化しても呼び出せます。全0では `all_real=true`、実根がなければ空列、二重根は1回です。相異なる根でも丸め後の近似値が同じになる場合があります。

判別式の符号は入力の二進値をBigRationalへ変換して厳密に判定します。正なら係数を最大絶対値でスケーリングし、q=−(b+copysign(√D,b))/2、根をq/aとc/qで求めます。通常の解の公式で生じる、近い二数の減算を避けます。係数のスケーリングや判別式の近似がunderflowした場合は `non_finite`。近似根の精度を一律に保証するAPIではありません。Boostヘッダが必要です。

## 計算量

O(1)回の多倍長演算、bit費用は入力の指数と仮数に依存。

## 使用例

```cpp
#include "numerical_analysis/quadratic.hpp"
#include <iomanip>
#include <iostream>
using namespace cp::numerical;
int main() {
    std::cout << std::setprecision(6);
    const auto result = quadratic_roots(1, -5, 6);
    for (std::size_t i = 0; i < result.roots.size(); ++i)
        std::cout << (i ? " " : "") << result.roots[i];
    std::cout << "\n";
}
```

出力:

```text
2 3
```

## 使用する問題・検証

放物線との交点、運動の到達時刻。 特定ジャッジでのオンラインACは未確認です。

[検証runner](../../verify/run_numerical_analysis.py)と[境界・比較テスト](../../verify/numerical_analysis/unit_test.cpp)で、解析解・Python Decimal/Fraction・既存の厳密Gauss–Jordanとの比較を行います。図解のサンプルはこのC++使用例の出力と照合します。共通の例外・丸め条件と検証コマンドは[カテゴリREADME](README.md)を参照してください。
