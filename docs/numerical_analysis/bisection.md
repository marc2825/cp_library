---
title: 二分法による求根
documentation_of: //numerical_analysis/root_finding.hpp
---

# Y-01 二分法による求根

[カテゴリ](README.md) / [実装](../../numerical_analysis/root_finding.hpp) / [操作できる図解](../../visualizations/numerical_analysis/index.html#bisection) / [参考資料](references.md)

符号の異なる端点を維持し区間を半減する。

## API・使用条件

`bisection(f, lower, upper, RootOptions)` → RootResult。

RootResultはstatus・criterion・root/value・反復/評価数を保持。value_toleranceの既定は0（計算された値が0のみ）。Newtonは微分0・停滞・循環を失敗として返し、区間保証なし。二分/Brentは有限の連続関数・符号反転が前提で、x_toleranceによる停止も可能。コールバック例外は伝播。

`bisection(f,lower,upper,RootOptions)`。連続な実関数と昇順の有限区間を与えます。端点が許容残差を満たす場合を除き、両端の関数値が異符号であることが必要です。根が複数ある場合の選択や、不連続関数の零点は保証しません。

両端の符号を保って中点へ縮めます。停止した近似は区間端点のうち残差が小さい側です。`criterion=interval` では区間幅が絶対＋相対許容誤差以下、`residual` では |f(x)| が `value_tolerance` 以下、`exact` では評価値が0です。小さい残差だけでは根の位置の誤差は保証されません。

RootOptionsの既定はx_tolerance={1e−12,1e−12}、value_tolerance=0、max_iterations=128、max_evaluations=512。RootResultはstatus/criterion/root/value/lower/upper/iterations/evaluations/bracketedを持ちます。評価前に終了した場合valueはNaNです。max_iterations/max_evaluations、丸め停止stagnation、非有限評価non_finite、異符号でないinvalid_bracketを区別します。callbackからの例外はそのまま伝播します。

## 計算量

反復I・評価費用FとしてO(IF)、O(1)追加領域。二分法は区間幅を各反復で半減。

## 使用例

```cpp
#include "numerical_analysis/root_finding.hpp"
#include <iomanip>
#include <iostream>
using namespace cp::numerical;
int main() {
    std::cout << std::setprecision(6);
    RootOptions options;
    options.value_tolerance = 1e-12L;
    auto f = [](long double x) { return x * x - 2; };
    const auto result = bisection(f, 0, 2, options);
    std::cout << result.root << "\n";
}
```

出力:

```text
1.41421
```

## 使用する問題・検証

単調な連続量のしきい値、方程式の区間内の根。 特定ジャッジでのオンラインACは未確認です。

[検証runner](../../verify/run_numerical_analysis.py)と[境界・比較テスト](../../verify/numerical_analysis/unit_test.cpp)で、解析解・Python Decimal/Fraction・既存の厳密Gauss–Jordanとの比較を行います。図解のサンプルはこのC++使用例の出力と照合します。共通の例外・丸め条件と検証コマンドは[カテゴリREADME](README.md)を参照してください。
