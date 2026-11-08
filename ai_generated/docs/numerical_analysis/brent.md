---
title: Brent法
documentation_of: //numerical_analysis/root_finding.hpp
---

# Y-01 Brent法

[カテゴリ](README.md) / [実装](../../numerical_analysis/root_finding.hpp) / [操作できる図解](../../visualizations/numerical_analysis/index.html#brent) / [参考資料](references.md)

逆二次補間・割線の候補を進行条件で採否し、不適切な候補では二分法へ戻る。

## API・使用条件

`brent(f, lower, upper, RootOptions)` → RootResult。

RootResultはstatus・criterion・root/value・反復/評価数を保持。value_toleranceの既定は0（計算された値が0のみ）。Newtonは微分0・停滞・循環を失敗として返し、区間保証なし。二分/Brentは有限の連続関数・符号反転が前提で、x_toleranceによる停止も可能。コールバック例外は伝播。

`brent(f,lower,upper,RootOptions)`。二分法と同じ連続性・異符号区間の条件です。端点の関数値による割線補間、3つの異なる関数値による逆二次補間を候補にします。候補が区間の十分内側にあり、以前のステップより十分小さい場合だけ採用し、それ以外は二分します。

異符号区間を保持しながら根の近くで加速します。停止した `root` は残差が小さい側の端点です。`criterion` と許容誤差の意味は[二分法](bisection.md)と共通です。

RootOptionsの既定はx_tolerance={1e−12,1e−12}、value_tolerance=0、max_iterations=128、max_evaluations=512。RootResultはstatus/criterion/root/value/lower/upper/iterations/evaluations/bracketedを持ちます。評価前に終了した場合valueはNaNです。max_iterations/max_evaluations、丸め停止stagnation、非有限評価non_finite、異符号でないinvalid_bracketを区別します。callbackからの例外はそのまま伝播します。

## 計算量

反復I・関数/導関数の評価費用FとしてO(IF)、O(1)追加領域。反復上限はoptionsで指定。

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
    const auto result = brent(f, 0, 2, options);
    std::cout << result.root << "\n";
}
```

出力:

```text
1.41421
```

## 使用する問題・検証

微分を用意できない連続関数の区間求根。 特定ジャッジでのオンラインACは未確認です。

[検証runner](../../verify/run_numerical_analysis.py)と[境界・比較テスト](../../verify/numerical_analysis/unit_test.cpp)で、解析解・Python Decimal/Fraction・既存の厳密Gauss–Jordanとの比較を行います。図解のサンプルはこのC++使用例の出力と照合します。共通の例外・丸め条件と検証コマンドは[カテゴリREADME](README.md)を参照してください。
