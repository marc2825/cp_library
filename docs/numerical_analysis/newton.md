---
title: Newton法
documentation_of: //numerical_analysis/root_finding.hpp
---

# Y-01 Newton法

[カテゴリ](README.md) / [実装](../../numerical_analysis/root_finding.hpp) / [操作できる図解](../../visualizations/numerical_analysis/index.html#newton) / [参考資料](references.md)

接線のx切片へ移り、関数値・微分・進行の停止を調べる。

## API・使用条件

`newton(f, derivative, start, RootOptions)` → RootResult。

RootResultはstatus・criterion・root/value・反復/評価数を保持。value_toleranceの既定は0（計算された値が0のみ）。Newtonは微分0・停滞・循環を失敗として返し、区間保証なし。二分/Brentは有限の連続関数・符号反転が前提で、x_toleranceによる停止も可能。コールバック例外は伝播。

`newton(f,derivative,start,RootOptions)`。x←x−f(x)/f′(x) で更新します。単純根の近傍では速く収束しますが、初期値からの大域収束・区間内への保持は保証しません。`bracketed=false`、lower/upperは最後の評価点です。

停止は評価値0または `value_tolerance` 以下の残差だけです。既定の `value_tolerance=0` は厳しいため、丸めで更新できなくなれば近い根でも `stagnation` になります。用途に合う残差許容値を設定してください。`x_tolerance` はNewtonでは使いません。微小な更新だけを根の証拠にしない設計です。導関数0は `zero_derivative`、循環は反復予算で停止します。導関数評価も `evaluations` に数えます。

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
    const auto result = newton(f, [](long double x) { return 2 * x; }, 1, options);
    std::cout << result.root << "\n";
}
```

出力:

```text
1.41421
```

## 使用する問題・検証

微分できる滑らかな式の局所求根。 特定ジャッジでのオンラインACは未確認です。

[検証runner](../../verify/run_numerical_analysis.py)と[境界・比較テスト](../../verify/numerical_analysis/unit_test.cpp)で、解析解・Python Decimal/Fraction・既存の厳密Gauss–Jordanとの比較を行います。図解のサンプルはこのC++使用例の出力と照合します。共通の例外・丸め条件と検証コマンドは[カテゴリREADME](README.md)を参照してください。
