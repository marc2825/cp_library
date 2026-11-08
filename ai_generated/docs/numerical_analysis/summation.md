---
title: Kahan和・Neumaier補償和
documentation_of: //numerical_analysis/stable_arithmetic.hpp
---

# Y-03 Kahan和・Neumaier補償和

[カテゴリ](README.md) / [実装](../../numerical_analysis/stable_arithmetic.hpp) / [操作できる図解](../../visualizations/numerical_analysis/index.html#summation) / [参考資料](references.md)

加算で失われた下位桁を別に保持し、次の加算または最後の結果へ補う。

## API・使用条件

`KahanSum` / `CompensatedSum` のadd/value/clear、`compensated_sum(first,last)`。

有限値。非有限入力はinvalid_argument、途中overflowはoverflow_error。失敗したaddは状態を保つ。どちらも全ての順序で正しく丸められた和を保証しない。fast-math不可。

`KahanSum` と `CompensatedSum` は add(long double)、value()、clear() を公開します。`compensated_sum(first,last)` は反復子区間のNeumaier補償和です。どちらも空和は0、有限入力のみ。内部overflowでは例外を投げ、失敗したaddは元の状態を維持します。

Kahanは前回加算で失われた量を次の入力から引きます。Neumaierは累積値と入力のどちらが大きいかに応じて誤差を回収するため、1e30,1,−1e30のような例でも1を保持します。ただし任意順序・任意入力で正確な最終丸めを保証しません。`-ffast-math` のような式の再結合を許す設定では補償式が壊れます。

## 計算量

1追加O(1)、列nでO(n)、O(1)領域。

## 使用例

```cpp
#include "numerical_analysis/stable_arithmetic.hpp"
#include <iomanip>
#include <iostream>
using namespace cp::numerical;
int main() {
    std::cout << std::setprecision(6);
    KahanSum kahan;
    CompensatedSum compensated;
    for (auto x : {1e30L, 1.L, -1e30L}) {
        kahan.add(x);
        compensated.add(x);
    }
    std::cout << kahan.value() << " " << compensated.value() << "\n";
}
```

出力:

```text
0 1
```

## 使用する問題・検証

多数の面積・確率・重みの合算。 特定ジャッジでのオンラインACは未確認です。

[検証runner](../../verify/run_numerical_analysis.py)と[境界・比較テスト](../../verify/numerical_analysis/unit_test.cpp)で、解析解・Python Decimal/Fraction・既存の厳密Gauss–Jordanとの比較を行います。図解のサンプルはこのC++使用例の出力と照合します。共通の例外・丸め条件と検証コマンドは[カテゴリREADME](README.md)を参照してください。
