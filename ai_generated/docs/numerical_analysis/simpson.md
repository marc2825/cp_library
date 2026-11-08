---
title: 複合Simpson積分
documentation_of: //numerical_analysis/integration.hpp
---

# Y-02 複合Simpson積分

[カテゴリ](README.md) / [実装](../../numerical_analysis/integration.hpp) / [操作できる図解](../../visualizations/numerical_analysis/index.html#simpson) / [参考資料](references.md)

隣接3点の放物線を積分し、1・4・2・…・4・1の重みを補償和で集める。

## API・使用条件

`simpson(f,a,b,even_subdivisions)` → IntegrationResult。

分割数は正の偶数。逆向きは負、同一端点は0評価。有限値・等間隔点の分離が条件。固定分割にはerror_estimateを付けない。statusのconvergedは指定の計算を完了した意味。

`simpson(f,a,b,n)` のnは正の偶数の小区間数です。h=(b−a)/nとして、重み1,4,2,…,4,1の和へh/3を掛けます。`evaluations=n+1`、`panels=n/2`。a=bなら評価せず0、a>bなら積分の符号が反転します。

`converged` は指定された等間隔評価の完了を表し、誤差目標の達成を意味しません。`error_estimate` は未設定です。十分滑らかな関数では誤差はO(h⁴)、等間隔の三次以下の多項式には数学的に厳密ですが丸め誤差は残ります。格子点が丸めで重なる場合は `stagnation`。

小さい区間幅と大きい関数値の積は指数を分離して計算し、和には補償和を使います。それでも途中の積や総和が表現不能なら `non_finite` になります。

## 計算量

n分割でn+1評価、O(nF)時間、O(1)領域。

## 使用例

```cpp
#include "numerical_analysis/integration.hpp"
#include <iomanip>
#include <iostream>
using namespace cp::numerical;
int main() {
    std::cout << std::setprecision(6);
    auto result = simpson([](long double x) { return x * x * x; }, 0, 1, 8);
    std::cout << result.value << "\n";
}
```

出力:

```text
0.25
```

## 使用する問題・検証

滑らかな曲線下の面積や固定メッシュ積分。 特定ジャッジでのオンラインACは未確認です。

[検証runner](../../verify/run_numerical_analysis.py)と[境界・比較テスト](../../verify/numerical_analysis/unit_test.cpp)で、解析解・Python Decimal/Fraction・既存の厳密Gauss–Jordanとの比較を行います。図解のサンプルはこのC++使用例の出力と照合します。共通の例外・丸め条件と検証コマンドは[カテゴリREADME](README.md)を参照してください。
