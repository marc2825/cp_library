---
title: 絶対＋相対誤差比較
documentation_of: //numerical_analysis/common.hpp
---

# Y-03 絶対＋相対誤差比較

[カテゴリ](README.md) / [実装](../../numerical_analysis/common.hpp) / [操作できる図解](../../visualizations/numerical_analysis/index.html#tolerance) / [参考資料](references.md)

値を共通の尺度で正規化し、差や相対誤差の積のoverflowを避けて比較する。

## API・使用条件

`ErrorTolerance{absolute,relative}`、`almost_equal(a,b,tolerance)`。

|a−b| ≤ absolute+relative·max(|a|,|b|)。許容誤差は有限・非負。NaNは常にfalse、同一の無限大はtrue。比較は非推移的なのでsort順序や同値関係として使わない。幾何の絶対Toleranceとは別API。

`ErrorTolerance{absolute,relative}` の既定は両方1e−12です。validate()は有限・非負を検査し、threshold(scale)はabsolute+relative*abs(scale)を返します。threshold自体はoverflowで∞となる場合があります。

`almost_equal(a,b,tolerance)` は差のoverflowを避けて比較します。NaNは常にfalse、同じ符号の無限大同士はtrue、有限値と無限大はfalse。0付近には絶対誤差、大きい値には相対誤差を使います。非推移的なので、連想配列の同値判定や比較関数には使えません。幾何の向き・交差判定には[厳密な幾何判定](../computational_geometry/robust_predicates.md)を使用します。

## 計算量

O(1)時間・領域。

## 使用例

```cpp
#include "numerical_analysis/common.hpp"
#include <iomanip>
#include <iostream>
using namespace cp::numerical;
int main() {
    std::cout << std::setprecision(6);
    std::cout << almost_equal(1, 1.0000001L, {1e-8L, 1e-6L}) << "\n";
}
```

出力:

```text
1
```

## 使用する問題・検証

数値結果の比較と反復の停止判定。 特定ジャッジでのオンラインACは未確認です。

[検証runner](../../verify/run_numerical_analysis.py)と[境界・比較テスト](../../verify/numerical_analysis/unit_test.cpp)で、解析解・Python Decimal/Fraction・既存の厳密Gauss–Jordanとの比較を行います。図解のサンプルはこのC++使用例の出力と照合します。共通の例外・丸め条件と検証コマンドは[カテゴリREADME](README.md)を参照してください。
