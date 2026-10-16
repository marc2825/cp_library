---
title: 重み付き点と半開長方形
documentation_of: //offline_query/rectangle.hpp
---

# 共通の点・長方形型

`cp::WeightedPoint<Coord=long long,Weight=long long>` は `{x,y,weight}`、`cp::Rectangle<Coord=long long>` は `{left,down,right,up}` の公開フィールドを持つ集成体です。

長方形は `[left,right) × [down,up)`。`empty()` は片方でも `left<right` / `down<up` を満たさなければ true を返します。逆向きの境界も空とし、自動で交換しません。O(1) 時間・O(1) 個の値を保持します（値のコピー・比較が O(1) の場合）。

```cpp
#include "offline_query/rectangle.hpp"
int main() {
    cp::WeightedPoint<> p{1, 2, 4};
    cp::Rectangle<> r{0, 0, 3, 3};
    return r.empty() || p.x < r.left; // 0
}
```

座標はコピー可能で `<` が一貫した全順序を与える型。既定構築・`==`・座標の加減算は不要です。NaN を含む浮動小数点数は使わないでください。重みは `Weight(0)`, `+=`, `-` を持つコピー可能な加法可換群で、逆元を含む全中間値が表現可能なことが条件です。既定構築や比較、整数との乗算は不要です。

この型を使う [Sweep Line](sweep_line.md) / [CDQ](cdq_rectangle_sum.md) に usage・計算量・問題例・図解があります。型の検証は [共通ランナー](../../verify/run_offline_query.py) で、既定構築・== のない座標型と二成分の加法群型を含めて実施します。[一次仕様・参考資料](references.md)。
