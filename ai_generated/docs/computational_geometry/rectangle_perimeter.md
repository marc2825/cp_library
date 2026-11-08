---
documentation_of: //computational_geometry/rectangle_union.hpp
---

# 長方形の和集合周長（H-02）

[カテゴリ](README.md) / [H全22項目](../geometry_extensions/README.md) / [実装](../../computational_geometry/rectangle_union.hpp) / [操作できる図解](../../visualizations/geometry_extensions/index.html#rectangle_perimeter)

## API・使用条件

`exact::rectangle_union_perimeter(rectangles)`、`rectangle_union_measure`。

面積と同じ入力。同じxでは追加を削除より先に処理し、接する辺を内部境界として消す。

C++17、namespace `cp::geometry`。整数・有理・浮動の型を分離しています。`rational::Scalar`は既存BigRational、`rational::Point`は任意精度の有理点で、`exact::Point`（int64）や旧`exact::RationalPoint`から明示変換できます。Boostヘッダが必要ですが追加リンクは不要です。既存128bitのexact APIの契約は変更していません。

浮動APIは既存long double・絶対Toleranceを使用し、有限かつ区別可能な尺度が条件です。丸め・underflowをすべて検出する保証はありません。H-14と厳密有理APIはEPSを使う浮動幾何とは別です。各ヘッダまたはcomputational_geometry/all.hppから利用できます。

## 原理・計算量

被覆の連結区間数から水平境界を、イベントでの被覆長変化から垂直境界を数える。

O(n log n)回の整数演算、O(n)領域。

## 使用例

[コンパイル可能なソース](../../examples/geometry_extensions/rectangle_perimeter.cpp)。

```cpp
#include "computational_geometry/rectangle_union.hpp"
#include <iomanip>
#include <iostream>
using namespace cp::geometry;
int main() {
    std::cout << std::setprecision(6);
    std::cout << exact::rectangle_union_perimeter({{0, 0, 3, 2}, {2, 1, 4, 3}}) << "\n";
}
```

出力:

```text
14
```

## 使用する問題・検証

長方形の和集合周長を必要とする領域処理・幾何クエリ。競技用途では入力規模と退化条件を先に確認してください。

[比較検証一覧](../geometry_extensions/README.md#検証)。格子の全列挙・独立数値積分・空円条件・Eulerの式・面の向きと体積などで照合し、Debug/NDEBUG・libstdc++ debug mode・sanitizerを確認します。オンライン提出ACは未確認です。

[一次資料・設計判断](../geometry_extensions/references.md) / [図解の対応範囲](../../visualizations/geometry_extensions/README.md)。
