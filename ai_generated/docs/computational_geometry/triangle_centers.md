---
documentation_of: //computational_geometry/triangle_centers.hpp
---

# 三角形の外接円・内接円・垂心（H-04）

[カテゴリ](README.md) / [H全22項目](../geometry_extensions/README.md) / [実装](../../computational_geometry/triangle_centers.hpp) / [操作できる図解](../../visualizations/geometry_extensions/index.html#triangle_centers)

## API・使用条件

`floating::triangle_centers(a,b,c,tol)`、`circumcircle`、`incircle`、`orthocenter`。

optional。Toleranceで面積0と判定した三角形はnullopt。中心一式は外接円・内接円・重心・垂心を保持。MECと外接円の計算を共有。

C++17、namespace `cp::geometry`。整数・有理・浮動の型を分離しています。`rational::Scalar`は既存BigRational、`rational::Point`は任意精度の有理点で、`exact::Point`（int64）や旧`exact::RationalPoint`から明示変換できます。Boostヘッダが必要ですが追加リンクは不要です。既存128bitのexact APIの契約は変更していません。

浮動APIは既存long double・絶対Toleranceを使用し、有限かつ区別可能な尺度が条件です。丸め・underflowをすべて検出する保証はありません。H-14と厳密有理APIはEPSを使う浮動幾何とは別です。各ヘッダまたはcomputational_geometry/all.hppから利用できます。

## 原理・計算量

外心は2本の垂直二等分線、内心は辺長の重み付き平均、垂心はA+B+C−2O。

O(1)。

## 使用例

[コンパイル可能なソース](../../examples/geometry_extensions/triangle_centers.cpp)。

```cpp
#include "computational_geometry/triangle_centers.hpp"
#include <iomanip>
#include <iostream>
using namespace cp::geometry;
int main() {
    std::cout << std::setprecision(6);
    const auto c = *floating::triangle_centers({0, 0}, {4, 0}, {0, 3});
    std::cout << c.circumcircle.radius << " " << c.incircle.radius << " " << c.orthocenter.x << " "
              << c.orthocenter.y << "\n";
}
```

出力:

```text
2.5 1 0 0
```

## 使用する問題・検証

三角形の外接円・内接円・垂心を必要とする領域処理・幾何クエリ。競技用途では入力規模と退化条件を先に確認してください。

[比較検証一覧](../geometry_extensions/README.md#検証)。格子の全列挙・独立数値積分・空円条件・Eulerの式・面の向きと体積などで照合し、Debug/NDEBUG・libstdc++ debug mode・sanitizerを確認します。オンライン提出ACは未確認です。

[一次資料・設計判断](../geometry_extensions/references.md) / [図解の対応範囲](../../visualizations/geometry_extensions/README.md)。
