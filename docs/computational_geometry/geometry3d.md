---
documentation_of: //computational_geometry/geometry3d.hpp
---

# 三次元幾何（H-24）

[カテゴリ](README.md) / [H全22項目](../geometry_extensions/README.md) / [実装](../../computational_geometry/geometry3d.hpp) / [操作できる図解](../../visualizations/geometry_extensions/index.html#geometry3d)

## API・使用条件

`spatial::Point/Line/Segment/Plane/Sphere`、dot/cross/projection/distance/intersection、`spatial::exact::convex_hull`、`spatial::convex_hull`。

ユーザー承認済み範囲。Lineはorigin/direction、Planeはnormal·x=offset。交差は直線×平面、平面×平面、直線×球、平面×球、球×球。距離は点対、直線/線分/平面対点、直線対直線。凸包は厳密有理座標、dimension=-1/0/1/2/3、外向き三角形とvolume。浮動点の凸包も実際のdyadic値で厳密計算する。

C++17、namespace `cp::geometry`。整数・有理・浮動の型を分離しています。`rational::Scalar`は既存BigRational、`rational::Point`は任意精度の有理点で、`exact::Point`（int64）や旧`exact::RationalPoint`から明示変換できます。Boostヘッダが必要ですが追加リンクは不要です。既存128bitのexact APIの契約は変更していません。

浮動APIは既存long double・絶対Toleranceを使用し、有限かつ区別可能な尺度が条件です。丸め・underflowをすべて検出する保証はありません。H-14と厳密有理APIはEPSを使う浮動幾何とは別です。各ヘッダまたはcomputational_geometry/all.hppから利用できます。

## 原理・計算量

3Dの内積・外積で距離と交差を計算し、凸包は可視面を除いて境界と新点を結ぶ。

基本操作O(1)、3D凸包O(n² log n)演算、O(n)領域。任意精度のbit計算量は別。

## 使用例

[コンパイル可能なソース](../../examples/geometry_extensions/geometry3d.cpp)。

```cpp
#include "computational_geometry/geometry3d.hpp"
#include <iomanip>
#include <iostream>
using namespace cp::geometry;
int main() {
    std::cout << std::setprecision(6);
    std::vector<spatial::exact::Point> points;
    for (int x : {0, 1})
        for (int y : {0, 1})
            for (int z : {0, 1})
                points.push_back({x, y, z});
    auto h = spatial::exact::convex_hull(points);
    std::cout << h.dimension << " " << h.faces.size() << " " << rational::approximate(h.volume()) << "\n";
}
```

出力:

```text
3 12 1
```

## 使用する問題・検証

三次元幾何を必要とする領域処理・幾何クエリ。競技用途では入力規模と退化条件を先に確認してください。

[比較検証一覧](../geometry_extensions/README.md#検証)。格子の全列挙・独立数値積分・空円条件・Eulerの式・面の向きと体積などで照合し、Debug/NDEBUG・libstdc++ debug mode・sanitizerを確認します。オンライン提出ACは未確認です。

[一次資料・設計判断](../geometry_extensions/references.md) / [図解の対応範囲](../../visualizations/geometry_extensions/README.md)。
