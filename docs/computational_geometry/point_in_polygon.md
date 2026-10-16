---
title: Point in Polygon
documentation_of: //computational_geometry/polygon.hpp
---
# Point in Polygon

[カテゴリ・共通仕様](README.md) / [操作できる図解](../../visualizations/computational_geometry/index.html#inside) / [一次資料・設計判断](references.md)

`point_in_polygon(polygon,q)`は`Location::outside=0 / boundary=1 / inside=2`を返します。最初に閉辺への所属を確認し、上下の横切りを半開区間に分けたwinding数を数えます。凹多角形、時計回り／反時計回りの両方に使えます。

入力は周上順の単純多角形。空はoutside、一点・線分はその閉集合上だけboundaryです。頂点や水平辺の上もboundary。浮動小数点版では末尾Toleranceを指定でき、境界の外積と座標範囲に絶対EPSを使います。浮動小数点の射線横切りではyの比較はそのまま行います。整数版なら境界を厳密に分類できます。

## usage

```cpp
#include <iostream>
#include <vector>
#include "computational_geometry/polygon.hpp"
namespace e=cp::geometry::exact;
namespace f=cp::geometry::floating;
int main(){
    std::vector<e::Point> p{{0,0},{3,0},{3,3},{0,3}};
    for(auto q:std::vector<e::Point>{{1,1},{3,2},{4,1}})
        std::cout<<static_cast<int>(e::point_in_polygon(p,q))<<'\n';
}
```

[実行できる例](../../examples/computational_geometry/point_in_polygon.cpp)。整数・浮動小数点の型とnamespaceは混ぜず、使用する方を明示してください。

## 条件・計算量

O(n) 時間・O(1) 追加領域。整数はGCC/Clangの検査付き128ビット演算です。有理数の正規化・比較にはO(log M)の整数演算を要し、上の計算量はこれを基本演算一回として数えています。中間値の範囲・浮動小数点の誤差と例外は[共通仕様](README.md#数値と境界の契約)を参照してください。

## 使用する問題・検証

CGL_3_C。ローカルでは[検証スクリプト](../../verify/run_computational_geometry.py)から使用例・独立したFraction／総当たりの解・境界と退化ケースを確認します。全体の検証範囲は[カテゴリ索引](README.md#ローカル検証)に記載しています。オンラインACは未確認です。
