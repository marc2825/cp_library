---
title: Convex Polygon Contains
documentation_of: //computational_geometry/polygon.hpp
---
# Convex Polygon Contains

[カテゴリ・共通仕様](README.md) / [操作できる図解](../../visualizations/computational_geometry/index.html#convex) / [一次資料・設計判断](references.md)

`ConvexPolygon(points)`は渡した点集合のstrictな凸包を構築します。`vertices()`で正規化済みの反時計回りの頂点列をconst参照、`contains(q)`でLocationを返します。凹多角形を渡すとその凸包の内部を判定するため、凹領域の判定にはpoint_in_polygonを使います。

すでにstrictなCCW凸包があるなら`convex_polygon_contains(hull,q)`で構築を省略できます。先頭からの扇形を二分探索します。問い合わせごとに全頂点を検査しないため、**反時計回り・共線の途中点なし・凸・末尾の重複なし**が呼出し側の前提です。開始頂点は任意。空・一点・線分の扱いは一般多角形版と同じです。

浮動小数点版クラスは`ConvexPolygon(points,Tolerance)`で誤差を保存し、構築と問い合わせで共用します。free関数は末尾Toleranceを指定します。曖昧な小さな角がないことも数値上の条件になります。

## usage

```cpp
#include <iostream>
#include <vector>
#include "computational_geometry/polygon.hpp"
namespace e=cp::geometry::exact;
namespace f=cp::geometry::floating;
int main(){
    e::ConvexPolygon p({{0,0},{3,0},{3,3},{0,3},{1,1}});
    std::cout<<static_cast<int>(p.contains({1,1}))<<' '<<static_cast<int>(p.contains({4,1}))<<'\n';
}
```

[実行できる例](../../examples/computational_geometry/convex_polygon_contains.cpp)。整数・浮動小数点の型とnamespaceは混ぜず、使用する方を明示してください。

## 条件・計算量

構築 O(n log n)・問い合わせ O(log h)・保持 O(h)。整数はGCC/Clangの検査付き128ビット演算です。有理数の正規化・比較にはO(log M)の整数演算を要し、上の計算量はこれを基本演算一回として数えています。中間値の範囲・浮動小数点の誤差と例外は[共通仕様](README.md#数値と境界の契約)を参照してください。

## 使用する問題・検証

凸領域への大量の所属問い合わせ。ローカルでは[検証スクリプト](../../verify/run_computational_geometry.py)から使用例・独立したFraction／総当たりの解・境界と退化ケースを確認します。全体の検証範囲は[カテゴリ索引](README.md#ローカル検証)に記載しています。オンラインACは未確認です。
