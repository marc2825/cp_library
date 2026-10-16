---
title: Convex Polygon Cut
documentation_of: //computational_geometry/convex_cut.hpp
---
# Convex Polygon Cut

[カテゴリ・共通仕様](README.md) / [操作できる図解](../../visualizations/computational_geometry/index.html#cut) / [一次資料・設計判断](references.md)

`convex_polygon_cut(polygon,line)`は有向直線a→bの**左側の閉半平面**を残します。頂点は凸多角形の周上順で、時計回り・反時計回りのどちらでもかまいません。辺が直線をまたぐ場合にだけ交点を挿入し、境界点は残します。隣接する同一点と末尾の閉点を除去します。

整数Point版はvector<RationalPoint>を返し、RationalPoint列を受けるオーバーロードでもう一度切れます。小数へ変換する必要はありません。空・一点・線分の結果も許し、入力が直線上なら境界をそのまま残します。凸性を自動検査せず、出力がstrictな凸包になることも保証しません。

浮動小数点版は末尾Toleranceで境界の内側・外側を分け、隣接点の近似等値にも同じEPSを使います。EPS内の点は直線上へ投影せず残るため、厳密な半平面からわずかに出る場合があります。

## usage

```cpp
#include <iostream>
#include <vector>
#include "computational_geometry/convex_cut.hpp"
namespace e=cp::geometry::exact;
namespace f=cp::geometry::floating;
int main(){
    std::vector<e::Point> p{{0,0},{2,0},{2,2},{0,2}};
    auto q=e::convex_polygon_cut(p,e::Line({0,1},{3,0}));
    std::cout<<e::polygon_area(q).str()<<'\n';
    q=e::convex_polygon_cut(q,e::Line({1,0},{1,1}));
    std::cout<<e::polygon_area(q).str()<<'\n';
}
```

[実行できる例](../../examples/computational_geometry/convex_polygon_cut.cpp)。整数・浮動小数点の型とnamespaceは混ぜず、使用する方を明示してください。

## 条件・計算量

O(h) 時間・O(h) 出力領域。整数はGCC/Clangの検査付き128ビット演算です。有理数の正規化・比較にはO(log M)の整数演算を要し、上の計算量はこれを基本演算一回として数えています。中間値の範囲・浮動小数点の誤差と例外は[共通仕様](README.md#数値と境界の契約)を参照してください。

## 使用する問題・検証

CGL_4_C。ローカルでは[検証スクリプト](../../verify/run_computational_geometry.py)から使用例・独立したFraction／総当たりの解・境界と退化ケースを確認します。全体の検証範囲は[カテゴリ索引](README.md#ローカル検証)に記載しています。オンラインACは未確認です。
