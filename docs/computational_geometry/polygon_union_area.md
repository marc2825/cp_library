---
documentation_of: //computational_geometry/polygon_union_area.hpp
---

# 一般・凸多角形の和集合面積（H-17）

[カテゴリ](README.md) / [H全22項目](../geometry_extensions/README.md) / [実装](../../computational_geometry/polygon_union_area.hpp) / [操作できる図解](../../visualizations/geometry_extensions/index.html#polygon_union_area)

## API・使用条件

`rational::polygon_union_area(polygons)`、`polygon_union_area(regions)`。`PolygonRegion{outer,holes}`。

各ringは単純、穴はouter内部で互いに内部が重ならないこと。ringの向きは任意。複数領域の重複・境界接触を統合。退化ringは面積0。自己交差ringの塗り規則は扱わない。

C++17、namespace `cp::geometry`。整数・有理・浮動の型を分離しています。`rational::Scalar`は既存BigRational、`rational::Point`は任意精度の有理点で、`exact::Point`（int64）や旧`exact::RationalPoint`から明示変換できます。Boostヘッダが必要ですが追加リンクは不要です。既存128bitのexact APIの契約は変更していません。

浮動APIは既存long double・絶対Toleranceを使用し、有限かつ区別可能な尺度が条件です。丸め・underflowをすべて検出する保証はありません。H-14と厳密有理APIはEPSを使う浮動幾何とは別です。各ヘッダまたはcomputational_geometry/all.hppから利用できます。

## 原理・計算量

全境界を分割し、各有界面の内点がどれかの領域に含まれる場合にだけ面積を足す。

元辺数n、分割後A辺・V頂点としてO(n²+(A+V)A+nA)演算と整列費用、O(n+K+A+V)領域。交点・候補予算はArrangementと共通。

## 使用例

[コンパイル可能なソース](../../examples/geometry_extensions/polygon_union_area.cpp)。

```cpp
#include "computational_geometry/polygon_union_area.hpp"
#include <iomanip>
#include <iostream>
using namespace cp::geometry;
int main() {
    std::cout << std::setprecision(6);
    std::vector<std::vector<rational::Point>> p{{{0, 0}, {3, 0}, {3, 2}, {0, 2}},
                                                {{2, 1}, {4, 1}, {4, 3}, {2, 3}}};
    std::cout << rational::approximate(rational::polygon_union_area(p)) << "\n";
}
```

出力:

```text
9
```

## 使用する問題・検証

一般・凸多角形の和集合面積を必要とする領域処理・幾何クエリ。競技用途では入力規模と退化条件を先に確認してください。

[比較検証一覧](../geometry_extensions/README.md#検証)。格子の全列挙・独立数値積分・空円条件・Eulerの式・面の向きと体積などで照合し、Debug/NDEBUG・libstdc++ debug mode・sanitizerを確認します。オンライン提出ACは未確認です。

[一次資料・設計判断](../geometry_extensions/references.md) / [図解の対応範囲](../../visualizations/geometry_extensions/README.md)。
