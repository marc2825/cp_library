---
documentation_of: //computational_geometry/convex_polygon_intersection.hpp
---

# 凸多角形の共通部分（H-13）

[カテゴリ](README.md) / [H全22項目](../geometry_extensions/README.md) / [実装](../../computational_geometry/convex_polygon_intersection.hpp) / [操作できる図解](../../visualizations/geometry_extensions/index.html#convex_polygon_intersection)

## API・使用条件

`rational::convex_polygon_intersection(a,b)`。

入力の凸頂点集合をstrict CCWへ正規化する。戻り値は空・一点・線分・凸多角形を区別できる頂点列。整数・旧128bit有理点はrational::Pointへ明示変換できる。

C++17、namespace `cp::geometry`。整数・有理・浮動の型を分離しています。`rational::Scalar`は既存BigRational、`rational::Point`は任意精度の有理点で、`exact::Point`（int64）や旧`exact::RationalPoint`から明示変換できます。Boostヘッダが必要ですが追加リンクは不要です。既存128bitのexact APIの契約は変更していません。

浮動APIは既存long double・絶対Toleranceを使用し、有限かつ区別可能な尺度が条件です。丸め・underflowをすべて検出する保証はありません。H-14と厳密有理APIはEPSを使う浮動幾何とは別です。各ヘッダまたはcomputational_geometry/all.hppから利用できます。

## 原理・計算量

片方の各辺の左側で他方を切断し、点・線分に退化した交差も厳密に残す。

入力n,mの凸包構築を含めO(n log n+m log m+nm)、O(n+m)追加領域。

## 使用例

[コンパイル可能なソース](../../examples/geometry_extensions/convex_polygon_intersection.cpp)。

```cpp
#include "computational_geometry/convex_polygon_intersection.hpp"
#include <iomanip>
#include <iostream>
using namespace cp::geometry;
int main() {
    std::cout << std::setprecision(6);
    auto p = rational::convex_polygon_intersection({{0, 0}, {3, 0}, {3, 3}, {0, 3}},
                                                   {{2, -1}, {4, -1}, {4, 2}, {2, 2}});
    std::cout << p.size() << " " << rational::approximate(cp::abs(rational::signed_area2(p)) / 2) << "\n";
}
```

出力:

```text
4 2
```

## 使用する問題・検証

凸多角形の共通部分を必要とする領域処理・幾何クエリ。競技用途では入力規模と退化条件を先に確認してください。

[比較検証一覧](../geometry_extensions/README.md#検証)。格子の全列挙・独立数値積分・空円条件・Eulerの式・面の向きと体積などで照合し、Debug/NDEBUG・libstdc++ debug mode・sanitizerを確認します。オンライン提出ACは未確認です。

[一次資料・設計判断](../geometry_extensions/references.md) / [図解の対応範囲](../../visualizations/geometry_extensions/README.md)。
