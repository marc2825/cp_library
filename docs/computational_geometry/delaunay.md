---
documentation_of: //computational_geometry/delaunay.hpp
---

# Delaunay三角形分割（H-20）

[カテゴリ](README.md) / [H全22項目](../geometry_extensions/README.md) / [実装](../../computational_geometry/delaunay.hpp) / [操作できる図解](../../visualizations/geometry_extensions/index.html#delaunay)

## API・使用条件

`rational::delaunay_triangulation(points)` → vertices/original_to_vertex/triangles/edges。

任意精度有理座標。重複は統合し入力からの対応を返す。全共線は連続点間の辺、三角形なし。共円ではstrictなincircle違反だけをflipし、決定的な分割の一つを返す。人工的な巨大三角形を使わない。

C++17、namespace `cp::geometry`。整数・有理・浮動の型を分離しています。`rational::Scalar`は既存BigRational、`rational::Point`は任意精度の有理点で、`exact::Point`（int64）や旧`exact::RationalPoint`から明示変換できます。Boostヘッダが必要ですが追加リンクは不要です。既存128bitのexact APIの契約は変更していません。

浮動APIは既存long double・絶対Toleranceを使用し、有限かつ区別可能な尺度が条件です。丸め・underflowをすべて検出する保証はありません。H-14と厳密有理APIはEPSを使う浮動幾何とは別です。各ヘッダまたはcomputational_geometry/all.hppから利用できます。

## 原理・計算量

凸包の三角形分割へ点を挿入し、空円条件に違反する対角線を局所的に交換する。

単純な点位置走査とLawson flipで最悪O(n² log n)演算、O(n)領域。bit計算量は別。

## 使用例

[コンパイル可能なソース](../../examples/geometry_extensions/delaunay.cpp)。

```cpp
#include "computational_geometry/delaunay.hpp"
#include <iomanip>
#include <iostream>
using namespace cp::geometry;
int main() {
    std::cout << std::setprecision(6);
    auto d = rational::delaunay_triangulation({{0, 0}, {4, 0}, {4, 4}, {0, 4}, {1, 1}});
    std::cout << d.vertices.size() << " " << d.triangles.size() << " " << d.edges.size() << "\n";
}
```

出力:

```text
5 4 8
```

## 使用する問題・検証

Delaunay三角形分割を必要とする領域処理・幾何クエリ。競技用途では入力規模と退化条件を先に確認してください。

[比較検証一覧](../geometry_extensions/README.md#検証)。格子の全列挙・独立数値積分・空円条件・Eulerの式・面の向きと体積などで照合し、Debug/NDEBUG・libstdc++ debug mode・sanitizerを確認します。オンライン提出ACは未確認です。

[一次資料・設計判断](../geometry_extensions/references.md) / [図解の対応範囲](../../visualizations/geometry_extensions/README.md)。
