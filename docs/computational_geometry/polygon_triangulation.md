---
documentation_of: //computational_geometry/polygon_triangulation.hpp
---

# 多角形の三角形分割（H-23）

[カテゴリ](README.md) / [H全22項目](../geometry_extensions/README.md) / [実装](../../computational_geometry/polygon_triangulation.hpp) / [操作できる図解](../../visualizations/geometry_extensions/index.html#polygon_triangulation)

## API・使用条件

`rational::triangulate_polygon(polygon)` → vector<array<size_t,3>>。

穴なし単純ring。CW可、連続重複・閉点再掲・辺上の冗長点を除去し、元入力添字のCCW三角形を返す。空・全共線は空。自己交差・重なる辺は拒否。穴のbridge生成は含めない。

C++17、namespace `cp::geometry`。整数・有理・浮動の型を分離しています。`rational::Scalar`は既存BigRational、`rational::Point`は任意精度の有理点で、`exact::Point`（int64）や旧`exact::RationalPoint`から明示変換できます。Boostヘッダが必要ですが追加リンクは不要です。既存128bitのexact APIの契約は変更していません。

浮動APIは既存long double・絶対Toleranceを使用し、有限かつ区別可能な尺度が条件です。丸め・underflowをすべて検出する保証はありません。H-14と厳密有理APIはEPSを使う浮動幾何とは別です。各ヘッダまたはcomputational_geometry/all.hppから利用できます。

## 原理・計算量

凸な耳の三角形内に他の頂点がないことを確かめて切り取り、両隣だけ耳候補を更新する。

O(n²)演算、O(n)追加領域。

## 使用例

[コンパイル可能なソース](../../examples/geometry_extensions/polygon_triangulation.cpp)。

```cpp
#include "computational_geometry/polygon_triangulation.hpp"
#include <iomanip>
#include <iostream>
using namespace cp::geometry;
int main() {
    std::cout << std::setprecision(6);
    std::vector<rational::Point> p{{0, 0}, {4, 0}, {4, 4}, {2, 2}, {0, 4}};
    auto triangles = rational::triangulate_polygon(p);
    cp::BigRational area;
    for (auto t : triangles)
        area += cp::abs(rational::cross(p[t[0]], p[t[1]], p[t[2]])) / 2;
    std::cout << triangles.size() << " " << rational::approximate(area) << "\n";
}
```

出力:

```text
3 12
```

## 使用する問題・検証

多角形の三角形分割を必要とする領域処理・幾何クエリ。競技用途では入力規模と退化条件を先に確認してください。

[比較検証一覧](../geometry_extensions/README.md#検証)。格子の全列挙・独立数値積分・空円条件・Eulerの式・面の向きと体積などで照合し、Debug/NDEBUG・libstdc++ debug mode・sanitizerを確認します。オンライン提出ACは未確認です。

[一次資料・設計判断](../geometry_extensions/references.md) / [図解の対応範囲](../../visualizations/geometry_extensions/README.md)。
