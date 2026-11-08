---
documentation_of: //computational_geometry/triangle_point_count.hpp
---

# 三角形内の点数クエリ（H-15）

[カテゴリ](README.md) / [H全22項目](../geometry_extensions/README.md) / [実装](../../computational_geometry/triangle_point_count.hpp) / [操作できる図解](../../visualizations/geometry_extensions/index.html#triangle_point_count)

## API・使用条件

`exact::TrianglePointCounter(points).count(a,b,c,include_boundary=true)`。

静的なint64点集合。重複点も個別に数える。CW可。退化三角形は境界込みなら線分／点上、strictなら0。

C++17、namespace `cp::geometry`。整数・有理・浮動の型を分離しています。`rational::Scalar`は既存BigRational、`rational::Point`は任意精度の有理点で、`exact::Point`（int64）や旧`exact::RationalPoint`から明示変換できます。Boostヘッダが必要ですが追加リンクは不要です。既存128bitのexact APIの契約は変更していません。

浮動APIは既存long double・絶対Toleranceを使用し、有限かつ区別可能な尺度が条件です。丸め・underflowをすべて検出する保証はありません。H-14と厳密有理APIはEPSを使う浮動幾何とは別です。各ヘッダまたはcomputational_geometry/all.hppから利用できます。

## 原理・計算量

三角形の各有向辺とKD-treeの矩形4隅を比較し、完全に内側なら個数をまとめて加える。

構築O(n log n)、領域O(n)、queryは最悪O(n)。KD-treeのboxを内外判定して枝刈りするが対数時間は保証しない。

## 使用例

[コンパイル可能なソース](../../examples/geometry_extensions/triangle_point_count.cpp)。

```cpp
#include "computational_geometry/triangle_point_count.hpp"
#include <iomanip>
#include <iostream>
using namespace cp::geometry;
int main() {
    std::cout << std::setprecision(6);
    exact::TrianglePointCounter p({{0, 0}, {1, 1}, {2, 0}, {3, 3}, {1, 0}});
    std::cout << p.count({0, 0}, {3, 0}, {0, 3}) << " " << p.count({0, 0}, {3, 0}, {0, 3}, false) << "\n";
}
```

出力:

```text
4 1
```

## 使用する問題・検証

三角形内の点数クエリを必要とする領域処理・幾何クエリ。競技用途では入力規模と退化条件を先に確認してください。

[比較検証一覧](../geometry_extensions/README.md#検証)。格子の全列挙・独立数値積分・空円条件・Eulerの式・面の向きと体積などで照合し、Debug/NDEBUG・libstdc++ debug mode・sanitizerを確認します。オンライン提出ACは未確認です。

[一次資料・設計判断](../geometry_extensions/references.md) / [図解の対応範囲](../../visualizations/geometry_extensions/README.md)。
