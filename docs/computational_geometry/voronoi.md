---
documentation_of: //computational_geometry/voronoi.hpp
---

# Voronoi図（H-21）

[カテゴリ](README.md) / [H全22項目](../geometry_extensions/README.md) / [実装](../../computational_geometry/voronoi.hpp) / [操作できる図解](../../visualizations/geometry_extensions/index.html#voronoi)

## API・使用条件

`rational::voronoi_diagram(points)`、`voronoi_cells(diagram,lower,upper)`。

真の非有界辺をsegment/ray/lineで返す。origin+t·direction、tは[0,1]/[0,∞)/全実数。共円三角形間の長さ0の双対辺は省略。有限cellは明示矩形内で、sorted unique siteごと。重複対応はdelaunayのmap参照。

C++17、namespace `cp::geometry`。整数・有理・浮動の型を分離しています。`rational::Scalar`は既存BigRational、`rational::Point`は任意精度の有理点で、`exact::Point`（int64）や旧`exact::RationalPoint`から明示変換できます。Boostヘッダが必要ですが追加リンクは不要です。既存128bitのexact APIの契約は変更していません。

浮動APIは既存long double・絶対Toleranceを使用し、有限かつ区別可能な尺度が条件です。丸め・underflowをすべて検出する保証はありません。H-14と厳密有理APIはEPSを使う浮動幾何とは別です。各ヘッダまたはcomputational_geometry/all.hppから利用できます。

## 原理・計算量

Delaunayの隣接三角形の外心を結び、凸包辺には外向きのrayを付ける。cellは隣接siteとの垂直二等分線で切断する。

Delaunay構築＋双対O(n log n)。有限cell切断はΣ degree²、最悪O(n²)演算、全cellの出力O(n)。

## 使用例

[コンパイル可能なソース](../../examples/geometry_extensions/voronoi.cpp)。

```cpp
#include "computational_geometry/voronoi.hpp"
#include <iomanip>
#include <iostream>
using namespace cp::geometry;
int main() {
    std::cout << std::setprecision(6);
    auto d = rational::voronoi_diagram({{1, 1}, {3, 1}, {2, 3}});
    auto cells = rational::voronoi_cells(d, {0, 0}, {4, 4});
    cp::BigRational area;
    for (const auto &p : cells)
        area += cp::abs(rational::signed_area2(p)) / 2;
    std::cout << cells.size() << " " << rational::approximate(area) << "\n";
}
```

出力:

```text
3 16
```

## 使用する問題・検証

Voronoi図を必要とする領域処理・幾何クエリ。競技用途では入力規模と退化条件を先に確認してください。

[比較検証一覧](../geometry_extensions/README.md#検証)。格子の全列挙・独立数値積分・空円条件・Eulerの式・面の向きと体積などで照合し、Debug/NDEBUG・libstdc++ debug mode・sanitizerを確認します。オンライン提出ACは未確認です。

[一次資料・設計判断](../geometry_extensions/references.md) / [図解の対応範囲](../../visualizations/geometry_extensions/README.md)。
