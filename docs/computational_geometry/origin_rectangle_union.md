---
documentation_of: //computational_geometry/origin_rectangle_union.hpp
---

# 原点共有矩形の増分和集合（H-11）

[カテゴリ](README.md) / [H全22項目](../geometry_extensions/README.md) / [実装](../../computational_geometry/origin_rectangle_union.hpp) / [操作できる図解](../../visualizations/geometry_extensions/index.html#origin_rectangle_union)

## API・使用条件

`exact::OriginRectangleUnion`、`insert(x,y)`、`area()`、`skyline()`。

[0,x]×[0,y]、x/yは非負int64。追加のみ、重複可。areaはBigInt。一般矩形の任意追加・削除を扱うAPIではない。

C++17、namespace `cp::geometry`。整数・有理・浮動の型を分離しています。`rational::Scalar`は既存BigRational、`rational::Point`は任意精度の有理点で、`exact::Point`（int64）や旧`exact::RationalPoint`から明示変換できます。Boostヘッダが必要ですが追加リンクは不要です。既存128bitのexact APIの契約は変更していません。

浮動APIは既存long double・絶対Toleranceを使用し、有限かつ区別可能な尺度が条件です。丸め・underflowをすべて検出する保証はありません。H-14と厳密有理APIはEPSを使う浮動幾何とは別です。各ヘッダまたはcomputational_geometry/all.hppから利用できます。

## 原理・計算量

x昇順・高さ降順のskylineをmapで保持し、新しい矩形に隠れた角を削除する。

k個のskyline頂点を削除する追加O(log n+k)、m追加合計O(m log m)、O(m)領域。

## 使用例

[コンパイル可能なソース](../../examples/geometry_extensions/origin_rectangle_union.cpp)。

```cpp
#include "computational_geometry/origin_rectangle_union.hpp"
#include <iomanip>
#include <iostream>
using namespace cp::geometry;
int main() {
    std::cout << std::setprecision(6);
    exact::OriginRectangleUnion area;
    for (auto p : std::vector<std::pair<int, int>>{{2, 3}, {4, 1}, {3, 2}})
        std::cout << area.insert(p.first, p.second) << " ";
    std::cout << "\n";
}
```

出力:

```text
6 8 9
```

## 使用する問題・検証

原点共有矩形の増分和集合を必要とする領域処理・幾何クエリ。競技用途では入力規模と退化条件を先に確認してください。

[比較検証一覧](../geometry_extensions/README.md#検証)。格子の全列挙・独立数値積分・空円条件・Eulerの式・面の向きと体積などで照合し、Debug/NDEBUG・libstdc++ debug mode・sanitizerを確認します。オンライン提出ACは未確認です。

[一次資料・設計判断](../geometry_extensions/references.md) / [図解の対応範囲](../../visualizations/geometry_extensions/README.md)。
