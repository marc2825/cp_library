---
documentation_of: //computational_geometry/convex_farthest_neighbors.hpp
---

# 凸多角形の全頂点の最遠点（H-16）

[カテゴリ](README.md) / [H全22項目](../geometry_extensions/README.md) / [実装](../../computational_geometry/convex_farthest_neighbors.hpp) / [操作できる図解](../../visualizations/geometry_extensions/index.html#convex_farthest_neighbors)

## API・使用条件

`exact::convex_farthest_neighbors(points)` → 各入力に`{index,squared_distance}`。

点集合は重複なし・strict凸位置。入力順は任意、凸包へ正規化し元添字へ戻す。空可、一点は自分。等距離のどれか1点を返し、全tie列挙はしない。距離はBigInt。

C++17、namespace `cp::geometry`。整数・有理・浮動の型を分離しています。`rational::Scalar`は既存BigRational、`rational::Point`は任意精度の有理点で、`exact::Point`（int64）や旧`exact::RationalPoint`から明示変換できます。Boostヘッダが必要ですが追加リンクは不要です。既存128bitのexact APIの契約は変更していません。

浮動APIは既存long double・絶対Toleranceを使用し、有限かつ区別可能な尺度が条件です。丸め・underflowをすべて検出する保証はありません。H-14と厳密有理APIはEPSを使う浮動幾何とは別です。各ヘッダまたはcomputational_geometry/all.hppから利用できます。

## 原理・計算量

凸包上の最遠点添字の巡回単調性を使い、行最大値を分割統治で求める。

正規化と単調行最大値探索でO(n log n)演算、O(n)領域。

## 使用例

[コンパイル可能なソース](../../examples/geometry_extensions/convex_farthest_neighbors.cpp)。

```cpp
#include "computational_geometry/convex_farthest_neighbors.hpp"
#include <iomanip>
#include <iostream>
using namespace cp::geometry;
int main() {
    std::cout << std::setprecision(6);
    for (const auto &p : exact::convex_farthest_neighbors({{0, 0}, {4, 0}, {3, 2}, {0, 3}}))
        std::cout << p.squared_distance << " ";
    std::cout << "\n";
}
```

出力:

```text
16 25 13 25
```

## 使用する問題・検証

凸多角形の全頂点の最遠点を必要とする領域処理・幾何クエリ。競技用途では入力規模と退化条件を先に確認してください。

[比較検証一覧](../geometry_extensions/README.md#検証)。格子の全列挙・独立数値積分・空円条件・Eulerの式・面の向きと体積などで照合し、Debug/NDEBUG・libstdc++ debug mode・sanitizerを確認します。オンライン提出ACは未確認です。

[一次資料・設計判断](../geometry_extensions/references.md) / [図解の対応範囲](../../visualizations/geometry_extensions/README.md)。
