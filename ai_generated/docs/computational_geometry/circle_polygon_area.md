---
documentation_of: //computational_geometry/circle_area.hpp
---

# 円と多角形の共通部分面積（H-06）

[カテゴリ](README.md) / [H全22項目](../geometry_extensions/README.md) / [実装](../../computational_geometry/circle_area.hpp) / [操作できる図解](../../visualizations/geometry_extensions/index.html#circle_polygon_area)

## API・使用条件

`floating::circle_polygon_intersection_area(circle,polygon)`。

単純多角形、CW/CCW可、点円・面積0の領域は0。辺を円との交点で分割し、接触も扱う。穴はringごとの符号付き差を呼出側で合成する。

C++17、namespace `cp::geometry`。整数・有理・浮動の型を分離しています。`rational::Scalar`は既存BigRational、`rational::Point`は任意精度の有理点で、`exact::Point`（int64）や旧`exact::RationalPoint`から明示変換できます。Boostヘッダが必要ですが追加リンクは不要です。既存128bitのexact APIの契約は変更していません。

浮動APIは既存long double・絶対Toleranceを使用し、有限かつ区別可能な尺度が条件です。丸め・underflowをすべて検出する保証はありません。H-14と厳密有理APIはEPSを使う浮動幾何とは別です。各ヘッダまたはcomputational_geometry/all.hppから利用できます。

## 原理・計算量

円内の部分は三角形の外積、円外はatan2による扇形の境界積分に置き換える。

O(n)時間、1辺あたりO(1)追加領域。

## 使用例

[コンパイル可能なソース](../../examples/geometry_extensions/circle_polygon_area.cpp)。

```cpp
#include "computational_geometry/circle_area.hpp"
#include <iomanip>
#include <iostream>
using namespace cp::geometry;
int main() {
    std::cout << std::setprecision(6);
    std::cout << floating::circle_polygon_intersection_area({{0, 0}, 1},
                                                            {{-2, -2}, {2, -2}, {2, 2}, {-2, 2}}) /
                     std::acos(-1.L)
              << "\n";
}
```

出力:

```text
1
```

## 使用する問題・検証

円と多角形の共通部分面積を必要とする領域処理・幾何クエリ。競技用途では入力規模と退化条件を先に確認してください。

[比較検証一覧](../geometry_extensions/README.md#検証)。格子の全列挙・独立数値積分・空円条件・Eulerの式・面の向きと体積などで照合し、Debug/NDEBUG・libstdc++ debug mode・sanitizerを確認します。オンライン提出ACは未確認です。

[一次資料・設計判断](../geometry_extensions/references.md) / [図解の対応範囲](../../visualizations/geometry_extensions/README.md)。
