---
documentation_of: //computational_geometry/circle_area.hpp
---

# 円の和集合面積（H-08）

[カテゴリ](README.md) / [H全22項目](../geometry_extensions/README.md) / [実装](../../computational_geometry/circle_area.hpp) / [操作できる図解](../../visualizations/geometry_extensions/index.html#circle_union_area)

## API・使用条件

`floating::circle_union_area(circles)`。

点円は寄与0、完全に同じ円は一度だけ数える。包含・交差・接触を扱う。分類は実際のlong double比較で、Toleranceによる近似同一視はしない。

C++17、namespace `cp::geometry`。整数・有理・浮動の型を分離しています。`rational::Scalar`は既存BigRational、`rational::Point`は任意精度の有理点で、`exact::Point`（int64）や旧`exact::RationalPoint`から明示変換できます。Boostヘッダが必要ですが追加リンクは不要です。既存128bitのexact APIの契約は変更していません。

浮動APIは既存long double・絶対Toleranceを使用し、有限かつ区別可能な尺度が条件です。丸め・underflowをすべて検出する保証はありません。H-14と厳密有理APIはEPSを使う浮動幾何とは別です。各ヘッダまたはcomputational_geometry/all.hppから利用できます。

## 原理・計算量

他の円に覆われる角度区間を統合し、露出する円弧の境界積分を足す。先頭の中心へ積分原点を移し、全周はπr²を直接加えて平行移動による桁落ちを抑える。

O(n² log n)時間、O(n)追加領域。

## 使用例

[コンパイル可能なソース](../../examples/geometry_extensions/circle_union_area.cpp)。

```cpp
#include "computational_geometry/circle_area.hpp"
#include <iomanip>
#include <iostream>
using namespace cp::geometry;
int main() {
    std::cout << std::setprecision(6);
    std::cout << floating::circle_union_area({{{0, 0}, 1}, {{4, 0}, 1}}) / std::acos(-1.L) << "\n";
}
```

出力:

```text
2
```

## 使用する問題・検証

円の和集合面積を必要とする領域処理・幾何クエリ。競技用途では入力規模と退化条件を先に確認してください。

[比較検証一覧](../geometry_extensions/README.md#検証)。格子の全列挙・独立数値積分・空円条件・Eulerの式・面の向きと体積などで照合し、Debug/NDEBUG・libstdc++ debug mode・sanitizerを確認します。オンライン提出ACは未確認です。

[一次資料・設計判断](../geometry_extensions/references.md) / [図解の対応範囲](../../visualizations/geometry_extensions/README.md)。
