---
documentation_of: //computational_geometry/circle_tangents.hpp
---

# 円の接線（H-03）

[カテゴリ](README.md) / [H全22項目](../geometry_extensions/README.md) / [実装](../../computational_geometry/circle_tangents.hpp) / [操作できる図解](../../visualizations/geometry_extensions/index.html#circle_tangents)

## API・使用条件

`floating::common_tangents(a,b,tol)`、`tangents_from_point(circle,p,tol)`。`CircleTangents{infinite,lines}`。

線は単位normal・offset（normal·x=offset）と両円の接点を返す。接点が一致しても線を保持。同一点円も無限本。点円同士の異なる2点は1本。

C++17、namespace `cp::geometry`。整数・有理・浮動の型を分離しています。`rational::Scalar`は既存BigRational、`rational::Point`は任意精度の有理点で、`exact::Point`（int64）や旧`exact::RationalPoint`から明示変換できます。Boostヘッダが必要ですが追加リンクは不要です。既存128bitのexact APIの契約は変更していません。

浮動APIは既存long double・絶対Toleranceを使用し、有限かつ区別可能な尺度が条件です。丸め・underflowをすべて検出する保証はありません。H-14と厳密有理APIはEPSを使う浮動幾何とは別です。各ヘッダまたはcomputational_geometry/all.hppから利用できます。

## 原理・計算量

2円の半径差・半径和と中心間距離から接線の法線を求め、同じ直線を統合する。

O(1)。

## 使用例

[コンパイル可能なソース](../../examples/geometry_extensions/circle_tangents.cpp)。

```cpp
#include "computational_geometry/circle_tangents.hpp"
#include <iomanip>
#include <iostream>
using namespace cp::geometry;
int main() {
    std::cout << std::setprecision(6);
    const auto t = floating::common_tangents({{0, 0}, 1}, {{4, 0}, 1});
    std::cout << t.infinite << " " << t.lines.size() << "\n";
}
```

出力:

```text
0 4
```

## 使用する問題・検証

円の接線を必要とする領域処理・幾何クエリ。競技用途では入力規模と退化条件を先に確認してください。

[比較検証一覧](../geometry_extensions/README.md#検証)。格子の全列挙・独立数値積分・空円条件・Eulerの式・面の向きと体積などで照合し、Debug/NDEBUG・libstdc++ debug mode・sanitizerを確認します。オンライン提出ACは未確認です。

[一次資料・設計判断](../geometry_extensions/references.md) / [図解の対応範囲](../../visualizations/geometry_extensions/README.md)。
