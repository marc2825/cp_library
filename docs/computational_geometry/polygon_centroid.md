---
documentation_of: //computational_geometry/polygon_centroid.hpp
---

# 多角形の重心（H-05）

[カテゴリ](README.md) / [H全22項目](../geometry_extensions/README.md) / [実装](../../computational_geometry/polygon_centroid.hpp) / [操作できる図解](../../visualizations/geometry_extensions/index.html#polygon_centroid)

## API・使用条件

`rational::polygon_centroid`、`exact::polygon_centroid`、`floating::polygon_centroid`。

単純な境界ring。CW/CCW可。面積0はnullopt。整数入力も結果は任意精度有理点。浮動版は先頭点を原点にして和を取る。

C++17、namespace `cp::geometry`。整数・有理・浮動の型を分離しています。`rational::Scalar`は既存BigRational、`rational::Point`は任意精度の有理点で、`exact::Point`（int64）や旧`exact::RationalPoint`から明示変換できます。Boostヘッダが必要ですが追加リンクは不要です。既存128bitのexact APIの契約は変更していません。

浮動APIは既存long double・絶対Toleranceを使用し、有限かつ区別可能な尺度が条件です。丸め・underflowをすべて検出する保証はありません。H-14と厳密有理APIはEPSを使う浮動幾何とは別です。各ヘッダまたはcomputational_geometry/all.hppから利用できます。

## 原理・計算量

有向辺ごとの外積で、原点との三角形の面積と重心を加重する。

O(n)演算、O(1)追加領域（整数からの変換はO(n)）。

## 使用例

[コンパイル可能なソース](../../examples/geometry_extensions/polygon_centroid.cpp)。

```cpp
#include "computational_geometry/polygon_centroid.hpp"
#include <iomanip>
#include <iostream>
using namespace cp::geometry;
int main() {
    std::cout << std::setprecision(6);
    const auto p = *rational::polygon_centroid({{0, 0}, {4, 0}, {4, 2}, {0, 2}});
    std::cout << rational::approximate(p.x) << " " << rational::approximate(p.y) << "\n";
}
```

出力:

```text
2 1
```

## 使用する問題・検証

多角形の重心を必要とする領域処理・幾何クエリ。競技用途では入力規模と退化条件を先に確認してください。

[比較検証一覧](../geometry_extensions/README.md#検証)。格子の全列挙・独立数値積分・空円条件・Eulerの式・面の向きと体積などで照合し、Debug/NDEBUG・libstdc++ debug mode・sanitizerを確認します。オンライン提出ACは未確認です。

[一次資料・設計判断](../geometry_extensions/references.md) / [図解の対応範囲](../../visualizations/geometry_extensions/README.md)。
