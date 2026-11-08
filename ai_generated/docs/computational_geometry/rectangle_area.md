---
documentation_of: //computational_geometry/rectangle_union.hpp
---

# 長方形の和集合面積（H-01）

[カテゴリ](README.md) / [H全22項目](../geometry_extensions/README.md) / [実装](../../computational_geometry/rectangle_union.hpp) / [操作できる図解](../../visualizations/geometry_extensions/index.html#rectangle_area)

## API・使用条件

`exact::Rectangle{x1,y1,x2,y2}`、`rectangle_union_area`、`rectangle_union_measure`。

座標はint64全域、戻り値はBigInt。閉矩形で、幅または高さ0は面積・周長とも寄与0。逆順座標は拒否。

C++17、namespace `cp::geometry`。整数・有理・浮動の型を分離しています。`rational::Scalar`は既存BigRational、`rational::Point`は任意精度の有理点で、`exact::Point`（int64）や旧`exact::RationalPoint`から明示変換できます。Boostヘッダが必要ですが追加リンクは不要です。既存128bitのexact APIの契約は変更していません。

浮動APIは既存long double・絶対Toleranceを使用し、有限かつ区別可能な尺度が条件です。丸め・underflowをすべて検出する保証はありません。H-14と厳密有理APIはEPSを使う浮動幾何とは別です。各ヘッダまたはcomputational_geometry/all.hppから利用できます。

## 原理・計算量

x方向に走査し、圧縮したy座標の被覆長を積分する。

O(n log n)回の整数演算、O(n)領域。BigIntのbit計算量は別。

## 使用例

[コンパイル可能なソース](../../examples/geometry_extensions/rectangle_area.cpp)。

```cpp
#include "computational_geometry/rectangle_union.hpp"
#include <iomanip>
#include <iostream>
using namespace cp::geometry;
int main() {
    std::cout << std::setprecision(6);
    std::cout << exact::rectangle_union_area({{0, 0, 3, 2}, {2, 1, 4, 3}}) << "\n";
}
```

出力:

```text
9
```

## 使用する問題・検証

[Library Checker: Area of Union of Rectangles](https://judge.yosupo.jp/problem/area_of_union_of_rectangles) / [ジャッジ形式](../../verify/geometry_extensions/area_of_union_of_rectangles.test.cpp)。ローカル48入力を独立な圧縮格子と比較します。

長方形の和集合面積を必要とする領域処理・幾何クエリ。競技用途では入力規模と退化条件を先に確認してください。

[比較検証一覧](../geometry_extensions/README.md#検証)。格子の全列挙・独立数値積分・空円条件・Eulerの式・面の向きと体積などで照合し、Debug/NDEBUG・libstdc++ debug mode・sanitizerを確認します。オンライン提出ACは未確認です。

[一次資料・設計判断](../geometry_extensions/references.md) / [図解の対応範囲](../../visualizations/geometry_extensions/README.md)。
