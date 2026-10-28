---
documentation_of: //computational_geometry/segment_arrangement.hpp
---

# 線分sweep・交点列挙・Arrangement（H-12）

[カテゴリ](README.md) / [H全22項目](../geometry_extensions/README.md) / [実装](../../computational_geometry/segment_arrangement.hpp) / [操作できる図解](../../visualizations/geometry_extensions/index.html#segment_arrangement)

## API・使用条件

`rational::any_segment_intersection`、`segment_intersections`、`segment_arrangement`。

点線分・端点接触・共線重複・多重点を含む。交差pairは点1個または重複区間の両端2個。Arrangementは幾何辺を統合し、元線分ID一覧を保持する。max_pairs=2000000、max_candidates=20000000で明示予算。

C++17、namespace `cp::geometry`。整数・有理・浮動の型を分離しています。`rational::Scalar`は既存BigRational、`rational::Point`は任意精度の有理点で、`exact::Point`（int64）や旧`exact::RationalPoint`から明示変換できます。Boostヘッダが必要ですが追加リンクは不要です。既存128bitのexact APIの契約は変更していません。

浮動APIは既存long double・絶対Toleranceを使用し、有限かつ区別可能な尺度が条件です。丸め・underflowをすべて検出する保証はありません。H-14と厳密有理APIはEPSを使う浮動幾何とは別です。各ヘッダまたはcomputational_geometry/all.hppから利用できます。

## 原理・計算量

x区間の開始・終了で候補を絞り、厳密な交点で全線分を分割して重複辺をまとめる。

bounding-box sweepはO(n log n+C)、Cはx区間が重なる候補対数、最悪O(n²)。分割点・辺の整列はO((n+K)log(n+K))。Bentley–Ottmannの出力依存上界は主張しない。

## 使用例

[コンパイル可能なソース](../../examples/geometry_extensions/segment_arrangement.cpp)。

```cpp
#include "computational_geometry/segment_arrangement.hpp"
#include <iomanip>
#include <iostream>
using namespace cp::geometry;
int main() {
    std::cout << std::setprecision(6);
    auto a = rational::segment_arrangement({{{0, 0}, {4, 4}}, {{0, 4}, {4, 0}}, {{0, 2}, {4, 2}}});
    std::cout << a.vertices.size() << " " << a.edges.size() << "\n";
}
```

出力:

```text
7 6
```

## 使用する問題・検証

線分sweep・交点列挙・Arrangementを必要とする領域処理・幾何クエリ。競技用途では入力規模と退化条件を先に確認してください。

[比較検証一覧](../geometry_extensions/README.md#検証)。格子の全列挙・独立数値積分・空円条件・Eulerの式・面の向きと体積などで照合し、Debug/NDEBUG・libstdc++ debug mode・sanitizerを確認します。オンライン提出ACは未確認です。

[一次資料・設計判断](../geometry_extensions/references.md) / [図解の対応範囲](../../visualizations/geometry_extensions/README.md)。
