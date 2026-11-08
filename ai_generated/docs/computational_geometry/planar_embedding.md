---
documentation_of: //computational_geometry/planar_embedding.hpp
---

# 平面埋め込みの面列挙・双対（H-09）

[カテゴリ](README.md) / [H全22項目](../geometry_extensions/README.md) / [実装](../../computational_geometry/planar_embedding.hpp) / [操作できる図解](../../visualizations/geometry_extensions/index.html#planar_embedding)

## API・使用条件

`rational::planar_embedding(vertices,edges,check=true)`、`planar_embedding(arrangement)`。

直線辺の交差は分割済み、同一座標頂点・自己辺・重複辺は禁止。half_edgesにfrom/to/twin/next/face。faces[0]が唯一の外面。穴・橋・孤立点・非連結成分を同じ面へまとめる。双対は橋のloopと平行辺を保持。外面のareaは0。

C++17、namespace `cp::geometry`。整数・有理・浮動の型を分離しています。`rational::Scalar`は既存BigRational、`rational::Point`は任意精度の有理点で、`exact::Point`（int64）や旧`exact::RationalPoint`から明示変換できます。Boostヘッダが必要ですが追加リンクは不要です。既存128bitのexact APIの契約は変更していません。

浮動APIは既存long double・絶対Toleranceを使用し、有限かつ区別可能な尺度が条件です。丸め・underflowをすべて検出する保証はありません。H-14と厳密有理APIはEPSを使う浮動幾何とは別です。各ヘッダまたはcomputational_geometry/all.hppから利用できます。

## 原理・計算量

偏角順のhalf-edgeを左側の面に沿って歩き、負向き境界を包含する最小の正向き境界へ所属させる。

E辺・V頂点でO((E+V) log(E+V+1)+E²+VE)演算、O(E+V)領域。検査を省略しても面包含の費用は残る。

## 使用例

[コンパイル可能なソース](../../examples/geometry_extensions/planar_embedding.cpp)。

```cpp
#include "computational_geometry/planar_embedding.hpp"
#include <iomanip>
#include <iostream>
using namespace cp::geometry;
int main() {
    std::cout << std::setprecision(6);
    auto e = rational::planar_embedding({{0, 0}, {6, 0}, {6, 6}, {0, 6}, {2, 2}, {4, 2}, {4, 4}, {2, 4}},
                                        {{0, 1}, {1, 2}, {2, 3}, {3, 0}, {4, 5}, {5, 6}, {6, 7}, {7, 4}});
    std::cout << e.faces.size() << " " << e.dual_edges.size() << "\n";
}
```

出力:

```text
3 8
```

## 使用する問題・検証

平面埋め込みの面列挙・双対を必要とする領域処理・幾何クエリ。競技用途では入力規模と退化条件を先に確認してください。

[比較検証一覧](../geometry_extensions/README.md#検証)。格子の全列挙・独立数値積分・空円条件・Eulerの式・面の向きと体積などで照合し、Debug/NDEBUG・libstdc++ debug mode・sanitizerを確認します。オンライン提出ACは未確認です。

[一次資料・設計判断](../geometry_extensions/references.md) / [図解の対応範囲](../../visualizations/geometry_extensions/README.md)。
