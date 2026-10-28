---
documentation_of: //computational_geometry/dynamic_convex_hull.hpp
---

# 動的凸包（H-22）

[カテゴリ](README.md) / [H全22項目](../geometry_extensions/README.md) / [実装](../../computational_geometry/dynamic_convex_hull.hpp) / [操作できる図解](../../visualizations/geometry_extensions/index.html#dynamic_convex_hull)

## API・使用条件

`exact::DynamicConvexHull(block_size=128)`、insert/erase/support/hull/contains/size。

int64点のmultiset。eraseは1個削除、存在しなければfalse。support(direction)は内積最大の点をoptionalで返す。hullはstrict CCW・最小点始まり、更新後に参照無効。

C++17、namespace `cp::geometry`。整数・有理・浮動の型を分離しています。`rational::Scalar`は既存BigRational、`rational::Point`は任意精度の有理点で、`exact::Point`（int64）や旧`exact::RationalPoint`から明示変換できます。Boostヘッダが必要ですが追加リンクは不要です。既存128bitのexact APIの契約は変更していません。

浮動APIは既存long double・絶対Toleranceを使用し、有限かつ区別可能な尺度が条件です。丸め・underflowをすべて検出する保証はありません。H-14と厳密有理APIはEPSを使う浮動幾何とは別です。各ヘッダまたはcomputational_geometry/all.hppから利用できます。

## 原理・計算量

点をブロックに分けて上下凸鎖を保持し、更新対象だけ再構築する。削除で疎になったブロックは隣接ブロックと統合・再配分し、ブロック数をO(n/B+1)に保つ。方向別の最大値は各鎖で二分探索する。

異なる点n、ブロック幅Bで更新O(n/B+B log B)、support O((n/B+1)log(B+1))。hull再構築O(n log n)、変更がなければ参照O(1)、containsは再構築後O(log n)。領域O(n)。

## 使用例

[コンパイル可能なソース](../../examples/geometry_extensions/dynamic_convex_hull.cpp)。

```cpp
#include "computational_geometry/dynamic_convex_hull.hpp"
#include <iomanip>
#include <iostream>
using namespace cp::geometry;
int main() {
    std::cout << std::setprecision(6);
    exact::DynamicConvexHull h(2);
    for (auto p : std::vector<exact::Point>{{0, 0}, {4, 0}, {0, 3}, {1, 1}})
        h.insert(p);
    h.erase({1, 1});
    auto p = *h.support({1, 1});
    std::cout << h.hull().size() << " " << p.x << " " << p.y << "\n";
}
```

出力:

```text
3 4 0
```

## 使用する問題・検証

動的凸包を必要とする領域処理・幾何クエリ。競技用途では入力規模と退化条件を先に確認してください。

[比較検証一覧](../geometry_extensions/README.md#検証)。格子の全列挙・独立数値積分・空円条件・Eulerの式・面の向きと体積などで照合し、Debug/NDEBUG・libstdc++ debug mode・sanitizerを確認します。オンライン提出ACは未確認です。

[一次資料・設計判断](../geometry_extensions/references.md) / [図解の対応範囲](../../visualizations/geometry_extensions/README.md)。
