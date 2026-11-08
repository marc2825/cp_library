---
documentation_of: //computational_geometry/rational_lattice.hpp
---

# 有理凸領域の格子点・整数凸包（H-10）

[カテゴリ](README.md) / [H全22項目](../geometry_extensions/README.md) / [実装](../../computational_geometry/rational_lattice.hpp) / [操作できる図解](../../visualizations/geometry_extensions/index.html#rational_lattice)

## API・使用条件

`rational::count_lattice_points(points)`、`integer_hull(points,max_columns=1000000)`。

ユーザー承認済みの有理凸領域。点集合を凸包に正規化し、境界込み。空・点・線分も扱う。計数結果BigInt。整数凸包だけは整数x列を走査し、予算超過をlength_errorで拒否する。

C++17、namespace `cp::geometry`。整数・有理・浮動の型を分離しています。`rational::Scalar`は既存BigRational、`rational::Point`は任意精度の有理点で、`exact::Point`（int64）や旧`exact::RationalPoint`から明示変換できます。Boostヘッダが必要ですが追加リンクは不要です。既存128bitのexact APIの契約は変更していません。

浮動APIは既存long double・絶対Toleranceを使用し、有限かつ区別可能な尺度が条件です。丸め・underflowをすべて検出する保証はありません。H-14と厳密有理APIはEPSを使う浮動幾何とは別です。各ヘッダまたはcomputational_geometry/all.hppから利用できます。

## 原理・計算量

上下境界の一次式にfloor-sumを適用する。整数凸包は各整数x列の最下端・最上端だけを凸包へ入れる。

頂点n、floor-sumのEuclid反復数Lとして計数O(n²+nL)。整数凸包は走査列WでO(n²+W log(W+1))、O(n+W)領域。bit計算量は別。

## 使用例

[コンパイル可能なソース](../../examples/geometry_extensions/rational_lattice.cpp)。

```cpp
#include "computational_geometry/rational_lattice.hpp"
#include <iomanip>
#include <iostream>
using namespace cp::geometry;
int main() {
    std::cout << std::setprecision(6);
    std::vector<rational::Point> p{{0, 0}, {3, 0}, {3, 2}, {0, 2}};
    std::cout << rational::count_lattice_points(p) << " " << rational::integer_hull(p).size() << "\n";
}
```

出力:

```text
12 4
```

## 使用する問題・検証

有理凸領域の格子点・整数凸包を必要とする領域処理・幾何クエリ。競技用途では入力規模と退化条件を先に確認してください。

[比較検証一覧](../geometry_extensions/README.md#検証)。格子の全列挙・独立数値積分・空円条件・Eulerの式・面の向きと体積などで照合し、Debug/NDEBUG・libstdc++ debug mode・sanitizerを確認します。オンライン提出ACは未確認です。

[一次資料・設計判断](../geometry_extensions/references.md) / [図解の対応範囲](../../visualizations/geometry_extensions/README.md)。
