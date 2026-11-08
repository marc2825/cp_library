---
documentation_of: //computational_geometry/robust_predicates.hpp
---

# 適応精度・厳密幾何判定（H-14）

[カテゴリ](README.md) / [H全22項目](../geometry_extensions/README.md) / [実装](../../computational_geometry/robust_predicates.hpp) / [操作できる図解](../../visualizations/geometry_extensions/index.html#robust_predicates)

## API・使用条件

`robust::orientation`、`incircle`：exact::Point / floating::Point。`dyadic`、`exact_value`。

整数はint64全域をBigIntへ拡張。浮動は実際に表現された有限radix-2 long doubleの値を判定する。外向きintervalで符号を確定できなければ厳密dyadic有理数へ切替。incircleはabcがCCWなら円内が正、CWなら逆。fast-math不可。

C++17、namespace `cp::geometry`。整数・有理・浮動の型を分離しています。`rational::Scalar`は既存BigRational、`rational::Point`は任意精度の有理点で、`exact::Point`（int64）や旧`exact::RationalPoint`から明示変換できます。Boostヘッダが必要ですが追加リンクは不要です。既存128bitのexact APIの契約は変更していません。

浮動APIは既存long double・絶対Toleranceを使用し、有限かつ区別可能な尺度が条件です。丸め・underflowをすべて検出する保証はありません。H-14と厳密有理APIはEPSを使う浮動幾何とは別です。各ヘッダまたはcomputational_geometry/all.hppから利用できます。

## 原理・計算量

符号が0を跨ぐかを外向きの区間演算で確かめ、不確かな場合だけ全bitを使う。

フィルタはO(1)、fallbackは座標の有効bit長に依存するBigInt/BigRational演算。

## 使用例

[コンパイル可能なソース](../../examples/geometry_extensions/robust_predicates.cpp)。

```cpp
#include "computational_geometry/robust_predicates.hpp"
#include <iomanip>
#include <iostream>
using namespace cp::geometry;
int main() {
    std::cout << std::setprecision(6);
    std::cout << robust::orientation(exact::Point{0, 0}, exact::Point{4, 0}, exact::Point{0, 3}) << " "
              << robust::incircle(exact::Point{0, 0}, exact::Point{4, 0}, exact::Point{0, 3},
                                  exact::Point{1, 1})
              << "\n";
}
```

出力:

```text
1 1
```

## 使用する問題・検証

適応精度・厳密幾何判定を必要とする領域処理・幾何クエリ。競技用途では入力規模と退化条件を先に確認してください。

[比較検証一覧](../geometry_extensions/README.md#検証)。格子の全列挙・独立数値積分・空円条件・Eulerの式・面の向きと体積などで照合し、Debug/NDEBUG・libstdc++ debug mode・sanitizerを確認します。オンライン提出ACは未確認です。

[一次資料・設計判断](../geometry_extensions/references.md) / [図解の対応範囲](../../visualizations/geometry_extensions/README.md)。
