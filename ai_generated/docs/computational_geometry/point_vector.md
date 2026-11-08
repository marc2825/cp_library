---
title: Point / Vector・内積・外積・CCW
documentation_of: //computational_geometry/exact/point.hpp
---
# Point / Vector・内積・外積・CCW

[カテゴリ・共通仕様](README.md) / [操作できる図解](../../visualizations/computational_geometry/index.html#vectors) / [一次資料・設計判断](references.md)

`exact::Point {int64_t x,y}`と`floating::Point {long double x,y}`は別の型です。`Vector`は各Pointの別名。`+`、`-`、`p*k`、等値と辞書順比較を提供し、浮動小数点版だけ`p/k`も使えます。整数Pointの演算結果はint64_tへ収まる必要があります。

`dot(a,b)`は内積、`cross(a,b)`は外積、`cross(a,b,c)`は(b−a)×(c−a)、`norm_squared(p)`と`squared_distance(a,b)`は二乗です。整数の三点外積と距離は座標を128ビットへ広げてから引き算します。`orientation(a,b,c)`は左+1・一直線0・右−1。`ccw`は`exact/primitives.hpp`または`floating/primitives.hpp`をincludeし、`Ccw`の5分類を返します。

| Ccw | 整数値 | 意味 |
|---|---:|---|
| counter_clockwise | 1 | a→bの左 |
| clockwise | −1 | 右 |
| online_back | 2 | aより後ろ |
| online_front | −2 | bより前 |
| on_segment | 0 | 閉線分上 |

基線a=bでは、p=aならon_segment、それ以外はonline_frontです。`Point::operator<`と`==`は浮動小数点版でも数値をそのまま比較します。近似等値は`floating::almost_equal(a,b,Tolerance)`で明示します。

整数の非整数座標は`RationalPoint {Rational x,y}`で保持します。`Rational`は既約分数・正の分母で、四則演算、`== / != / <`、`numerator()`、`denominator()`、`sign()`、`str()`、`approximate()`を提供。文字列は整数または`a/b`です。`exact::to_string(Wide)`で128ビット整数を表示できます。`cp::geometry::to_floating`は整数PointまたはRationalPointを明示的に近似変換します（`conversion.hpp`）。

## usage

```cpp
#include <iostream>
#include <vector>
#include "computational_geometry/exact/primitives.hpp"
namespace e=cp::geometry::exact;
int main(){
    e::Vector a{3,1},b{1,2};
    std::cout<<e::to_string(e::dot(a,b))<<' '<<e::to_string(e::cross(a,b))<<'\n';
    std::cout<<static_cast<int>(e::ccw({0,0},a,b))<<'\n';
}
```

[実行できる例](../../examples/computational_geometry/point_vector.cpp)。整数・浮動小数点の型とnamespaceは混ぜず、使用する方を明示してください。

## 条件・計算量

O(1)。整数はGCC/Clangの検査付き128ビット演算です。有理数の正規化・比較にはO(log M)の整数演算を要し、上の計算量はこれを基本演算一回として数えています。中間値の範囲・浮動小数点の誤差と例外は[共通仕様](README.md#数値と境界の契約)を参照してください。

## 使用する問題・検証

CGL_1_C。ローカルでは[検証スクリプト](../../verify/run_computational_geometry.py)から使用例・独立したFraction／総当たりの解・境界と退化ケースを確認します。全体の検証範囲は[カテゴリ索引](README.md#ローカル検証)に記載しています。オンラインACは未確認です。
