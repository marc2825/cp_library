---
title: 射影・距離
documentation_of: //computational_geometry/floating/primitives.hpp
---
# 射影・距離

[カテゴリ・共通仕様](README.md) / [操作できる図解](../../visualizations/computational_geometry/index.html#projection) / [一次資料・設計判断](references.md)

`projection(line,p)`は無限直線への垂足です。整数版はRationalPoint、浮動小数点版はPointを返します。線分への距離は射影係数を[0,1]へ制限するので、垂足が外にある場合は端点を選びます。

`squared_distance(a,b)`はPoint–Point、Line–Point、Segment–Point、Line–Line、Line–Segment、Segment–Segmentと対称な引数順を提供。整数版はPoint同士だけWide、それ以外はRationalです。整数版に平方根を取るdistance関数は設けません。必要なら`std::sqrt(value.approximate())`など、呼出し側で近似へ移します。

浮動小数点版は全組合せに`squared_distance`と`distance`を提供し、long doubleを返します。交差・平行判定を伴うLine–Line、Line–Segment、Segment–Segmentは末尾Toleranceを指定可能。Point–Point、Line–Point、Segment–Pointの計量にはEPSを入れません。長さ0の線分は点として扱います。方向の二乗が0へアンダーフローした直線／非零線分の射影ではdomain_errorです。

## usage

```cpp
#include <iostream>
#include <vector>
#include "computational_geometry/all.hpp"
namespace e=cp::geometry::exact;
namespace f=cp::geometry::floating;
int main(){
    const e::Line line({0,0},{2,1});
    auto q=e::projection(line,{1,2});
    std::cout<<q.x.str()<<' '<<q.y.str()<<' '<<e::squared_distance(line,e::Point{1,2}).str()<<'\n';
    const f::Line other({0,0},{2,1});
    std::cout<<(f::distance(other,f::Point{1,2})>1)<<'\n';
}
```

[実行できる例](../../examples/computational_geometry/projection_distance.cpp)。整数・浮動小数点の型とnamespaceは混ぜず、使用する方を明示してください。

## 条件・計算量

O(1)。整数はGCC/Clangの検査付き128ビット演算です。有理数の正規化・比較にはO(log M)の整数演算を要し、上の計算量はこれを基本演算一回として数えています。中間値の範囲・浮動小数点の誤差と例外は[共通仕様](README.md#数値と境界の契約)を参照してください。

## 使用する問題・検証

CGL_1_A / CGL_2_D。ローカルでは[検証スクリプト](../../verify/run_computational_geometry.py)から使用例・独立したFraction／総当たりの解・境界と退化ケースを確認します。全体の検証範囲は[カテゴリ索引](README.md#ローカル検証)に記載しています。オンラインACは未確認です。
