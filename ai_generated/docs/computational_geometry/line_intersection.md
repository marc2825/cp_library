---
title: 直線交点
documentation_of: //computational_geometry/exact/primitives.hpp
---
# 直線交点

[カテゴリ・共通仕様](README.md) / [操作できる図解](../../visualizations/computational_geometry/index.html#lines) / [一次資料・設計判断](references.md)

`Line(a,b)`は異なる二点で定義する無限直線で、a=bはinvalid_argumentです。`line_intersection(l,m)`の返り値は`LineIntersection {LineRelation relation, optional<Point型> point}`です。

| relation | point |
|---|---|
| unique | 一意の交点。整数版はRationalPoint、浮動小数点版はPoint |
| parallel | 平行な別直線。nullopt |
| coincident | 同一直線。nullopt |

整数版では交点が格子点でなくても分数のまま返します。浮動小数点版は末尾のToleranceで分母の外積と同一直線判定を行います。極めて近い傾きをEPSで平行扱いする場合があるため、用途と座標尺度に合わせます。

## usage

```cpp
#include <iostream>
#include <vector>
#include "computational_geometry/exact/primitives.hpp"
namespace e=cp::geometry::exact;
int main(){
    auto r=e::line_intersection(e::Line({0,0},{3,3}),e::Line({0,2},{4,0}));
    if(r.point)std::cout<<r.point->x.str()<<' '<<r.point->y.str()<<'\n';
}
```

[実行できる例](../../examples/computational_geometry/line_intersection.cpp)。整数・浮動小数点の型とnamespaceは混ぜず、使用する方を明示してください。

## 条件・計算量

O(1)。整数はGCC/Clangの検査付き128ビット演算です。有理数の正規化・比較にはO(log M)の整数演算を要し、上の計算量はこれを基本演算一回として数えています。中間値の範囲・浮動小数点の誤差と例外は[共通仕様](README.md#数値と境界の契約)を参照してください。

## 使用する問題・検証

CGL_2_C。ローカルでは[検証スクリプト](../../verify/run_computational_geometry.py)から使用例・独立したFraction／総当たりの解・境界と退化ケースを確認します。全体の検証範囲は[カテゴリ索引](README.md#ローカル検証)に記載しています。オンラインACは未確認です。
