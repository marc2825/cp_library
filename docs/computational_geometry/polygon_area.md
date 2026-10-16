---
title: Polygon Area
documentation_of: //computational_geometry/polygon.hpp
---
# Polygon Area

[カテゴリ・共通仕様](README.md) / [操作できる図解](../../visualizations/computational_geometry/index.html#area) / [一次資料・設計判断](references.md)

`signed_area2(polygon)`は符号付き面積の2倍で、反時計回りなら正、時計回りなら負です。`polygon_area(polygon)`は絶対値を取り非負の面積を返します。閉じる先頭点を末尾へ重ねる必要はありません。空・一点・線分は0です。

頂点は単純多角形の周上順に渡します。自己交差では符号付き代数面積になるため、塗りつぶし面積の意味では使いません。整数Point入力のsigned_area2はWide、面積はRational。切断後のvector<RationalPoint>にも両関数を提供し、いずれもRationalです。浮動小数点版はlong double。頂点0からの三角形の和で計算します。

## usage

```cpp
#include <iostream>
#include <vector>
#include "computational_geometry/polygon.hpp"
namespace e=cp::geometry::exact;
namespace f=cp::geometry::floating;
int main(){
    std::vector<e::Point> p{{0,0},{3,0},{0,1}};
    std::cout<<e::to_string(e::signed_area2(p))<<' '<<e::polygon_area(p).str()<<'\n';
}
```

[実行できる例](../../examples/computational_geometry/polygon_area.cpp)。整数・浮動小数点の型とnamespaceは混ぜず、使用する方を明示してください。

## 条件・計算量

O(n) 時間・O(1) 追加領域。整数はGCC/Clangの検査付き128ビット演算です。有理数の正規化・比較にはO(log M)の整数演算を要し、上の計算量はこれを基本演算一回として数えています。中間値の範囲・浮動小数点の誤差と例外は[共通仕様](README.md#数値と境界の契約)を参照してください。

## 使用する問題・検証

CGL_3_A。ローカルでは[検証スクリプト](../../verify/run_computational_geometry.py)から使用例・独立したFraction／総当たりの解・境界と退化ケースを確認します。全体の検証範囲は[カテゴリ索引](README.md#ローカル検証)に記載しています。オンラインACは未確認です。
