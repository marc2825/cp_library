---
title: 線分交差判定
documentation_of: //computational_geometry/exact/primitives.hpp
---
# 線分交差判定

[カテゴリ・共通仕様](README.md) / [操作できる図解](../../visualizations/computational_geometry/index.html#segments) / [一次資料・設計判断](references.md)

`Segment {Point a,b}`は長さ0も許す閉線分です。`on_segment(s,p)`は境界込みの所属、`segments_intersect(s,t)`は一点の接触・共線の重なりもtrueにします。整数版は外積の符号と軸ごとの区間を照合します。

浮動小数点版は末尾に`Tolerance`を渡せます。外積と座標範囲の両方に絶対EPSを使うため、厳密には離れた線分もEPSの範囲で交差と扱う場合があります。点に退化した線分も同じ規約です。交差点の座標列を返す関数ではありません。

## usage

```cpp
#include <iostream>
#include <vector>
#include "computational_geometry/exact/primitives.hpp"
namespace e=cp::geometry::exact;
int main(){
    std::cout<<e::segments_intersect({{0,0},{2,2}},{{0,2},{2,0}})<<'\n';
    std::cout<<e::segments_intersect({{0,0},{0,0}},{{1,0},{2,0}})<<'\n';
}
```

[実行できる例](../../examples/computational_geometry/segment_intersection.cpp)。整数・浮動小数点の型とnamespaceは混ぜず、使用する方を明示してください。

## 条件・計算量

O(1)。整数はGCC/Clangの検査付き128ビット演算です。有理数の正規化・比較にはO(log M)の整数演算を要し、上の計算量はこれを基本演算一回として数えています。中間値の範囲・浮動小数点の誤差と例外は[共通仕様](README.md#数値と境界の契約)を参照してください。

## 使用する問題・検証

CGL_2_B。ローカルでは[検証スクリプト](../../verify/run_computational_geometry.py)から使用例・独立したFraction／総当たりの解・境界と退化ケースを確認します。全体の検証範囲は[カテゴリ索引](README.md#ローカル検証)に記載しています。オンラインACは未確認です。
