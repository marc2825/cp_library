---
title: 最近点対
documentation_of: //computational_geometry/closest_pair.hpp
---
# 最近点対

[カテゴリ・共通仕様](README.md) / [操作できる図解](../../visualizations/computational_geometry/index.html#closest) / [一次資料・設計判断](references.md)

`closest_pair(points)`は最近点対を`optional<PairResult>`で返します。`first < second`は**入力配列の元の位置**で、入力は並べ替えません。`squared_distance`は整数版Wide、浮動小数点版long double。二点未満はnullopt、同一座標が二つあれば距離0です。同距離の対の選択順は保証しません。

x順に二分し、各側の結果をy順にマージして、境界から距離d未満の帯だけ比較します。帯の候補をy順に絞るのでO(n log n)です。毎回の帯で再ソートはしません。

浮動小数点版の計量と重複判定にEPSを使いません。たとえば距離1e−12の異なる二点を、既定EPS1e−10によって距離0へ丸めません。数値の丸め・アンダーフローによって距離の分離を失う極端な尺度は条件外です。

## usage

```cpp
#include <iostream>
#include <vector>
#include "computational_geometry/closest_pair.hpp"
namespace e=cp::geometry::exact;
namespace f=cp::geometry::floating;
int main(){
    auto r=e::closest_pair({{0,0},{3,4},{1,1},{7,8}});
    if(r)std::cout<<r->first<<' '<<r->second<<' '<<e::to_string(r->squared_distance)<<'\n';
}
```

[実行できる例](../../examples/computational_geometry/closest_pair.cpp)。整数・浮動小数点の型とnamespaceは混ぜず、使用する方を明示してください。

## 条件・計算量

O(n log n) 時間・O(n) 追加領域。整数はGCC/Clangの検査付き128ビット演算です。有理数の正規化・比較にはO(log M)の整数演算を要し、上の計算量はこれを基本演算一回として数えています。中間値の範囲・浮動小数点の誤差と例外は[共通仕様](README.md#数値と境界の契約)を参照してください。

## 使用する問題・検証

Library Checker: Closest Pair。ローカルでは[検証スクリプト](../../verify/run_computational_geometry.py)から使用例・独立したFraction／総当たりの解・境界と退化ケースを確認します。全体の検証範囲は[カテゴリ索引](README.md#ローカル検証)に記載しています。オンラインACは未確認です。
