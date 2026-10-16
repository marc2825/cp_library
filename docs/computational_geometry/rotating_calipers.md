---
title: Rotating Calipers・凸多角形の直径
documentation_of: //computational_geometry/rotating_calipers.hpp
---
# Rotating Calipers・凸多角形の直径

[カテゴリ・共通仕様](README.md) / [操作できる図解](../../visualizations/computational_geometry/index.html#calipers) / [一次資料・設計判断](references.md)

`convex_diameter(strict_hull)`は最遠点対を`optional<PairResult>`で返します。PairResultは`squared_distance,first,second`を持ち、添字は**渡した凸包の頂点列**の位置です。距離の二乗は整数版Wide、浮動小数点版long double。元の点集合の添字が必要なら座標から対応づけます。

反時計回り・共線の途中点なし・閉点の重複なしの凸包が前提です。通常はconvex_hullの既定出力を渡します。空はnullopt、一点は{0,0,0}、二点以上は異なる添字を返し、複数最適解の辞書順最小は保証しません。

各辺と向かい合う点の三角形面積が増える間ポインタを進め、面積が同じときは次の点も候補にします。浮動小数点版の距離・面積比較にはEPSを使いません。共線点を保持した凸包を直接渡すのは前提外です。

## usage

```cpp
#include <iostream>
#include <vector>
#include "computational_geometry/rotating_calipers.hpp"
namespace e=cp::geometry::exact;
namespace f=cp::geometry::floating;
int main(){
    auto h=e::convex_hull({{0,0},{3,0},{3,4},{0,4}});
    auto r=e::convex_diameter(h);
    if(r)std::cout<<e::to_string(r->squared_distance)<<'\n';
}
```

[実行できる例](../../examples/computational_geometry/rotating_calipers.cpp)。整数・浮動小数点の型とnamespaceは混ぜず、使用する方を明示してください。

## 条件・計算量

O(h) 時間・O(1) 追加領域。整数はGCC/Clangの検査付き128ビット演算です。有理数の正規化・比較にはO(log M)の整数演算を要し、上の計算量はこれを基本演算一回として数えています。中間値の範囲・浮動小数点の誤差と例外は[共通仕様](README.md#数値と境界の契約)を参照してください。

## 使用する問題・検証

Library Checker: Furthest Pair。ローカルでは[検証スクリプト](../../verify/run_computational_geometry.py)から使用例・独立したFraction／総当たりの解・境界と退化ケースを確認します。全体の検証範囲は[カテゴリ索引](README.md#ローカル検証)に記載しています。オンラインACは未確認です。
