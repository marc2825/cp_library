---
title: Convex Hull
documentation_of: //computational_geometry/convex_hull.hpp
---
# Convex Hull

[カテゴリ・共通仕様](README.md) / [操作できる図解](../../visualizations/computational_geometry/index.html#hull) / [一次資料・設計判断](references.md)

`convex_hull(points,keep_collinear=false)`は入力を変更せず、単調鎖で凸包を返します。既定は辺上の途中点を除いたstrictな凸包、反時計回り、辞書順最小(x,y)が先頭で、閉じるための先頭点の再掲はありません。重複は完全一致で除きます。

空は空、一点は一点、全共線なら両端点を返します。`keep_collinear=true`なら辺上の全ての異なる点を一度ずつ残し、全共線なら辞書順に並べます。回転キャリパーと凸多角形の二分探索には既定のstrictな出力を渡してください。

浮動小数点版は`convex_hull(points,keep_collinear,Tolerance)`。ソートと重複除去にEPSを使わず、曲がる方向の判定だけに使います。許容誤差に比べて辺・角が極端に小さい場合、厳密な位相は保証できません。AOJの先頭点は最小(y,x)なので、ジャッジ用コードで回転して合わせています。

## usage

```cpp
#include <iostream>
#include <vector>
#include "computational_geometry/convex_hull.hpp"
namespace e=cp::geometry::exact;
namespace f=cp::geometry::floating;
int main(){
    auto h=e::convex_hull({{0,0},{2,0},{2,2},{0,2},{1,1},{0,0}});
    for(auto p:h)std::cout<<p.x<<' '<<p.y<<'\n';
}
```

[実行できる例](../../examples/computational_geometry/convex_hull.cpp)。整数・浮動小数点の型とnamespaceは混ぜず、使用する方を明示してください。

## 条件・計算量

O(n log n) 時間・O(n) 追加領域。整数はGCC/Clangの検査付き128ビット演算です。有理数の正規化・比較にはO(log M)の整数演算を要し、上の計算量はこれを基本演算一回として数えています。中間値の範囲・浮動小数点の誤差と例外は[共通仕様](README.md#数値と境界の契約)を参照してください。

## 使用する問題・検証

CGL_4_A。ローカルでは[検証スクリプト](../../verify/run_computational_geometry.py)から使用例・独立したFraction／総当たりの解・境界と退化ケースを確認します。全体の検証範囲は[カテゴリ索引](README.md#ローカル検証)に記載しています。オンラインACは未確認です。
