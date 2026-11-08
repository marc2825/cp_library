---
title: 長方形加算・長方形和
documentation_of: //offline_query/rectangle_add_rectangle_sum.hpp
---

# 長方形加算・長方形和

矩形の四隅に差分を置きます。各差分の係数(w,-wx,-wy,wxy)を疎な2D BITで集計し、原点からの積分をwXY-wxY-wyX+wxyとして計算します。

## 使用例

```cpp
#include <iostream>
#include "offline_query/rectangle_add_rectangle_sum.hpp"
int main() {
    cp::RectangleAddRectangleSum<> s({{0,0,3,2}}); s.add(0,0,3,2,4);
    std::cout << s.sum(1,0,3,2) << "\n";
}
```

出力: `16`。

## APIと使用条件

`RectangleAddRectangleSum<Weight>(vector<array<long long,4>> update_rectangles)`、add(left,down,right,up,value)、sum(left,down,right,up)。

整数格子のセル[x,x+1)×[y,y+1)への加算、有限整数矩形の和。更新矩形の全四隅を事前登録。問い合わせは未登録境界可。Weightは座標から構築できる可換環で、係数・中間積まで型内に収めます。負座標可、空／逆向きは0。オンライン順序で更新できます。

## 計算量

R登録矩形に対し構築O(R log²(R+1))、領域O(R log(R+1))、更新／和O(log²(R+1))。 演算・比較・値のコピーが O(1) の場合です。文字列などを格納すると、その処理時間・領域が加わります。

## 使用する問題と検証

[Library Checker: Static Rectangle Add Rectangle Sum](https://judge.yosupo.jp/problem/static_rectangle_add_rectangle_sum)。小さい整数格子の各セルを直接更新して比較します。

`python3 verify/run_b_1_6.py`（`--sanitize` 対応）で単独／重複include、debug/releaseの比較検証、上の使用例を確認します。検証範囲の詳細は[追加分の一覧](../b_1_6/README.md)。オンライン提出・ACは未確認です。

[操作できる図解](../../visualizations/b_1_6/index.html#rectangle_add_rectangle_sum) / [カテゴリ索引](README.md) / [参考資料と設計判断](../b_1_6/references.md#spatial) / [図解の簡略化](../../visualizations/b_1_6/README.md)
