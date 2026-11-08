---
title: 長方形加算・点取得
documentation_of: //offline_query/rectangle_add_point_get.hpp
---

# 長方形加算・点取得

四隅の差分を座圧2D BITに追加し、点以下の差分和で値を得ます。upper_boundを使うので座標に1を足す必要がありません。

## 使用例

```cpp
#include <iostream>
#include "offline_query/rectangle_add_point_get.hpp"
int main() {
    cp::RectangleAddPointGet<> s({{0,0,3,2}}); s.add(0,0,3,2,4);
    std::cout << s.get(2,1) << " " << s.get(3,1) << "\n";
}
```

出力: `4 0`。

## APIと使用条件

`RectangleAddPointGet<T>(vector<array<long long,4>> update_rectangles)`、add(left,down,right,up,value)、get(x,y)。

Tは可換加法群、初期0。更新矩形の全四隅を事前登録。点は未登録でも可。左・下は含み右・上は含まない。LLONG_MIN/MAXも±1なしで取得できます。

## 計算量

R登録矩形に対し構築O(R log²(R+1))、領域O(R log(R+1))、更新／取得O(log²(R+1))。 演算・比較・値のコピーが O(1) の場合です。文字列などを格納すると、その処理時間・領域が加わります。

## 使用する問題と検証

静的／動的な区間集計、版の分岐、矩形更新などで使います。独立した配列・多重集合・全点走査と比較します。

`python3 verify/run_b_1_6.py`（`--sanitize` 対応）で単独／重複include、debug/releaseの比較検証、上の使用例を確認します。検証範囲の詳細は[追加分の一覧](../b_1_6/README.md)。オンライン提出・ACは未確認です。

[操作できる図解](../../visualizations/b_1_6/index.html#rectangle_add_point_get) / [カテゴリ索引](README.md) / [参考資料と設計判断](../b_1_6/references.md#spatial) / [図解の簡略化](../../visualizations/b_1_6/README.md)
