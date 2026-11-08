---
title: Range Tree（共通入口）
documentation_of: //offline_query/range_tree.hpp
---

# Range Tree（共通入口）

x順のセグメント木の各節点に、y順の座標とFenwick Treeを置きます。更新点の座標を固定し、各点の重みをオンラインで変更します。

## 使用例

```cpp
#include <iostream>
#include "offline_query/range_tree.hpp"
int main() {
    cp::RangeTree<> s({{1,2},{3,4}}); s.add(0,7); s.add(1,5);
    std::cout << s.sum(0,0,3,4) << "\n";
}
```

出力: `7`。

## APIと使用条件

`RangeTree<Coordinate,Weight>(vector<pair<Coordinate,Coordinate>>)`、add(id,delta)、set(id,value)、get(id)、sum(left,down,right,up)、size。

初期重みは0。IDは入力順、同一座標も別の点です。問い合わせ境界は未登録でも可。Weightは可換加法群。空・逆向きの矩形は0。座標の±1は作りません。 3.3の実体を共有します。

## 計算量

構築O(N log N)、領域O(N log N)、更新・矩形和O(log²(N+1))。 演算・比較・値のコピーが O(1) の場合です。文字列などを格納すると、その処理時間・領域が加わります。

## 使用する問題と検証

[Library Checker: Rectangle Sum](https://judge.yosupo.jp/problem/rectangle_sum)。同一座標・負の重み・未登録閾値を直接走査と比較します。

`python3 verify/run_b_1_6.py`（`--sanitize` 対応）で単独／重複include、debug/releaseの比較検証、上の使用例を確認します。検証範囲の詳細は[追加分の一覧](../b_1_6/README.md)。オンライン提出・ACは未確認です。

[操作できる図解](../../visualizations/b_1_6/index.html#range_tree) / [カテゴリ索引](README.md) / [参考資料と設計判断](../b_1_6/references.md#spatial) / [図解の簡略化](../../visualizations/b_1_6/README.md)
