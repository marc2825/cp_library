---
title: Kd-Tree
documentation_of: //offline_query/kd_tree.hpp
---

# Kd-Tree

x/yを交互に中央値分割した静的Kd木です。各部分木の閉じたbounding boxで検索を枝刈りし、矩形内の点IDと最近点を求めます。

## 使用例

```cpp
#include <iostream>
#include "offline_query/kd_tree.hpp"
int main() {
    cp::KdTree<long long> s({{0,0},{4,3},{2,1}});
    std::cout << s.nearest(2,2)->first << "\n";
}
```

出力: `2`。

## APIと使用条件

`KdTree<Coordinate>(vector<pair<Coordinate,Coordinate>>)`、report(left,down,right,up)はID列、nearest(x,y)はoptional<pair<ID,long double二乗距離>>。

IDは入力順、重複点可。reportの列順は未規定、空入力のnearestはnullopt。同距離は最小ID。座標は順序付き数値、浮動小数のNaN/∞は禁止。距離の演算はlong doubleへ変換後に行い、整数の減算溢れを避けます。大座標での距離比較は浮動小数点誤差を含みます。

## 計算量

構築はnth_elementに基づく期待O(N log N)、領域O(N)。矩形報告O(√N+出力数)を目安とし、退化・重複を含む保守的な最悪はO(N)。最近点の最悪もO(N)。 演算・比較・値のコピーが O(1) の場合です。文字列などを格納すると、その処理時間・領域が加わります。

## 使用する問題と検証

[AOJ DSL_2_C: Range Search](https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=DSL_2_C)。矩形内のIDと最近点を全点走査と比較します。

`python3 verify/run_b_1_6.py`（`--sanitize` 対応）で単独／重複include、debug/releaseの比較検証、上の使用例を確認します。検証範囲の詳細は[追加分の一覧](../b_1_6/README.md)。オンライン提出・ACは未確認です。

[操作できる図解](../../visualizations/b_1_6/index.html#kd_tree) / [カテゴリ索引](README.md) / [参考資料と設計判断](../b_1_6/references.md#spatial) / [図解の簡略化](../../visualizations/b_1_6/README.md)
