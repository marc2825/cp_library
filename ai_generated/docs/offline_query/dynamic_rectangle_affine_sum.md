---
title: Dynamic Rectangle Affine Sum
documentation_of: //offline_query/dynamic_rectangle_affine_sum.hpp
---

# Dynamic Rectangle Affine Sum

lazy作用付きのKd木へオンラインで点を追加します。部分木のbounding boxが矩形に含まれるときは、和と要素数から一括してw←aw+bを適用します。

## 使用例

```cpp
#include <iostream>
#include "offline_query/dynamic_rectangle_affine_sum.hpp"
int main() {
    cp::DynamicRectangleAffineSum<> s; s.add_point(1,1,3); s.apply(0,0,3,3,2,1); s.add_point(2,2,5);
    std::cout << s.sum(0,0,3,3) << "\n";
}
```

出力: `12`。

## APIと使用条件

`DynamicRectangleAffineSum<Coordinate,T>`、add_point(x,y,value)は安定ID、get(id)/set(id,value)、sum(left,down,right,up)、apply(left,down,right,up,a,b)、size。

Tは可換環、すべての中間値が型内に収まること。座標は順序付き数値、NaN/∞は禁止。点は削除・移動できず、同座標も別ID。新しい点は過去の作用を受けません。作用の合成は時刻順。挿入で平衡条件が崩れた部分木を再構築し、既存IDとlazy反映後の値を保存します。

## 計算量

領域O(N)。点get/set O(log(N+1))、挿入はnth_elementの期待計算を含め償却O(log³(N+1))の保守的上界、単発O(N log N)。矩形処理は枝刈り依存、最悪O(N)。 演算・比較・値のコピーが O(1) の場合です。文字列などを格納すると、その処理時間・領域が加わります。

## 使用する問題と検証

静的／動的な区間集計、版の分岐、矩形更新などで使います。独立した配列・多重集合・全点走査と比較します。

`python3 verify/run_b_1_6.py`（`--sanitize` 対応）で単独／重複include、debug/releaseの比較検証、上の使用例を確認します。検証範囲の詳細は[追加分の一覧](../b_1_6/README.md)。オンライン提出・ACは未確認です。

[操作できる図解](../../visualizations/b_1_6/index.html#dynamic_rectangle_affine_sum) / [カテゴリ索引](README.md) / [参考資料と設計判断](../b_1_6/references.md#spatial) / [図解の簡略化](../../visualizations/b_1_6/README.md)
