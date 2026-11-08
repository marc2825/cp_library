---
title: Fractional Cascading
documentation_of: //basic_utility/fractional_cascading.hpp
---

# Fractional Cascading

複数のソート済み列へ同じ値を検索します。次の拡張列の一つおきの要素を取り込み、各位置から元の列・次の列への境界を結びます。

## 使用例

```cpp
#include <iostream>
#include "basic_utility/fractional_cascading.hpp"
int main() {
    cp::FractionalCascading<int> f({{1,4,8},{2,4,7},{}});
    std::cout << f.lower_bounds(4)[0] << " " << f.lower_bounds(4)[1] << "\n";
}
```

出力: `1 1`。

## APIと使用条件

`FractionalCascading<T>(lists)`、`lower_bounds(x)`。各列は昇順。返値は各元列のlower_bound位置。空列・重複可、Tの<だけを使用。

次の列への移動時に補正する位置は高々一つ。入力列は構築後に変更できません。

## 計算量

総入力長N・列数Kに対し構築O(N+K)、領域O(N+K)、検索O(log(N+1)+K)。 演算・比較・値のコピーが O(1) の場合です。文字列などを格納すると、その処理時間・領域が加わります。

## 使用する問題と検証

静的／動的な区間集計、版の分岐、矩形更新などで使います。独立した配列・多重集合・全点走査と比較します。

`python3 verify/run_b_1_6.py`（`--sanitize` 対応）で単独／重複include、debug/releaseの比較検証、上の使用例を確認します。検証範囲の詳細は[追加分の一覧](../b_1_6/README.md)。オンライン提出・ACは未確認です。

[操作できる図解](../../visualizations/b_1_6/index.html#fractional_cascading) / [カテゴリ索引](README.md) / [参考資料と設計判断](../b_1_6/references.md#range) / [図解の簡略化](../../visualizations/b_1_6/README.md)
