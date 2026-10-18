---
title: Leftist Heap
documentation_of: //ordered_structure/leftist_heap.hpp
---

# Leftist Heap

右の経路を併合してから子を交換する併合可能heapです。null-path lengthが左>=右となるよう交換し、右の経路を対数長に保ちます。

## 使用例

```cpp
#include <iostream>
#include "ordered_structure/leftist_heap.hpp"
int main() {
    cp::LeftistHeap<int> a,b; a.push(2); b.push(7); a.meld(b);
    std::cout << a.top() << " " << b.empty() << "\n";
}
```

出力: `7 1`。

## APIと使用条件

`LeftistHeap<T,Compare=less<T>>`、push/top/pop/size/empty/clear、meld(other)。コピー不可、move可。

std::priority_queueと同じくlessで最大値、greaterで最小値。meldは同じ順序を定める比較器同士、otherは空になります。listのspliceで所有権を移し、mergeと破棄は長い再帰を使いません。pop済み節点はclearまで保持します。

## 計算量

push/pop/meldは最悪O(log(N+1))。 top/size O(1)、記憶量はclearまでの通算挿入数。 演算・比較・値のコピーが O(1) の場合です。文字列などを格納すると、その処理時間・領域が加わります。

## 使用する問題と検証

順位付き多重集合、複数優先度キューの併合、上位k個の動的集計などで使います。std::multiset／priority_queueと操作列を比較します。

`python3 verify/run_b_1_6.py`（`--sanitize` 対応）で単独／重複include、debug/releaseの比較検証、上の使用例を確認します。検証範囲の詳細は[追加分の一覧](../b_1_6/README.md)。オンライン提出・ACは未確認です。

[操作できる図解](../../visualizations/b_1_6/index.html#leftist_heap) / [カテゴリ索引](README.md) / [参考資料と設計判断](../b_1_6/references.md#ordered) / [図解の簡略化](../../visualizations/b_1_6/README.md)
