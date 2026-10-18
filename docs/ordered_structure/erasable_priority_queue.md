---
title: Erasable Priority Queue
documentation_of: //ordered_structure/erasable_priority_queue.hpp
---

# Erasable Priority Queue

挿入heapと削除予約heapで遅延削除します。個数辞書で存在を確認し、不在の値を消して後続の同値挿入へ影響させることを防ぎます。

## 使用例

```cpp
#include <iostream>
#include "ordered_structure/erasable_priority_queue.hpp"
int main() {
    cp::ErasablePriorityQueue<int> s; s.push(4); s.push(9); s.erase(9);
    std::cout << s.top() << "\n";
}
```

出力: `4`。

## APIと使用条件

`ErasablePriorityQueue<T,Compare=less<T>>`、push、eraseは一個削除しbool、top/pop/size/empty/clear。

lessで最大値、greaterで最小値。同値は比較器による。Tはコピー可能。空のtop/popは禁止。topの参照は次の変更操作までだけ有効。

## 計算量

各操作は償却O(log(N+M))、予約の一括清掃があるため単発O(M log(N+M))、領域O(N+未回収予約数)。 演算・比較・値のコピーが O(1) の場合です。文字列などを格納すると、その処理時間・領域が加わります。

## 使用する問題と検証

順位付き多重集合、複数優先度キューの併合、上位k個の動的集計などで使います。std::multiset／priority_queueと操作列を比較します。

`python3 verify/run_b_1_6.py`（`--sanitize` 対応）で単独／重複include、debug/releaseの比較検証、上の使用例を確認します。検証範囲の詳細は[追加分の一覧](../b_1_6/README.md)。オンライン提出・ACは未確認です。

[操作できる図解](../../visualizations/b_1_6/index.html#erasable_priority_queue) / [カテゴリ索引](README.md) / [参考資料と設計判断](../b_1_6/references.md#ordered) / [図解の簡略化](../../visualizations/b_1_6/README.md)
