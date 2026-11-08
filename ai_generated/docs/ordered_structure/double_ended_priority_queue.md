---
title: Double-Ended Priority Queue
documentation_of: //ordered_structure/double_ended_priority_queue.hpp
---

# Double-Ended Priority Queue（D-02）

多重集合の最小・最大を参照し、どちらからでも1個ずつ削除します。標準の `std::multiset` を共有する実装です。遅延削除の古い要素は蓄積しません。

## API・使用条件

`DoubleEndedPriorityQueue<T,Compare>(compare={})`、iterator 範囲の構築、`push(value)`、`min()/max()`、`pop_min()/pop_max()`、`size()/empty()/clear()`。

重複は独立した要素です。既定 `std::less<T>` で数値の最小・最大、`std::greater<T>` では順序が逆になります。比較器は厳密弱順序。同値の要素の識別・安定順序は保証しません。T の `==` やデフォルト構築は不要です。

空集合の min/max/pop は呼べません（debug assert）。min/max は const 参照で、その要素の削除・clear・所有オブジェクトの破棄で無効になります。コピーは独立した集合です。

C++17、namespace cp、0始まり・半開区間。添字・範囲の前提は debug assert で検査します。比較・コピー・集約演算を O(1) として計算量を示します。整数の途中値は使用型に収まることが必要です。

## Usage

```cpp
#include <iostream>
#include "ordered_structure/double_ended_priority_queue.hpp"
int main() {
    cp::DoubleEndedPriorityQueue<int> q;
    for (int x : {4,1,4,9}) q.push(x);
    std::cout << q.min() << " " << q.max() << "\n";
    q.pop_max(); q.pop_min();
    std::cout << q.min() << " " << q.max() << " " << q.size() << "\n";
}
```

```text
1 9
4 4 2
```

## 計算量

push は O(log(n+1))、min/max/size/empty は O(1)、iterator による pop は償却 O(1)（保守的な上界 O(log(n+1))）。範囲構築 O(n log(n+1))、コピー・clear O(n)、領域 O(n)。

## 問題例・検証・参考資料

Library Checker Double-Ended Priority Queue。10万のランダム両端操作をソート済み vector と比較し、重複・空・コピー・逆順比較・非デフォルト構築型を検証します。

[共通の検証入口・対応範囲](../data_structure_extensions/README.md) / [一次資料と設計判断](../data_structure_extensions/references.md)。オンライン AC は未確認です。

[操作できる図解](../../visualizations/data_structure_extensions/index.html#double_ended_priority_queue) / [図解の範囲](../../visualizations/data_structure_extensions/README.md) / [動く例](../../examples/ordered_structure/double_ended_priority_queue.cpp) / [カテゴリ索引](README.md)
