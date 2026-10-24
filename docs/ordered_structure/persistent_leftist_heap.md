---
title: Persistent Leftist Heap
documentation_of: //ordered_structure/persistent_leftist_heap.hpp
---

# Persistent Leftist Heap（D-06）

永続 leftist heap の汎用公開 API です。古い root をそのまま残して push/pop/meld できます。k_shortest_walks の内部 heap もこの実装を共有します。

## API・使用条件

`PersistentLeftistHeap<T,Compare>(compare={})`、`empty_root()` (=0)、`push(root,value)`、`pop(root)`、`meld(a,b)`、`build(first,last)` は root（int）を返します。`top(root)`、`size(root)`、`empty(root)`、`children(root)`、`node_count()` で問い合わせます。

**既存 LeftistHeap / std::priority_queue と同じ向き**で、less は最大、greater は最小が先頭です。top/pop に非空 root が必要です。children(0) は (0,0)。T はコピー可能、デフォルト構築・== は不要です。

root は生成した pool 専用です。他の pool の ID を渡してはいけません。pool が全ノードを vector で所有し、root の破棄では領域を回収しません。pool の破棄は平坦で、左の長い鎖も再帰破棄しません。top の参照はノード追加による vector 再確保で無効になります。

meld は多重集合の和で、同じ版同士の meld や共有部分も重複として数えます。size_t の要素数上限および int のノード上限は length_error。既存版は更新・例外でも保持しますが、失敗までに確保した未到達ノードが残る場合があります。pool のコピーは全ノードの独立コピーで、コピー時点の ID を同じ番号で使えます。

右の最短距離 rank を左以下に保ち、右鎖だけ経路コピーします。build は新しい singleton を pairwise に破壊的 meld するため、過去版を壊さず線形構築できます。

C++17、namespace cp、0始まり・半開区間。添字・範囲の前提は debug assert で検査します。比較・コピー・集約演算を O(1) として計算量を示します。整数の途中値は使用型に収まることが必要です。

## Usage

```cpp
#include <functional>
#include <iostream>
#include "ordered_structure/persistent_leftist_heap.hpp"
int main() {
    cp::PersistentLeftistHeap<int,std::greater<int>> h;
    int a=h.push(0,4), b=h.push(a,1), c=h.push(a,7);
    int d=h.meld(b,c), next=h.pop(d);
    std::cout << h.top(a) << " " << h.top(b) << " " << h.top(c) << "\n";
    std::cout << h.size(d) << " " << h.top(next) << "\n";
}
```

```text
4 1 4
4 4
```

## 計算量

push/pop/meld は O(log(a+b+1)) 時間・追加ノード（空との meld は O(1)）。top/size/empty/children/node_count O(1)、build は k 要素に O(k) 時間・追加ノード。全領域は通算確保ノード数、pool のコピー・破棄もその数に比例します。

## 問題例・検証・参考資料

例題: 分岐する版ごとの優先度キュー、Eppstein の k 最短 walk。12,000 の版分岐を multiset と比較し、同一版 meld、非デフォルト型、20万の線形構築・全削除、要素数 overflow を検証。既存 k_shortest_walks の独立最短 walk 比較も回帰実行します。

[共通の検証入口・対応範囲](../data_structure_extensions/README.md) / [一次資料と設計判断](../data_structure_extensions/references.md)。オンライン AC は未確認です。

[操作できる図解](../../visualizations/data_structure_extensions/index.html#persistent_leftist_heap) / [図解の範囲](../../visualizations/data_structure_extensions/README.md) / [動く例](../../examples/ordered_structure/persistent_leftist_heap.cpp) / [カテゴリ索引](README.md)
