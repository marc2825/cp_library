---
title: Ordered Set
documentation_of: //ordered_structure/ordered_set.hpp
---

# Ordered Set

事前登録なしの動的なキー集合と順位検索。

## API・使用条件

OrderedSet<Key,Compare>(compare,seed)、insert(x)、erase(x)->一個の削除有無、count(x)、order_of_key(x)、kth(k)、lower_bound(x)、size/empty/clear。

OrderedSetは同値の重複を無視しinsertは変更有無を返す。Compareは厳密弱順序、greaterも可。k範囲外/見つからないlower_boundはnullopt。要素/ノード数はINT_MAX未満。削除ノードはclearまで保持。Treapはランダム優先度を使い、BST順とheap順を同時に保つ。固定seedで再現可能だが悪意のある入力への最悪時間保証はない。

C++17、namespace cp。特記以外は0始まり・半開区間。前提条件は呼出し側が守り、添字/範囲の違反はdebug assertで検査します。整数の全中間値とコンテナの添字が使用型に収まることが必要です。計算量は要素の演算/比較/コピーをO(1)としたものです。

## Usage

```cpp
#include <iostream>
#include "ordered_structure/ordered_set.hpp"
int main() {
    cp::OrderedSet<int> s; s.insert(5); s.insert(1); s.insert(3);
    std::cout<<*s.kth(1)<<" "<<s.order_of_key(5)<<"\n";
}
```

出力: `3 2`。

## 計算量

各操作は期待O(log(N+1))、最悪O(N)。作成ノード数KにO(K)領域。


## 問題例・検証・参考資料

用途に対応する[問題例と一次資料](../a_priority/references.md)を参照。`python3 verify/run_a_priority.py`（`--sanitize`対応）で単独/重複include、複数翻訳単位、上記使用例と独立した素朴解の差分検証を行います。[検証範囲](../a_priority/README.md)。オンラインACは未確認です。

[操作できる図解](../../visualizations/a_priority/index.html#ordered_set) / [図の範囲](../../visualizations/a_priority/README.md) / [カテゴリ索引](README.md) / [動く例](../../examples/ordered_structure/ordered_set.cpp)
