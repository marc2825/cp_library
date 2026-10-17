---
title: RBST
documentation_of: //ordered_structure/rbst.hpp
---

# RBST

事前登録なしの動的なキー集合と順位検索。

## API・使用条件

RBST<Key,Compare>(compare,seed)、insert(x)、erase(x)->一個の削除有無、count(x)、order_of_key(x)、kth(k)、lower_bound(x)、size/empty/clear。

重複を個数として保持しinsertはvoid。Compareは厳密弱順序、greaterも可。k範囲外/見つからないlower_boundはnullopt。要素/ノード数はINT_MAX未満。削除ノードはclearまで保持。RBSTは部分木の異なるキー数に比例して根を選ぶ。優先度によるTreapとは別のランダム結合。固定seedで再現可能だが悪意のある入力への最悪時間保証はない。

C++17、namespace cp。特記以外は0始まり・半開区間。前提条件は呼出し側が守り、添字/範囲の違反はdebug assertで検査します。整数の全中間値とコンテナの添字が使用型に収まることが必要です。計算量は要素の演算/比較/コピーをO(1)としたものです。

## Usage

```cpp
#include <iostream>
#include "ordered_structure/rbst.hpp"
int main() {
    cp::RBST<int> s; s.insert(5); s.insert(1); s.insert(3);
    std::cout<<*s.kth(1)<<" "<<s.order_of_key(5)<<"\n";
}
```

出力: `3 2`。

## 計算量

各操作は期待O(log(N+1))、最悪O(N)。作成ノード数KにO(K)領域。


## 問題例・検証・参考資料

用途に対応する[問題例と一次資料](../a_priority/references.md)を参照。`python3 verify/run_a_priority.py`（`--sanitize`対応）で単独/重複include、複数翻訳単位、上記使用例と独立した素朴解の差分検証を行います。[検証範囲](../a_priority/README.md)。オンラインACは未確認です。

[操作できる図解](../../visualizations/a_priority/index.html#rbst) / [図の範囲](../../visualizations/a_priority/README.md) / [カテゴリ索引](README.md) / [動く例](../../examples/ordered_structure/rbst.cpp)
