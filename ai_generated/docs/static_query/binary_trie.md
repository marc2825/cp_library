---
title: Binary Trie
documentation_of: //static_query/binary_trie.hpp
---

# Binary Trie

整数多重集合の順位・k番目・XOR一括変更・最小XOR側の検索。

## API・使用条件

BinaryTrie<U=uint64_t,Bits=digits>、insert(x)、erase(x)->bool、count(x)、size/empty/clear、xor_all(x)、kth(k)、kth_xor(k,x)、order_of_key(x)。

Uはbool以外の符号なし整数、1<=Bits<=digits、値とXORはBits内。重複可、eraseは一つ削除。kは0始まり、範囲外はnullopt。kth_xorはXOR距離ではなく格納値を返す。count総数はLLONG_MAX以下。

C++17、namespace cp。特記以外は0始まり・半開区間。前提条件は呼出し側が守り、添字/範囲の違反はdebug assertで検査します。整数の全中間値とコンテナの添字が使用型に収まることが必要です。計算量は要素の演算/比較/コピーをO(1)としたものです。

## Usage

```cpp
#include <iostream>
#include "static_query/binary_trie.hpp"
int main() {
    cp::BinaryTrie<> s; s.insert(1); s.insert(6); s.insert(6); s.xor_all(3);
    std::cout<<*s.kth(0)<<" "<<*s.kth_xor(0,7)<<"\n";
}
```

出力: `2 5`。

## 計算量

操作O(Bits)、K個の新規挿入でO(1+K Bits)領域。削除ノードはclearまで保持。


## 問題例・検証・参考資料

用途に対応する[問題例と一次資料](../a_priority/references.md)を参照。`python3 verify/run_a_priority.py`（`--sanitize`対応）で単独/重複include、複数翻訳単位、上記使用例と独立した素朴解の差分検証を行います。[検証範囲](../a_priority/README.md)。オンラインACは未確認です。

[操作できる図解](../../visualizations/a_priority/index.html#binary_trie) / [図の範囲](../../visualizations/a_priority/README.md) / [カテゴリ索引](README.md) / [動く例](../../examples/static_query/binary_trie.cpp)
