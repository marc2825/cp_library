---
title: Disjoint Sparse Table
documentation_of: //static_query/disjoint_sparse_table.hpp
---

# Disjoint Sparse Table

結合則だけで静的な区間積をO(1)照会。

## API・使用条件

DisjointSparseTable<S,op,e>(vector)、prod(l,r)、size()。

空区間はe()、一点は元の値。lとr-1の最上位の異なるbitを使い、分割点を挟むsuffix/prefixを結合。冪等性や可換性は不要。

C++17、namespace cp。特記以外は0始まり・半開区間。前提条件は呼出し側が守り、添字/範囲の違反はdebug assertで検査します。整数の全中間値とコンテナの添字が使用型に収まることが必要です。計算量は要素の演算/比較/コピーをO(1)としたものです。

## Usage

```cpp
#include <iostream>
#include "static_query/disjoint_sparse_table.hpp"
long long add(long long a,long long b){return a+b;} long long zero(){return 0;}
int main() {
    cp::DisjointSparseTable<long long,add,zero> s({2,3,5,7});
    std::cout<<s.prod(1,4)<<" "<<s.prod(2,2)<<"\n";
}
```

出力: `15 0`。

## 計算量

構築/領域O(N log(N+1))、prodのモノイド演算は高々1回。bit位置探索は固定32bitのO(1)。


## 問題例・検証・参考資料

用途に対応する[問題例と一次資料](../a_priority/references.md)を参照。`python3 verify/run_a_priority.py`（`--sanitize`対応）で単独/重複include、複数翻訳単位、上記使用例と独立した素朴解の差分検証を行います。[検証範囲](../a_priority/README.md)。オンラインACは未確認です。

[操作できる図解](../../visualizations/a_priority/index.html#disjoint_sparse_table) / [図の範囲](../../visualizations/a_priority/README.md) / [カテゴリ索引](README.md) / [動く例](../../examples/static_query/disjoint_sparse_table.cpp)
