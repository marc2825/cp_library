---
title: 部分永続Union-Find
documentation_of: //union_find/partially_persistent_disjoint_set_union.hpp
---

# 部分永続Union-Find

最新版に併合し、過去時刻の連結性と成分サイズを照会。

## API・使用条件

PartiallyPersistentDisjointSetUnion(n)、merge(u,v)->bool、time()、leader(t,v)、same(t,u,v)、size(t,v)。時刻0は初期状態。

失敗したmergeも時刻を1進める。0<=t<=time()、頂点は[0,n)。merge回数はINT_MAX以下。親が変わる時刻と根のサイズ履歴を保持し経路圧縮しない。

C++17、namespace cp。特記以外は0始まり・半開区間。前提条件は呼出し側が守り、添字/範囲の違反はdebug assertで検査します。整数の全中間値とコンテナの添字が使用型に収まることが必要です。計算量は要素の演算/比較/コピーをO(1)としたものです。

## Usage

```cpp
#include <iostream>
#include "union_find/partially_persistent_disjoint_set_union.hpp"
int main() {
    cp::PartiallyPersistentDisjointSetUnion d(3); d.merge(0,1); d.merge(1,2);
    std::cout<<d.same(0,0,1)<<" "<<d.size(1,0)<<" "<<d.size(2,0)<<"\n";
}
```

出力: `0 2 3`。

## 計算量

構築O(N)、併合/照会O(log(N+1))、全体O(N+M)領域。


## 問題例・検証・参考資料

用途に対応する[問題例と一次資料](../a_priority/references.md)を参照。`python3 verify/run_a_priority.py`（`--sanitize`対応）で単独/重複include、複数翻訳単位、上記使用例と独立した素朴解の差分検証を行います。[検証範囲](../a_priority/README.md)。オンラインACは未確認です。

[操作できる図解](../../visualizations/a_priority/index.html#partially_persistent_disjoint_set_union) / [図の範囲](../../visualizations/a_priority/README.md) / [カテゴリ索引](README.md) / [動く例](../../examples/union_find/partially_persistent_disjoint_set_union.cpp)
