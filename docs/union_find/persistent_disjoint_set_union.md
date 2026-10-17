---
title: 完全永続Union-Find
documentation_of: //union_find/persistent_disjoint_set_union.hpp
---

# 完全永続Union-Find

任意の版を親に新しい併合版を作る。永続セグ木の一点取得/更新を共有。

## API・使用条件

PersistentDisjointSetUnion(n)、empty_root()、merge(root,u,v)->root、leader(root,v)、same(root,u,v)、size(root,v)。

rootは同じオブジェクトが返したものに限る。0は全孤立、同成分のmergeは同じrootを返す。union by size、経路圧縮なし。過去の親/サイズ配列を更新しない。

C++17、namespace cp。特記以外は0始まり・半開区間。前提条件は呼出し側が守り、添字/範囲の違反はdebug assertで検査します。整数の全中間値とコンテナの添字が使用型に収まることが必要です。計算量は要素の演算/比較/コピーをO(1)としたものです。

## Usage

```cpp
#include <iostream>
#include "union_find/persistent_disjoint_set_union.hpp"
int main() {
    cp::PersistentDisjointSetUnion d(3); int a=d.merge(0,0,1), b=d.merge(0,1,2);
    std::cout<<d.same(a,0,1)<<" "<<d.same(b,0,1)<<"\n";
}
```

出力: `1 0`。

## 計算量

構築O(1)、操作O(log²(N+1))、成功併合1回の追加領域O(log(N+1))。


## 問題例・検証・参考資料

用途に対応する[問題例と一次資料](../a_priority/references.md)を参照。`python3 verify/run_a_priority.py`（`--sanitize`対応）で単独/重複include、複数翻訳単位、上記使用例と独立した素朴解の差分検証を行います。[検証範囲](../a_priority/README.md)。オンラインACは未確認です。

[操作できる図解](../../visualizations/a_priority/index.html#persistent_disjoint_set_union) / [図の範囲](../../visualizations/a_priority/README.md) / [カテゴリ索引](README.md) / [動く例](../../examples/union_find/persistent_disjoint_set_union.cpp)
