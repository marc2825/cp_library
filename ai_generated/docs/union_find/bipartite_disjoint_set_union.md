---
title: 二部グラフ判定付きUnion-Find
documentation_of: //union_find/bipartite_disjoint_set_union.hpp
---

# 二部グラフ判定付きUnion-Find

無向辺追加のたびに成分ごと/全体の二部性を維持。

## API・使用条件

BipartiteDisjointSetUnion(n)、add_edge(u,v)->追加後の成分の二部性、is_bipartite()、component_is_bipartite(v)、same/leader/size。

矛盾辺も追加済みとして保持し、奇閉路がある成分は後の併合でも失敗状態を保つ。自己ループは失敗、重複辺は可。親との色差のXORを経路圧縮と同時に更新。

C++17、namespace cp。特記以外は0始まり・半開区間。前提条件は呼出し側が守り、添字/範囲の違反はdebug assertで検査します。整数の全中間値とコンテナの添字が使用型に収まることが必要です。計算量は要素の演算/比較/コピーをO(1)としたものです。

## Usage

```cpp
#include <iostream>
#include "union_find/bipartite_disjoint_set_union.hpp"
int main() {
    cp::BipartiteDisjointSetUnion d(3); d.add_edge(0,1); d.add_edge(1,2);
    std::cout<<d.add_edge(2,0)<<" "<<d.is_bipartite()<<"\n";
}
```

出力: `0 0`。

## 計算量

構築O(N)、追加/成分照会ならしO(α(N))、全体判定O(1)、領域O(N)。


## 問題例・検証・参考資料

用途に対応する[問題例と一次資料](../a_priority/references.md)を参照。`python3 verify/run_a_priority.py`（`--sanitize`対応）で単独/重複include、複数翻訳単位、上記使用例と独立した素朴解の差分検証を行います。[検証範囲](../a_priority/README.md)。オンラインACは未確認です。

[操作できる図解](../../visualizations/a_priority/index.html#bipartite_disjoint_set_union) / [図の範囲](../../visualizations/a_priority/README.md) / [カテゴリ索引](README.md) / [動く例](../../examples/union_find/bipartite_disjoint_set_union.cpp)
