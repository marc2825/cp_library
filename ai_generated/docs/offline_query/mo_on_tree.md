---
title: Mo on Tree
documentation_of: //offline_query/mo_on_tree.hpp
---

# Mo on Tree

森の頂点パスをEuler列の区間に写し、偶奇のtoggleで集計。

## API・使用条件

MoOnTree(Graph<T>,root=0)、add_query(u,v)->ID、run(add_vertex,remove_vertex,answer,block_size=0)。

無向の木/森、重みは無視。閉路/有向入力、異なる成分のパスはinvalid_argument。頂点を両端とも含む。run前に利用者の集計を空へ戻す。終了状態は空とは限らない。非祖先のパスはLCAをanswerの直前直後だけ追加/除去。辺パスの順序付き集約は提供しない。

C++17、namespace cp。特記以外は0始まり・半開区間。前提条件は呼出し側が守り、添字/範囲の違反はdebug assertで検査します。整数の全中間値とコンテナの添字が使用型に収まることが必要です。計算量は要素の演算/比較/コピーをO(1)としたものです。

## Usage

```cpp
#include <iostream>
#include "offline_query/mo_on_tree.hpp"
int main() {
    cp::Graph<int> g(4); g.add_edge(0,1); g.add_edge(1,2); g.add_edge(1,3);
    cp::MoOnTree mo(g); mo.add_query(2,3); int sum=0;
    mo.run([&](int v){sum+=v;},[&](int v){sum-=v;},[&](int){std::cout<<sum<<"\n";});
}
```

出力: `6`。

## 計算量

構築O(N log(N+1))時間/領域、Q問いを幅BでO(QB+N²/B+Q log(Q+1))程度のtoggleと整列。


## 問題例・検証・参考資料

用途に対応する[問題例と一次資料](../a_priority/references.md)を参照。`python3 verify/run_a_priority.py`（`--sanitize`対応）で単独/重複include、複数翻訳単位、上記使用例と独立した素朴解の差分検証を行います。[検証範囲](../a_priority/README.md)。オンラインACは未確認です。

[操作できる図解](../../visualizations/a_priority/index.html#mo_on_tree) / [図の範囲](../../visualizations/a_priority/README.md) / [カテゴリ索引](README.md) / [動く例](../../examples/offline_query/mo_on_tree.cpp)
