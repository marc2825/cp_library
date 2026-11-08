---
title: K-th Shortest Walk
documentation_of: //graph_basic/k_shortest_walks.hpp
---

# K-th Shortest Walk

頂点・辺の繰り返しを許すwalkをコスト順に列挙し、必要なものだけ経路復元。

## API・使用条件

k_shortest_walks(Graph<T>,s,t,k) -> KShortestWalksResult<T>。costs[i]、path(i)->WeightedPath<T>。k>=0。

Tは非負整数、bool以外。有向/無向・自己ループ・多重辺可。無向自己ループの隣接表の二つの出現は一つの論理辺として数える。辺列の異なるwalkを別々に数え、同コストの順序は未保証。0閉路では同コストのwalkが無限個になり得る。s=tでは空walkが最初。返す数は高々k。途中距離とsidetrack差分は型の最大値未満。

C++17、namespace cp、0始まり。頂点・辺IDと結果の公開配列は読み取り用です。入力を変えたら再構築/再計算してください。前提条件は呼出し側が守り、添字・符号などはdebug assertで検査します。整数の全中間値は使用型に収まる必要があります。

## Usage

```cpp
#include <iostream>
#include "graph_basic/k_shortest_walks.hpp"
int main() {
    cp::Graph<long long> g(2,cp::GraphKind::Directed);g.add_edge(0,1,2);g.add_edge(1,1,1);
    auto a=cp::k_shortest_walks(g,0,1,3);for(auto c:a.costs) std::cout<<c<<" ";
    std::cout<<"\n"<<a.path(2).edge_ids.size()<<"\n";
}
```

出力: `2 3 4
3`。

## 原理・計算量

Eppstein法。終点への最短路木から外れる辺の増分delta=w+d[to]-d[from]を計算する。木の経路上の逸脱候補を永続leftist heapで共有し、候補の置換/追加から次のwalkを作る。結果はsidetrackの連結履歴だけ持ち、長いwalkを全て保存しない。

O((N+M)log(N+M+1)+K log(K+1))時間、O(N+M+N log(M+1)+K)領域。path(i)は復元する辺数LにO(L)時間・領域。

## 問題例・検証・参考資料

[一次資料・対象ごとの問題例](../graph_a/references.md)。`python3 verify/run_graph_a.py` / `--sanitize` で単独/重複include、複数翻訳単位、使用例、独立した全探索/素朴解と比較します。[検証範囲](../graph_a/README.md)。オンラインACは未確認です。

[操作できる図解](../../visualizations/graph_a/index.html#k_shortest_walks) / [図の範囲](../../visualizations/graph_a/README.md) / [動く例](../../examples/graph_basic/k_shortest_walks.cpp) / [カテゴリ索引](README.md)

2026-10-24から、内部の永続heapは[公開PersistentLeftistHeap](../ordered_structure/persistent_leftist_heap.md)を共有しています。各頂点の局所heapは線形build、親との結合は永続meldです。旧APIと最短walkの計算量は保持します。
