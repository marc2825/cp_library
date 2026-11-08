---
title: Incremental Minimum Spanning Forest
documentation_of: //graph_basic/incremental_msf.hpp
---

# Incremental Minimum Spanning Forest

辺を1本追加するたびに最小全域森を保ちます。成分をつなぐ辺なら採用し、閉路を作る辺なら既存Link-Cut Treeでパス上の最大辺を調べて交換します。

[操作できる図解](../../visualizations/graph_extensions/index.html#incremental_msf) / [15項目の検証](../graph_extensions/README.md) / [実装](../../graph_basic/incremental_msf.hpp)

## usage

```cpp
#include <iostream>
#include "graph_basic/incremental_msf.hpp"
int main() {
    cp::IncrementalMSF<> forest(4);
    for (auto e:std::vector<cp::IncrementalMSF<>::EdgeInfo>{{0,1,5},{1,2,4},{0,2,1},{2,3,2}}) {
        forest.add_edge(e.from,e.to,e.cost);
        std::cout << cp::optimization_to_string(forest.cost()) << '\n';
    }
    std::cout << forest.selected_edges().size() << '\n';
}
```

出力:

```text
5
9
5
7
3
```

## API・使用条件

`IncrementalMSF<T>(n)`、`add_edge(u,v,w)`。返却Insertionのidは登録ID、selectedはその時点で採用したか、removedは交換で外れた辺IDまたは−1。`cost()`は128bit総重み、`components()`、`selected_edges()`、`edges()`も公開。

固定頂点・無向の辺追加のみ。負重み・平行辺・loop可（loopは不採用）。同重みなら既存辺を保持し、パス最大辺が複数ならID最大を外します。型Tはbool以外64bit以下整数。棄却辺もIDを持ちます。edges()の参照は追加で無効化され得ます。切り離した辺ノードは再利用。LCTにも孤立頂点の`add_vertex(value)`を追加しました。

## 計算量

追加ならしO(log(n+1))、総重み・成分数O(1)、採用辺列挙O(m)。記憶O(n+m)、LCTノードは高々2n−1。

## 使用する問題・検証

道路が逐次増えるネットワークの最小建設費。削除を含む完全動的MSTは対象外です。

[動く使用例](../../examples/graph_basic/incremental_msf.cpp)と[独立比較テスト](../../verify/graph_extensions/)を`make verify-graph-extensions`で実行します。小規模全探索、復元した頂点・辺ID、最適値、境界・不正入力を確認します。オンラインACは未確認です。

[一次資料・設計判断](../graph_extensions/references.md) / [カテゴリ索引](README.md)。
