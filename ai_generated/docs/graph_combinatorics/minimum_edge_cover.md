---
title: 最小辺被覆
documentation_of: //graph_combinatorics/minimum_edge_cover.hpp
---

# 最小辺被覆

全頂点が少なくとも1本の選択辺に接するよう、辺数を最小化します。最大matchingの辺を採用し、残る未対応頂点に任意の接続辺を補います。

[操作できる図解](../../visualizations/graph_extensions/index.html#minimum_edge_cover) / [15項目の検証](../graph_extensions/README.md) / [実装](../../graph_combinatorics/minimum_edge_cover.hpp)

## usage

```cpp
#include <iostream>
#include "graph_combinatorics/minimum_edge_cover.hpp"
int main() {
    cp::Graph<int> g(4);
    g.add_edge(0,1); g.add_edge(1,2); g.add_edge(2,0); g.add_edge(2,3);
    auto ids=cp::minimum_edge_cover(g);
    if (ids) std::cout << ids->size() << '\n';
}
```

出力:

```text
2
```

## API・使用条件

`minimum_edge_cover(g)`は一般無向グラフ、`bipartite_minimum_edge_cover(L,R,edges)`は二部専用。どちらも`optional<vector<int>>`で元辺IDを返します。

無向・自己ループなし、多重辺可。二部版IDは入力edgesの添字。孤立頂点がある場合はnullopt、空グラフは空の被覆。被覆の大きさはV−最大matching数。同じ端点の平行辺では最初のIDを使います。一般版は有向・loopでinvalid_argument。

## 計算量

一般版O(V³+E)、二部版O((V+E)√(V+1))。復元O(V+E)、空間O(V+E)。

## 使用する問題・検証

全施設を端点として含む最小本数の接続線選択。一般の奇閉路も扱います。

[動く使用例](../../examples/graph_combinatorics/minimum_edge_cover.cpp)と[独立比較テスト](../../verify/graph_extensions/)を`make verify-graph-extensions`で実行します。小規模全探索、復元した頂点・辺ID、最適値、境界・不正入力を確認します。オンラインACは未確認です。

[一次資料・設計判断](../graph_extensions/references.md) / [カテゴリ索引](README.md)。
