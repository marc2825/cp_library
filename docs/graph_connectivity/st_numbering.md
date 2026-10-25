---
title: st-Numbering・Bipolar Orientation
documentation_of: //graph_connectivity/st_numbering.hpp
---

# st-Numbering・Bipolar Orientation

sを先頭、tを末尾とし、中間頂点が小さい番号と大きい番号の隣接頂点をともに持つ順序を作ります。小さい番号から大きい番号へ辺を向ければ、唯一の始点sと終点tを持つDAGになります。

[操作できる図解](../../visualizations/graph_extensions/index.html#st_numbering) / [15項目の検証](../graph_extensions/README.md) / [実装](../../graph_connectivity/st_numbering.hpp)

## usage

```cpp
#include <iostream>
#include "graph_connectivity/st_numbering.hpp"
int main() {
    cp::Graph<int> g(4);
    g.add_edge(0,1); g.add_edge(1,2); g.add_edge(2,3); g.add_edge(3,0);
    auto r=cp::st_numbering(g,0,2);
    if (r) std::cout << r->order.front() << ' ' << r->order.back() << ' ' << r->oriented_edges.size() << '\n';
}
```

出力:

```text
0 2 4
```

## API・使用条件

`st_numbering(g,s,t)`はoptional結果。`order[i]`がi番目の頂点、`rank[v]`が番号、`oriented_edges[id]`が元辺IDに対応する向き付き端点pair。

無向・loopなし、多重辺可。s≠tで有効な端点を指定します。条件違反はinvalid_argument。仮想辺s-tを加えたグラフが二重連結なら構成でき、元のst辺は必須ではありません（n=2は接続辺があれば可）。経路状グラフでも両端をs,tにすれば可能。構成不能・非連結はnullopt。DFS lowpointと符号付きリスト挿入を反復実装し、最終的に元辺で条件を再確認します。

## 計算量

O(V+E)時間・空間。再帰DFSを使いません。

## 使用する問題・検証

双極向き付け、平面グラフ描画、始点と終点を固定した順序付け。

[動く使用例](../../examples/graph_connectivity/st_numbering.cpp)と[独立比較テスト](../../verify/graph_extensions/)を`make verify-graph-extensions`で実行します。小規模全探索、復元した頂点・辺ID、最適値、境界・不正入力を確認します。オンラインACは未確認です。

[一次資料・設計判断](../graph_extensions/references.md) / [カテゴリ索引](README.md)。
