---
title: 距離の結果型・経路復元
documentation_of: //graph_basic/shortest_path.hpp
---

# ShortestPathResult と経路復元

単一始点の最短路は `ShortestPathResult<T>` で共通化しています。BFS／単一始点 DFS の T は int、重み付きアルゴリズムは Graph の重み型です。Warshall–Floyd はこれを始点ごとに並べた vector を返します。

[操作できる図解](../../visualizations/graph_basic/index.html#path) は Dijkstra の結果から、終点→始点へ親を辿り、列を反転する過程を表示します。他の最短路モードでも終点を変更して復元できます。

| API・フィールド | 意味 |
|---|---|
| `source` | 始点 |
| `dist[v]` | 距離。到達不能は `distance_inf<T>()` |
| `parent[v]`, `parent_edge[v]` | 直前の頂点、論理辺 ID。通常の始点・到達不能は −1 |
| `negative_cycle[v]` | 始点から届く負閉路が v の距離へ影響するか |
| `reachable(v)` | 到達可能か。負閉路の影響下でも true |
| `has_finite_distance(v)` | 到達可能で、負閉路の影響がないか |
| `vertex_path(v)` | 始点から v の頂点列 |
| `edge_path(v)` | 同じ経路の論理辺 ID 列 |

到達不能／負閉路の影響下では両方空列。始点自身への頂点列は `{source}`、辺列は空です。始点自身が負閉路の影響下なら頂点列も空です。負閉路の影響下での parent や dist は有効な最短路情報として使用しません。結果の公開配列は読み取り用として扱い、書き換えてから復元しないでください。

## usage

```cpp
#include <cassert>
#include <vector>
#include "graph_basic/dijkstra.hpp"
int main() {
    cp::Graph<> g(3,cp::GraphKind::Directed);
    g.add_edge(0,1,8); g.add_edge(0,1,2); // 多重辺を ID で区別
    auto r=cp::dijkstra(g,0);
    assert(r.vertex_path(1)==(std::vector<int>{0,1}));
    assert(r.edge_path(1)==(std::vector<int>{1}));
    assert(r.vertex_path(0)==(std::vector<int>{0}));
    assert(r.edge_path(0).empty() && r.vertex_path(2).empty());
}
```

始点から全頂点分を計算した後、目的の終点の経路だけを復元できます。親を逆に辿って配列へ入れ、最後に reverse します。多重辺では端点だけで使用した辺を特定できないため、`parent_edge` を保存します。

結果の保持は O(V)、reachable／has_finite_distance は O(1)、各復元は経路長 L に対して O(L) 時間・出力メモリ。Warshall–Floyd は全結果で O(V²)。数値範囲は [共通の約束](README.md#共通の約束) を参照してください。

[Library Checker: Shortest Path](https://judge.yosupo.jp/problem/shortest_path) / [ジャッジ形式](../../verify/graph_basic/shortest_path.test.cpp) / [動く例](../../examples/graph_basic/shortest_paths.cpp)。600 グラフで、各有限距離の経路について始終点・辺 ID・向き・重み・頂点の重複がないことを検査します。到達不能・負閉路・始点自身・多重辺・20 万頂点も含みます。[検証](README.md#検証)。

参考: [CP-Algorithms の Dijkstra と経路復元](https://cp-algorithms.com/graph/dijkstra.html) / [一次問題仕様](https://github.com/yosupo06/library-checker-problems/blob/master/graph/shortest_path/task.md) / [参考資料](references.md)。
