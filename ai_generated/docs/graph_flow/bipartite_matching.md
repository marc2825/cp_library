---
title: 二部最大マッチング（Hopcroft–Karp）
documentation_of: //graph_flow/bipartite_matching.hpp
---

# 二部最大マッチング

左の頂点と右の頂点から、一つの頂点を高々一回だけ使って、辺で結ばれた組を最大数作ります。仕事と人の割当、ドミノ配置、二部グラフの独立集合・被覆への帰着等で使います。

[交互路と組み替えを操作する図解](../../visualizations/graph_flow/index.html#matching) / [カテゴリ索引](README.md)

## 原理

未マッチの左から、未採用辺を左→右、採用辺を右→左に辿ります。未マッチの右へ届く交互路が見つかれば、採用と未採用を反転して組が一つ増えます。早く決めた組を固定する貪欲法では、この組み替えができません。

Hopcroft–Karp は複数の未マッチの左を根とする BFS で最短の層を作り、同じ最短長の交互路を反復 DFS で探します。層は左頂点間の距離で、未採用辺と採用辺の一組を一段と数えます。未マッチの右への最後の辺も含めた左→右の区間数を最短長として管理します。

## usage

```cpp
#include <cassert>
#include "graph_flow/bipartite_matching.hpp"
int main() {
    cp::BipartiteMatching graph(2, 2);
    graph.add_edge(0, 0);
    graph.add_edge(0, 1);
    graph.add_edge(1, 0);
    const auto result = graph.maximum_matching();
    assert(result.cardinality == 2);
    assert(result.left_match[0] == 1 && result.left_match[1] == 0);
    assert(result.right_match[0] == 1 && result.right_match[1] == 0);
    assert(result.pairs.size() == 2);
}
```

[動く使用例](../../examples/graph_flow/bipartite_matching.cpp) / [実装](../../graph_flow/bipartite_matching.hpp)

## API と条件

| API | 意味 | 時間 |
|---|---|---|
| `BipartiteMatching(left_size,right_size)` | 左右の頂点数を指定 | O(L) |
| `left_size()` / `right_size()` | 各側の頂点数 | O(1) |
| `add_edge(left,right)` | 左右を結ぶ辺を追加 | ならし O(1) |
| `maximum_matching() const` | 空のマッチングから最大解を計算し、値で返す | O((V+E)√(V+1)) |

`BipartiteMatchingResult`:

| メンバー | 意味 |
|---|---|
| `cardinality` | 最大の組数 |
| `left_match[u]` | 左 u の右の相手。未マッチは −1 |
| `right_match[v]` | 右 v の左の相手。未マッチは −1 |
| `pairs` | `(left,right)` の組一覧。left の昇順 |

V=L+R、E は重複を含む登録辺数です。グラフ本体 O(L+E)、計算作業と結果 O(L+R)。DFS の明示スタックも O(L)。上界は孤立頂点を含む頂点走査を考慮しています。

- L,R≥0。左右は**別々の番号空間**です。left=0 と right=0 を結ぶ辺も通常の辺であり、自己ループではありません。
- 同じ辺の重複は許容します。一つの組を重複して採用することはありません。左右どちらかが空なら最大組数 0。
- 分割を決める機能や、一般グラフのマッチングは含みません。無向グラフから使う場合は [3.8 の二部判定](../graph_connectivity/bipartite.md) を行い、色ごとに番号を振り直します。
- 最大の組数を保証します。pairs の並びは左昇順ですが、最大解同士での辞書順最小の選択は保証しません。
- 呼出しごとに空のマッチングから再計算します。辺を追記して再実行できます。以前の result は変化しません。増分更新・辺削除・辺 ID の復元 API は提供しません。

## Dinic との関係・問題例

s→左、左→右、右→t をそれぞれ容量 1 で作ると、同じ問題を Dinic でも解けます。matching はこの二部構造を直接扱う実装です。

- [Library Checker Bipartite Matching](https://judge.yosupo.jp/problem/bipartitematching)：組一覧まで出力する [検証プログラム](../../verify/graph_flow/bipartite_matching.test.cpp)。
- [AOJ GRL_7_A Bipartite Matching](https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=GRL_7_A)：組数を出力する [検証プログラム](../../verify/graph_flow/matching_size.test.cpp)。
- [AtCoder practice2 D Maxflow](https://atcoder.jp/contests/practice2/tasks/practice2_d)：市松模様の二部グラフでドミノ配置を求めます。

3×3 までの全 689 グラフ、700 ランダム入力を独立した右部分集合 DP と比較し、組の存在・左右の一意性・相互対応・繰り返しと追記も確認します。さらに Dinic への帰着とも照合します。20 万左頂点の長い交互路を反復処理する検証を含みます。オンライン AC 済みとはしていません。

[参考資料](references.md)
