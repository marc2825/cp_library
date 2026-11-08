---
title: トポロジカルソート
documentation_of: //graph_basic/topological_sort.hpp
---

# トポロジカルソート

有向グラフで、全ての辺が前から後へ向く頂点順を求めます。 [操作できる図解](../../visualizations/graph_basic/index.html#topological)で入力と内部状態を変更して追えます。

## API・使用条件

```text
cp::topological_sort(g) -> optional<vector<int>>
```

有向グラフ用。空グラフ・多重辺・自己ループにも対応。閉路があると nullopt、空の DAG なら値を持つ空 vector を返します。


## usage

```cpp
#include <cassert>
#include <vector>
#include "graph_basic/topological_sort.hpp"
int main() {
    cp::Graph<> g(3, cp::GraphKind::Directed);
    g.add_edge(0,1); g.add_edge(1,2);
    auto order=cp::topological_sort(g);
    assert(order && *order==(std::vector<int>{0,1,2}));
    g.add_edge(2,0);
    assert(!cp::topological_sort(g));
}
```

## 原理と注意点

Kahn 法です。入次数 0 の頂点を FIFO キューに入れ、処理した出辺の行先の入次数を減らします。全頂点を出力できたときだけ成功します。

順序は一つ返し、辞書順最小を保証しません。処理できず残った頂点には閉路の先にある頂点も含まれます。閉路そのものが欲しい場合は find_cycle を使います。

## 計算量

時間 O(V+E)、追加メモリ O(V)。 E は論理辺の数で、無向辺の隣接リストには両方向を保持します。入力グラフ自体の O(V+E) メモリは別途必要です。

## 問題・検証・参考

[AOJ GRL_4_B: Topological Sort](https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=GRL_4_B) / [ジャッジ形式](../../verify/graph_basic/topological_sort.test.cpp)。ローカルの Python オラクルと比較しており、オンライン AC を実施したという意味ではありません。

600 グラフの差分検証、経路の始終点・辺・重みの検査、DFS の再帰版比較、DAG の順列全探索、最小全域森の辺部分集合全探索、20 万頂点を含む [共通検証](README.md#検証) を用意しています。

[CP-Algorithms の解説](https://cp-algorithms.com/graph/topological-sort.html) を参照しました。[参考資料・設計判断](references.md)。

[DAGのGrundy数計算とゲーム](../game_theory/README.md)では、トポロジカル順の逆順に次局面から値を求めます。
