---
title: 閉路検出・復元
documentation_of: //graph_basic/cycle.hpp
---

# 閉路検出・復元

閉路を一つ検出し、頂点列と辺番号列を返します。 [操作できる図解](../../visualizations/graph_basic/index.html#cycle)で入力と内部状態を変更して追えます。

## API・使用条件

```text
cp::find_cycle(g,start=-1) -> optional<Cycle>
```

有向・無向、空グラフ、自己ループ・多重辺に対応。start=-1 なら全体、有効な頂点を指定するとその頂点から届く範囲のみを探索します。


## usage

```cpp
#include <cassert>
#include <vector>
#include "graph_basic/cycle.hpp"
int main() {
    cp::Graph<> g(2); // 無向
    g.add_edge(0,1); g.add_edge(0,1);
    auto cycle=cp::find_cycle(g);
    assert(cycle && cycle->size()==2);
    assert(cycle->edges==(std::vector<int>{0,1}));
}
```

## 原理と注意点

再帰を使わない 3 色 DFS で、探索中の祖先へ戻る辺を探します。無向グラフでは親辺 ID の逆向きだけを除外します。親頂点で除外しないため平行辺も検出できます。

vertices の末尾に始点を重ねません。edges[i] は vertices[i] から vertices[(i+1)%size()] への論理辺 ID です。自己ループは長さ 1、無向の平行辺は長さ 2 の閉路になります。最短の閉路は保証しません。

## 計算量

時間 O(V+E)、追加メモリ O(V)。復元する閉路の長さは O(V)。 E は論理辺の数で、無向辺の隣接リストには両方向を保持します。入力グラフ自体の O(V+E) メモリは別途必要です。

## 問題・検証・参考

[Library Checker: Cycle Detection](https://judge.yosupo.jp/problem/cycle_detection) / [ジャッジ形式](../../verify/graph_basic/cycle_detection.test.cpp)。ローカルの Python オラクルと比較しており、オンライン AC を実施したという意味ではありません。

600 グラフの差分検証、経路の始終点・辺・重みの検査、DFS の再帰版比較、DAG の順列全探索、最小全域森の辺部分集合全探索、20 万頂点を含む [共通検証](README.md#検証) を用意しています。

[CP-Algorithms の解説](https://cp-algorithms.com/graph/finding-cycle.html) を参照しました。[参考資料・設計判断](references.md)。
