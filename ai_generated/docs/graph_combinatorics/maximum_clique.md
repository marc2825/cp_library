---
title: 最大クリーク
documentation_of: //graph_combinatorics/maximum_clique.hpp
---

# 最大クリーク

単純無向グラフから、全ての二頂点が隣接する最大サイズの集合を一つ返します。全列挙を保存する必要はありません。

[貪欲彩色の上界と枝刈りの図解](../../visualizations/graph_combinatorics/index.html#maximum)

## 原理

現在のクリーク Q に追加可能な頂点集合 P を貪欲彩色します。c 色で塗れれば P 内のクリークは高々 c 頂点です。`|Q|+c<=|best|` なら、この枝では最良解を更新できません。色番号の昇順で候補を並べ、後ろから選ぶことで、残った候補に対する色の上界を使います。貪欲彩色を最小彩色だとみなす必要はありません。

## usage

```cpp
#include <cassert>
#include <vector>
#include "graph_combinatorics/maximum_clique.hpp"
int main() {
    cp::Graph<> g(4);
    g.add_edge(0,1); g.add_edge(0,2); g.add_edge(1,2); g.add_edge(2,3);
    auto vertices=cp::maximum_clique(g);
    assert((vertices==std::vector<int>{0,1,2}));
}
```

## API・条件・計算量

`std::vector<int> maximum_clique(const Graph<T>& g)`。元の頂点番号を昇順で返します。空グラフは空列、辺のない非空グラフは一頂点。最大解が複数ある場合、どの集合を選ぶかの辞書順保証はありません。

単純・無向、N≤64。重みは無視します。65頂点以上は length_error。64ビットの補集合を取るときは頂点範囲でマスクし、64のシフトを避けます。再帰の深さは最大64。

前処理 O(N+M)、最悪 O(N²2^N) 時間、作業領域 O(N²+N+M)。ビット集合で隣接候補の交差を行いますが、64は表現の上限であり、全ての64頂点グラフの実行時間保証ではありません。

## 問題・検証

[AtCoder ABC002 D 派閥](https://atcoder.jp/contests/abc002/tasks/abc002_4) の [ラッパー](../../verify/graph_combinatorics/maximum_clique.test.cpp) は1始まり入力を0始まりへ直して解きます。[動く例](../../examples/graph_combinatorics/optimization.cpp)。

全頂点部分集合で最大サイズを比較し、返された集合の隣接条件を確認します。空・完全・辺なし・非連結、64頂点の境界を含みます。図解では枝刈り時の上界も独立に検査します。[カテゴリの検証](README.md#動く例問題検証) / [参考資料](references.md)
