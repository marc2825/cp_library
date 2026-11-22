---
title: Dominator Tree
documentation_of: //graph_connectivity/dominator_tree.hpp
---

# Dominator Tree

[カテゴリ索引](README.md) / [B項目一覧](../b_7_11/README.md) / [操作できる図解](../../visualizations/b_7_11/index.html#dominator_tree) / [実装](../../graph_connectivity/dominator_tree.hpp) / [一次資料と設計](../b_7_11/references.md)

指定始点から頂点vへ至る全経路が通る頂点を、即時支配頂点の木として求めます。Lengauer–Tarjanの半支配頂点とlink/evalを使います。

## usage

```cpp
#include <iostream>
#include "graph_connectivity/dominator_tree.hpp"
int main() {
    cp::Graph<long long> g(3,cp::GraphKind::Directed);
    g.add_edge(0,1); g.add_edge(1,2); g.add_edge(0,2);
    auto d=cp::dominator_tree(g);
    std::cout << d.idom[2] << ' ' << d.dominates(1,2) << '\n';
}
```

出力:

```text
0 0
```

## API・使用条件

`dominator_tree(g,root=0)` → `DominatorTreeResult{idom,tin,tout}`。有向（無向グラフは`invalid_argument`）。根のidomは自身、未到達は−1。`dominates(u,v)`は未到達vに対してfalse。空グラフ可。

頂点・辺IDは0始まり。グラフを受ける関数は構築時点の入力に対する計算です。重みや値の型は特記した制約に従います。

## 仕組みと境界

DFS・eval圧縮・支配木の巡回は非再帰。u=vも支配に含みますが、未到達の頂点は支配関係に含めません。辺重みは無視します。

## 計算量

時間O((V+E) log(V+1))の上界、領域O(V+E)。 Vは頂点数、Eは論理辺数。

## 使用する問題・検証

想定問題: [Library Checker: Dominator Tree](https://judge.yosupo.jp/problem/dominatortree)（根は自身・未到達は−1 という出力形式がそのまま `idom` に一致）。 [実行できる小問題](../../examples/graph_connectivity/dominator_tree_b.cpp)。

検証では、n≤9・m≤29 の有向600グラフで、各頂点を取り除いたときの到達性から `dominates` を全頂点対で比較し、`idom` が最も近い真の支配頂点であることを確かめます。10万頂点の鎖で再帰を使わないことも確認します。上の使用例は実際にコンパイル・実行し、文書中のコードとの一致も検査します。

`python3 verify/run_b_7_11.py` / `--sanitize`。[検証コード](../../verify/b_7_11/graph_test.cpp)。[比較方法](../b_7_11/README.md#ローカル検証)を参照してください。オンライン提出・ACは未確認です。図解は小入力の教材モデルで、C++との差異は[図解の説明](../../visualizations/b_7_11/README.md)へ明記しています。
