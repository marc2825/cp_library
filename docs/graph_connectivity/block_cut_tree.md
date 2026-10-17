---
title: Block-Cut Tree
documentation_of: //graph_connectivity/block_cut_tree.hpp
---

# Block-Cut Tree

既存の二重頂点連結分解を共有し、元頂点とブロックの所属関係を森にする。

## API・使用条件

block_cut_forest(Graph<T>) -> BlockCutForestResult<T>{blocks,forest}。

無向、重みを無視。forestの[0,N)は全ての元頂点、[N,N+B)はブロック。関節点だけを残す表現ではない。橋は2頂点のブロック、孤立点は単独ブロック、自己ループは無視、多重辺可。

C++17、namespace cp、0始まり。頂点・辺IDと結果の公開配列は読み取り用です。入力を変えたら再構築/再計算してください。前提条件は呼出し側が守り、添字・符号などはdebug assertで検査します。整数の全中間値は使用型に収まる必要があります。

## Usage

```cpp
#include <iostream>
#include "graph_connectivity/block_cut_tree.hpp"
int main() {
    cp::Graph<int> g(4);g.add_edge(0,1);g.add_edge(1,2);g.add_edge(2,0);g.add_edge(1,3);
    auto r=cp::block_cut_forest(g);std::cout<<r.blocks.size()<<" "<<r.forest.size()<<" "<<r.forest.edge_count()<<"\n";
}
```

出力: `2 6 5`。

## 原理・計算量

graph/biconnected_components.hppに既に完成した公開APIがあったため、新しい入口と説明/検証だけを追加。DFSのlow[child]>=ord[parent]で辺スタックからブロックを取り出し、各所属を森の辺とする。

O(N+M)時間・領域。

## 問題例・検証・参考資料

[一次資料・対象ごとの問題例](../graph_a/references.md)。`python3 verify/run_graph_a.py` / `--sanitize` で単独/重複include、複数翻訳単位、使用例、独立した全探索/素朴解と比較します。[検証範囲](../graph_a/README.md)。オンラインACは未確認です。

[操作できる図解](../../visualizations/graph_a/index.html#block_cut_tree) / [図の範囲](../../visualizations/graph_a/README.md) / [動く例](../../examples/graph_connectivity/block_cut_tree.cpp) / [カテゴリ索引](README.md)
