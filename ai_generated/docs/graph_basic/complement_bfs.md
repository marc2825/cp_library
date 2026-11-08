---
title: 補グラフBFS
documentation_of: //graph_basic/complement_bfs.hpp
---

# 補グラフBFS

補グラフを明示構築せず、最小辺数と頂点経路を取得。

## API・使用条件

complement_bfs(Graph<T>,start=0) -> ComplementBFSResult{source,dist,parent}、reachable(v)、vertex_path(v)。

無向、非空、有効な始点。重みと自己ループは無視、多重辺可。元に隣接しない異なる頂点を補グラフの辺で結ぶ。到達不能はdistance_inf<int>()。補辺には元の辺IDがないためedge_pathは提供しない。

C++17、namespace cp、0始まり。頂点・辺IDと結果の公開配列は読み取り用です。入力を変えたら再構築/再計算してください。前提条件は呼出し側が守り、添字・符号などはdebug assertで検査します。整数の全中間値は使用型に収まる必要があります。

## Usage

```cpp
#include <iostream>
#include "graph_basic/complement_bfs.hpp"
int main() {
    cp::Graph<int> g(4);g.add_edge(0,1);g.add_edge(1,2);g.add_edge(2,3);
    auto p=cp::complement_bfs(g); std::cout<<p.dist[2]<<" "<<p.dist[1]<<"\n";
}
```

出力: `1 2`。

## 原理・計算量

未訪問頂点を配列連結リストで保持。現在頂点の元の隣接先をマークし、未マークの未訪問頂点をまとめて取り除きBFSキューへ追加する。リストに残す走査は元の辺に対応する。

O(N+M)時間・領域。

## 問題例・検証・参考資料

[一次資料・対象ごとの問題例](../graph_a/references.md)。`python3 verify/run_graph_a.py` / `--sanitize` で単独/重複include、複数翻訳単位、使用例、独立した全探索/素朴解と比較します。[検証範囲](../graph_a/README.md)。オンラインACは未確認です。

[操作できる図解](../../visualizations/graph_a/index.html#complement_bfs) / [図の範囲](../../visualizations/graph_a/README.md) / [動く例](../../examples/graph_basic/complement_bfs.cpp) / [カテゴリ索引](README.md)
