---
title: Radix Heap Dijkstra
documentation_of: //graph_basic/radix_heap_dijkstra.hpp
---

# Radix Heap Dijkstra

非負整数重みの最短路を既存Radix Heapと組み合わせ、元の頂点列・辺ID列を復元。

## API・使用条件

radix_heap_dijkstra(Graph<T>,start=0) -> ShortestPathResult<T>。dist/parent/parent_edge、reachable、vertex_path、edge_path。

Tはbool以外の64bit以下の整数、全辺非負。有向/無向・自己ループ・多重辺可。最大値は到達不能専用、有限距離と緩和途中は最大値未満。超過overflow_error。既存iwiwi版はGCC/Clangのbit組込みを使用。

C++17、namespace cp、0始まり。頂点・辺IDと結果の公開配列は読み取り用です。入力を変えたら再構築/再計算してください。前提条件は呼出し側が守り、添字・符号などはdebug assertで検査します。整数の全中間値は使用型に収まる必要があります。

## Usage

```cpp
#include <iostream>
#include "graph_basic/radix_heap_dijkstra.hpp"
int main() {
    cp::Graph<long long> g(3,cp::GraphKind::Directed); g.add_edge(0,1,7); g.add_edge(1,2,3); g.add_edge(0,2,15);
    auto p=cp::radix_heap_dijkstra(g); std::cout<<p.dist[2]<<" ";
    for(int v:p.vertex_path(2)) { std::cout<<v; } std::cout<<"\n";
}
```

出力: `10 012`。

## 原理・計算量

単調に取り出す距離を、直前のキーとのXORの最上位bitでバケットへ振り分ける。古い距離のエントリは捨てる。旧Radix Heapを再実装せず共有し、不足していたtupleのincludeを補い、std型のusingをradix_heap名前空間内に閉じて他カテゴリとの衝突を防いだ。

w=整数型のbit数として保守的にO((N+M)w)時間、O(N+M+w)領域。

## 問題例・検証・参考資料

[一次資料・対象ごとの問題例](../graph_a/references.md)。`python3 verify/run_graph_a.py` / `--sanitize` で単独/重複include、複数翻訳単位、使用例、独立した全探索/素朴解と比較します。[検証範囲](../graph_a/README.md)。オンラインACは未確認です。

[操作できる図解](../../visualizations/graph_a/index.html#radix_heap_dijkstra) / [図の範囲](../../visualizations/graph_a/README.md) / [動く例](../../examples/graph_basic/radix_heap_dijkstra.cpp) / [カテゴリ索引](README.md)
