---
title: Dial's Algorithm
documentation_of: //graph_basic/dial_shortest_path.hpp
---

# Dial's Algorithm

小さい非負整数重みの最短路を循環バケットで処理。

## API・使用条件

dial_shortest_path(Graph<T>,start,max_weight) -> ShortestPathResult<T>。

Tはbool以外の64bit以下の整数、0<=cost<=C=max_weight。Cは0..100000000（超過length_error）で、C+1バケットを確保できること。N*Cの走査時間にも注意。有向/無向可。距離の上限条件と復元は通常のDijkstraと同じ。

C++17、namespace cp、0始まり。頂点・辺IDと結果の公開配列は読み取り用です。入力を変えたら再構築/再計算してください。前提条件は呼出し側が守り、添字・符号などはdebug assertで検査します。整数の全中間値は使用型に収まる必要があります。

## Usage

```cpp
#include <iostream>
#include "graph_basic/dial_shortest_path.hpp"
int main() {
    cp::Graph<long long> g(3,cp::GraphKind::Directed); g.add_edge(0,1,7); g.add_edge(1,2,3); g.add_edge(0,2,15);
    auto p=cp::dial_shortest_path(g,0,15); std::cout<<p.dist[2]<<"\n";
}
```

出力: `10`。

## 原理・計算量

現在の距離dの候補はbucket[d%(C+1)]にある。新しい距離はd..d+Cなので、未処理の最小距離を循環させて取り出せる。C=0も扱う。

O(N(C+1)+M)時間、O(N+M+C)領域。

## 問題例・検証・参考資料

[一次資料・対象ごとの問題例](../graph_a/references.md)。`python3 verify/run_graph_a.py` / `--sanitize` で単独/重複include、複数翻訳単位、使用例、独立した全探索/素朴解と比較します。[検証範囲](../graph_a/README.md)。オンラインACは未確認です。

[操作できる図解](../../visualizations/graph_a/index.html#dial_shortest_path) / [図の範囲](../../visualizations/graph_a/README.md) / [動く例](../../examples/graph_basic/dial_shortest_path.cpp) / [カテゴリ索引](README.md)
