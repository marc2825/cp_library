---
title: DAG最短路
documentation_of: //graph_basic/dag_shortest_path.hpp
---

# DAG最短路

負辺を含むDAGの最短路と経路を一度の走査で取得。

## API・使用条件

dag_shortest_path(Graph<T>,start=0) -> optional<ShortestPathResult<T>>。

有向、非空、有効な始点。どこかに閉路があればnullopt（始点から到達しない閉路も含む）。距離演算は既存ShortestPathResultと同じ条件。

C++17、namespace cp、0始まり。頂点・辺IDと結果の公開配列は読み取り用です。入力を変えたら再構築/再計算してください。前提条件は呼出し側が守り、添字・符号などはdebug assertで検査します。整数の全中間値は使用型に収まる必要があります。

## Usage

```cpp
#include <iostream>
#include "graph_basic/dag_shortest_path.hpp"
int main() {
    cp::Graph<long long> g(3,cp::GraphKind::Directed); g.add_edge(0,1,-2); g.add_edge(1,2,3); g.add_edge(0,2,5);
    auto p=cp::dag_shortest_path(g); std::cout<<p->dist[2]<<"\n";
}
```

出力: `1`。

## 原理・計算量

トポロジカル順では入ってくる辺を先に処理できる。到達済みの頂点だけ出辺を緩和するので、負辺でも優先キューが不要。

O(N+M)時間・領域。

## 問題例・検証・参考資料

[一次資料・対象ごとの問題例](../graph_a/references.md)。`python3 verify/run_graph_a.py` / `--sanitize` で単独/重複include、複数翻訳単位、使用例、独立した全探索/素朴解と比較します。[検証範囲](../graph_a/README.md)。オンラインACは未確認です。

[操作できる図解](../../visualizations/graph_a/index.html#dag_shortest_path) / [図の範囲](../../visualizations/graph_a/README.md) / [動く例](../../examples/graph_basic/dag_shortest_path.cpp) / [カテゴリ索引](README.md)
