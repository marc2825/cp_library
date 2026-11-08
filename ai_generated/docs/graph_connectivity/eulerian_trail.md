---
title: Eulerian Trail
documentation_of: //graph_connectivity/eulerian_trail.hpp
---

# Eulerian Trail

全ての論理辺をちょうど一度使う道を頂点列と辺ID列で復元。

## API・使用条件

eulerian_trail(Graph<T>,start=-1) -> optional<EulerianTrailResult{vertices,edge_ids}>。

有向/無向、自己ループ/多重辺可、重みは無視。start=-1で自動、有効な頂点指定も可。次数条件、指定始点、辺の連結性が満たされなければnullopt。空グラフは空列、辺なしN>0は{start}（自動なら{0}）。

C++17、namespace cp、0始まり。頂点・辺IDと結果の公開配列は読み取り用です。入力を変えたら再構築/再計算してください。前提条件は呼出し側が守り、添字・符号などはdebug assertで検査します。整数の全中間値は使用型に収まる必要があります。

## Usage

```cpp
#include <iostream>
#include "graph_connectivity/eulerian_trail.hpp"
int main() {
    cp::Graph<int> g(3,cp::GraphKind::Directed);g.add_edge(0,1);g.add_edge(1,0);g.add_edge(0,2);
    auto p=cp::eulerian_trail(g);for(int v:p->vertices) { std::cout<<v; } std::cout<<"\n";
}
```

出力: `0102`。

## 原理・計算量

次数差/奇数次数から開始点を選ぶ。Hierholzer法で未使用辺を進み、出辺が尽きた頂点を終了順に積む。逆順に直し、使用辺数がMかを確認して離れた辺成分を拒否する。

O(N+M)時間・領域。深い道も再帰なし。

## 問題例・検証・参考資料

[一次資料・対象ごとの問題例](../graph_a/references.md)。`python3 verify/run_graph_a.py` / `--sanitize` で単独/重複include、複数翻訳単位、使用例、独立した全探索/素朴解と比較します。[検証範囲](../graph_a/README.md)。オンラインACは未確認です。

[操作できる図解](../../visualizations/graph_a/index.html#eulerian_trail) / [図の範囲](../../visualizations/graph_a/README.md) / [動く例](../../examples/graph_connectivity/eulerian_trail.cpp) / [カテゴリ索引](README.md)
