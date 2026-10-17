---
title: 重み付き経路の結果型
documentation_of: //graph_basic/weighted_path.hpp
---

# 重み付き経路の結果型

K-shortestの単純経路とwalkの共通の値型。

## API・使用条件

WeightedPath<T>{cost,vertices,edge_ids}。

verticesは始点/終点を含み、辺数+1個。辺IDは元グラフの論理辺ID。空経路でもvertices={s}、edge_idsは空。単純性は型自体では保証せず、生成したAPIの契約に従う。

C++17、namespace cp、0始まり。頂点・辺IDと結果の公開配列は読み取り用です。入力を変えたら再構築/再計算してください。前提条件は呼出し側が守り、添字・符号などはdebug assertで検査します。整数の全中間値は使用型に収まる必要があります。

## Usage

```cpp
#include <iostream>
#include "graph_basic/weighted_path.hpp"
int main() {
    cp::WeightedPath<long long> p{5,{0,1},{7}};
    std::cout<<p.cost<<" "<<p.vertices.size()<<" "<<p.edge_ids[0]<<"\n";
}
```

出力: `5 2 7`。

## 原理・計算量

結果を値として保存するだけの型。アルゴリズムはk_shortest_pathsとk_shortest_walksを参照。

経路長Lの保持にO(L)領域。

## 問題例・検証・参考資料

[一次資料・対象ごとの問題例](../graph_a/references.md)。`python3 verify/run_graph_a.py` / `--sanitize` で単独/重複include、複数翻訳単位、使用例、独立した全探索/素朴解と比較します。[検証範囲](../graph_a/README.md)。オンラインACは未確認です。

[操作できる図解](../../visualizations/graph_a/index.html#k_shortest_paths) / [図の範囲](../../visualizations/graph_a/README.md) / [動く例](../../examples/graph_basic/weighted_path.cpp) / [カテゴリ索引](README.md)
