---
title: 三重辺連結成分
documentation_of: //graph_connectivity/three_edge_connected_components.hpp
---

# 三重辺連結成分

[カテゴリ索引](README.md) / [B項目一覧](../b_7_11/README.md) / [操作できる図解](../../visualizations/b_7_11/index.html#three_edge_connected_components) / [実装](../../graph_connectivity/three_edge_connected_components.hpp) / [一次資料と設計](../b_7_11/references.md)

どの2本の辺を削除しても同じ連結成分に残る頂点をまとめます。橋を除いてDFSのpath absorptionを行います。

## usage

```cpp
#include <iostream>
#include "graph_connectivity/three_edge_connected_components.hpp"
int main() {
    cp::Graph<int> g(2); for(int i=0;i<3;++i) g.add_edge(0,1);
    auto c=cp::three_edge_connected_components(g);
    std::cout << c.groups.size() << '\n';
}
```

出力:

```text
1
```

## API・使用条件

`three_edge_connected_components(g)` → `ThreeEdgeConnectedComponentsResult{component,groups}`。無向（有向グラフは`invalid_argument`）、多重辺・ループ可、重み無視。成分IDはgroupsの添字。

頂点・辺IDは0始まり。グラフを受ける関数は構築時点の入力に対する計算です。重みや値の型は特記した制約に従います。

## 仕組みと境界

橋判定で親辺IDを区別し、平行辺を保ちます。ループは頂点間連結性に影響しません。LuzhiledのUnlicense版path absorptionを非再帰に移植し、pathの所有権を移して二乗コピーを避けています。

## 計算量

時間O((V+E) α(V+1))、領域O(V+E)。 Vは頂点数、Eは論理辺数。

## 使用する問題・検証

想定問題: [Library Checker: Three-Edge-Connected Components](https://judge.yosupo.jp/problem/three_edge_connected_components)。 [実行できる小問題](../../examples/graph_connectivity/three_edge_connected_components_b.cpp)。

検証では、n≤10・m≤24 の無向多重グラフ（ループを含む）600個で、全頂点対について「最大流が3以上」と同じ成分かを比較します。上の使用例は実際にコンパイル・実行し、文書中のコードとの一致も検査します。

`python3 verify/run_b_7_11.py` / `--sanitize`。[検証コード](../../verify/b_7_11/graph_test.cpp)。[比較方法](../b_7_11/README.md#ローカル検証)を参照してください。オンライン提出・ACは未確認です。図解は小入力の教材モデルで、C++との差異は[図解の説明](../../visualizations/b_7_11/README.md)へ明記しています。
