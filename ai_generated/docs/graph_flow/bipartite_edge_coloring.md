---
title: 二部グラフ辺彩色
documentation_of: //graph_flow/bipartite_edge_coloring.hpp
---

# 二部グラフ辺彩色

[カテゴリ索引](README.md) / [B項目一覧](../b_7_11/README.md) / [操作できる図解](../../visualizations/b_7_11/index.html#bipartite_edge_coloring) / [実装](../../graph_flow/bipartite_edge_coloring.hpp) / [一次資料と設計](../b_7_11/references.md)

最大次数Δ色で、端点を共有する辺が異なる色になるよう塗ります。追加辺の両端の空き色を、2色の交互路で揃えます。

## usage

```cpp
#include <iostream>
#include "graph_flow/bipartite_edge_coloring.hpp"
int main() {
    auto c=cp::bipartite_edge_coloring(2,2,{{0,0},{0,1},{1,0},{1,1}});
    std::cout << c.color_count << '\n';
}
```

出力:

```text
2
```

## API・使用条件

`bipartite_edge_coloring(L,R,edges)`→{color_count,color}。左・右の番号はそれぞれ0始まり。多重辺可。空の辺集合なら0色。

頂点・辺IDは0始まり。グラフを受ける関数は構築時点の入力に対する計算です。重みや値の型は特記した制約に従います。

## 仕組みと境界

2色交互路上の色を一度外してから交換します。二部性により、反対端の空き色がある始点側へは交互路が届きません。Δ正則化とEuler分割による高速版ではなく、交互路版です。

## 計算量

時間O(E(V+Δ))、領域O(VΔ+E)。 Vは頂点数、Eは論理辺数。テンプレートの比較・加減乗算・Specの各演算はO(1)として数えています。

## 使用する問題・検証

想定問題: 最大次数Δ色で、端点を共有する辺が異なる色になるよう塗ります。追加辺の両端の空き色を、2色の交互路で揃えます。 [実行できる小問題](../../examples/graph_flow/bipartite_edge_coloring_b.cpp)。

関連する出題: [Edge Coloring of Bipartite Graph](https://judge.yosupo.jp/problem/bipartite_edge_coloring)。入出力仕様の参照先であり、オンラインACの主張ではありません。

二部グラフ辺彩色を必要とする3.9の最適化・構造判定・更新クエリで利用します。上の使用例は実際にコンパイル・実行し、文書中のコードとの一致も検査します。

`python3 verify/run_b_7_11.py` / `--sanitize`。[検証コード](../../verify/b_7_11/)には定義からの全列挙、独立した素朴解、乱択比較と復元の整合性検査を置いています。[比較方法](../b_7_11/README.md#ローカル検証)を参照してください。オンライン提出・ACは未確認です。図解は小入力の教材モデルで、C++との差異は[図解の説明](../../visualizations/b_7_11/README.md)へ明記しています。
