---
title: Cactus Graph分解
documentation_of: //graph_connectivity/cactus_decomposition.hpp
---

# Cactus Graph分解

[カテゴリ索引](README.md) / [B項目一覧](../b_7_11/README.md) / [操作できる図解](../../visualizations/b_7_11/index.html#cactus_decomposition) / [実装](../../graph_connectivity/cactus_decomposition.hpp) / [一次資料と設計](../b_7_11/references.md)

各辺が高々一つの単純閉路に属するかを、頂点二重連結blockごとに調べます。橋・閉路・孤立頂点をblockとして返します。

## usage

```cpp
#include <iostream>
#include "graph_connectivity/cactus_decomposition.hpp"
int main() {
    cp::Graph<int> g(4); g.add_edge(0,1); g.add_edge(1,2); g.add_edge(2,0); g.add_edge(2,3);
    auto c=cp::cactus_decomposition(g);
    std::cout << c.is_cactus << ' ' << c.blocks.size() << '\n';
}
```

出力:

```text
1 2
```

## API・使用条件

`cactus_decomposition(g)` → `{is_cactus,blocks,block_cut_forest}`。無向、多重辺・ループ可。blockにvertices/edge_ids、閉路なら閉じたcycleとcycle_edge_ids。block-cutの頂点は元のV頂点の後にblock節点。

頂点・辺IDは0始まり。グラフを受ける関数は構築時点の入力に対する計算です。重みや値の型は特記した制約に従います。

## 仕組みと境界

ループ一本も閉路、平行辺2本も閉路とします。閉路blockは辺数=頂点数かつ各内部次数2。3本以上の平行辺はcactusではありません。is_cactus=falseでもblock情報を返しますが、不正blockにcycle証明はありません。

## 計算量

時間・領域O(V+E)。 Vは頂点数、Eは論理辺数。テンプレートの比較・加減乗算・Specの各演算はO(1)として数えています。

## 使用する問題・検証

想定問題: 各辺が高々一つの単純閉路に属するかを、頂点二重連結blockごとに調べます。橋・閉路・孤立頂点をblockとして返します。 [実行できる小問題](../../examples/graph_connectivity/cactus_decomposition_b.cpp)。

Cactus Graph分解を必要とする3.8の最適化・構造判定・更新クエリで利用します。上の使用例は実際にコンパイル・実行し、文書中のコードとの一致も検査します。

`python3 verify/run_b_7_11.py` / `--sanitize`。[検証コード](../../verify/b_7_11/)には定義からの全列挙、独立した素朴解、乱択比較と復元の整合性検査を置いています。[比較方法](../b_7_11/README.md#ローカル検証)を参照してください。オンライン提出・ACは未確認です。図解は小入力の教材モデルで、C++との差異は[図解の説明](../../visualizations/b_7_11/README.md)へ明記しています。
