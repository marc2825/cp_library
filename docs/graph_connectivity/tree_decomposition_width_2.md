---
title: Tree Decomposition（幅2）
documentation_of: //graph_connectivity/tree_decomposition_width_2.hpp
---

# Tree Decomposition（幅2）

[カテゴリ索引](README.md) / [B項目一覧](../b_7_11/README.md) / [操作できる図解](../../visualizations/b_7_11/index.html#tree_decomposition_width_2) / [実装](../../graph_connectivity/tree_decomposition_width_2.hpp) / [一次資料と設計](../b_7_11/references.md)

次数2以下の頂点を消し、残る隣接頂点間へfill辺を加えます。幅2以下ならサイズ3以下のbagからなる木分解を作ります。

## usage

```cpp
#include <iostream>
#include "graph_connectivity/tree_decomposition_width_2.hpp"
int main() {
    cp::Graph<int> g(4); for(int i=0;i<4;++i) g.add_edge(i,(i+1)%4);
    auto d=cp::tree_decomposition_width_2(g);
    std::cout << bool(d) << ' ' << d->bags.size() << '\n';
}
```

出力:

```text
1 4
```

## API・使用条件

`tree_decomposition_width_2(g)` → `optional<WidthTwoTreeDecomposition>`。単純無向。幅>2ならnullopt。`bags[v]`はv消去時のbag、`elimination_order`、bag間の`forest`を返す。空グラフ可。

頂点・辺IDは0始まり。グラフを受ける関数は構築時点の入力に対する計算です。重みや値の型は特記した制約に従います。

## 仕組みと境界

残りの次数が全て3以上なら失敗。bagの親はそのbagに残る頂点のうち最初に消去される頂点です。非連結入力のため結果は森で、空bagを使って一本へ結ぶ処理は行いません。ループ・重複辺は拒否します。

## 計算量

時間O((V+E) log(V+1))、領域O(V+E)。 Vは頂点数、Eは論理辺数。テンプレートの比較・加減乗算・Specの各演算はO(1)として数えています。

## 使用する問題・検証

想定問題: 次数2以下の頂点を消し、残る隣接頂点間へfill辺を加えます。幅2以下ならサイズ3以下のbagからなる木分解を作ります。 [実行できる小問題](../../examples/graph_connectivity/tree_decomposition_width_2_b.cpp)。

Tree Decomposition（幅2）を必要とする3.8の最適化・構造判定・更新クエリで利用します。上の使用例は実際にコンパイル・実行し、文書中のコードとの一致も検査します。

`python3 verify/run_b_7_11.py` / `--sanitize`。[検証コード](../../verify/b_7_11/)には定義からの全列挙、独立した素朴解、乱択比較と復元の整合性検査を置いています。[比較方法](../b_7_11/README.md#ローカル検証)を参照してください。オンライン提出・ACは未確認です。図解は小入力の教材モデルで、C++との差異は[図解の説明](../../visualizations/b_7_11/README.md)へ明記しています。
