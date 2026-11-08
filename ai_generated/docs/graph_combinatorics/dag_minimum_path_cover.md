---
title: DAGの最小Path Cover
documentation_of: //graph_combinatorics/dag_minimum_path_cover.hpp
---

# DAGの最小Path Cover

[カテゴリ索引](README.md) / [B項目一覧](../b_7_11/README.md) / [操作できる図解](../../visualizations/b_7_11/index.html#dag_minimum_path_cover) / [実装](../../graph_combinatorics/dag_minimum_path_cover.hpp) / [一次資料と設計](../b_7_11/references.md)

元の辺を左→右に写した二部グラフの最大マッチングを求め、頂点を一度ずつ覆うパスへ戻します。

## usage

```cpp
#include <iostream>
#include "graph_combinatorics/dag_minimum_path_cover.hpp"
int main() {
    cp::Graph<long long> g(3,cp::GraphKind::Directed);
    g.add_edge(0,1); g.add_edge(1,2);
    std::cout << cp::dag_minimum_path_cover(g).size() << '\n';
}
```

出力:

```text
1
```

## API・使用条件

`dag_minimum_path_cover(g)`→DAGPathCoverResult（paths,edge_ids,size()）。有向DAG、重み無視。閉路はinvalid_argument。孤立頂点も長さ0のパスに含む。

頂点・辺IDは0始まり。グラフを受ける関数は構築時点の入力に対する計算です。重みや値の型は特記した制約に従います。

## 仕組みと境界

最小パス数はV−最大マッチング数。推移閉包の辺は追加しないため、返す各パスは元グラフの実際の辺だけから成ります。

## 計算量

時間O(E sqrt(V)+V+E)、領域O(V+E)。 Vは頂点数、Eは論理辺数。テンプレートの比較・加減乗算・Specの各演算はO(1)として数えています。

## 使用する問題・検証

想定問題: 元の辺を左→右に写した二部グラフの最大マッチングを求め、頂点を一度ずつ覆うパスへ戻します。 [実行できる小問題](../../examples/graph_combinatorics/dag_minimum_path_cover_b.cpp)。

DAGの最小Path Coverを必要とする3.10の最適化・構造判定・更新クエリで利用します。上の使用例は実際にコンパイル・実行し、文書中のコードとの一致も検査します。

`python3 verify/run_b_7_11.py` / `--sanitize`。[検証コード](../../verify/b_7_11/)には定義からの全列挙、独立した素朴解、乱択比較と復元の整合性検査を置いています。[比較方法](../b_7_11/README.md#ローカル検証)を参照してください。オンライン提出・ACは未確認です。図解は小入力の教材モデルで、C++との差異は[図解の説明](../../visualizations/b_7_11/README.md)へ明記しています。
