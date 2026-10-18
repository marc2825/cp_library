---
title: Incremental SCC
documentation_of: //graph_connectivity/incremental_scc.hpp
---

# Incremental SCC

[カテゴリ索引](README.md) / [B項目一覧](../b_7_11/README.md) / [操作できる図解](../../visualizations/b_7_11/index.html#incremental_scc) / [実装](../../graph_connectivity/incremental_scc.hpp) / [一次資料と設計](../b_7_11/references.md)

有向辺を一つずつ追加し、縮約DAGで新たにできる閉路上の成分だけを併合します。

## usage

```cpp
#include <iostream>
#include "graph_connectivity/incremental_scc.hpp"
int main() {
    cp::IncrementalSCC s(3); s.add_edge(0,1); s.add_edge(1,2); s.add_edge(2,0);
    std::cout << s.component_count() << ' ' << s.same(0,2) << '\n';
}
```

出力:

```text
1 1
```

## API・使用条件

`IncrementalSCC(n)`、`add_edge(u,v)`（成分併合があればtrue）、`same(u,v),component_count(),components()`。components()はtopological順IDの既存SCCResultを返す。辺削除なし。

頂点・辺IDは0始まり。グラフを受ける関数は構築時点の入力に対する計算です。重みや値の型は特記した制約に従います。

## 仕組みと境界

bからaへ到達する状態でa→bを追加すると、bから到達可能かつaへ到達可能な成分をまとめます。併合後は全追加辺から縮約DAGを作り直す堅実な版で、高度な償却高速版ではありません。返値snapshotは後の追加では変わりません。

## 計算量

追加最悪O(V+(V+E) log(V+1))、same/count O(1)、snapshot O(V+E)、領域O(V+E)。 Vは頂点数、Eは論理辺数。テンプレートの比較・加減乗算・Specの各演算はO(1)として数えています。

## 使用する問題・検証

想定問題: 有向辺を一つずつ追加し、縮約DAGで新たにできる閉路上の成分だけを併合します。 [実行できる小問題](../../examples/graph_connectivity/incremental_scc_b.cpp)。

Incremental SCCを必要とする3.8の最適化・構造判定・更新クエリで利用します。上の使用例は実際にコンパイル・実行し、文書中のコードとの一致も検査します。

`python3 verify/run_b_7_11.py` / `--sanitize`。[検証コード](../../verify/b_7_11/)には定義からの全列挙、独立した素朴解、乱択比較と復元の整合性検査を置いています。[比較方法](../b_7_11/README.md#ローカル検証)を参照してください。オンライン提出・ACは未確認です。図解は小入力の教材モデルで、C++との差異は[図解の説明](../../visualizations/b_7_11/README.md)へ明記しています。
