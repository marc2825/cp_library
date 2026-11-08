---
title: Chordal Graph判定
documentation_of: //graph_connectivity/chordal_graph.hpp
---

# Chordal Graph判定

[カテゴリ索引](README.md) / [B項目一覧](../b_7_11/README.md) / [操作できる図解](../../visualizations/b_7_11/index.html#chordal_graph) / [実装](../../graph_connectivity/chordal_graph.hpp) / [一次資料と設計](../b_7_11/references.md)

Maximum Cardinality Searchの逆順が完全消去順序かを検証します。chordalなら最適彩色と最大クリークも返します。

## usage

```cpp
#include <iostream>
#include "graph_connectivity/chordal_graph.hpp"
int main() {
    cp::Graph<long long> g(3); g.add_edge(0,1,1); g.add_edge(1,2,2); g.add_edge(0,2,4);
    auto c=cp::chordal_graph(g);
    std::cout << c.is_chordal << ' ' << c.chromatic_number << '\n';
}
```

出力:

```text
1 3
```

## API・使用条件

`chordal_graph(g)` → `ChordalGraphResult`。単純無向、重み無視。`is_chordal,perfect_elimination_order,color,maximum_clique,chromatic_number,violation`。ループ・多重辺はinvalid_argument。

頂点・辺IDは0始まり。グラフを受ける関数は構築時点の入力に対する計算です。重みや値の型は特記した制約に従います。

## 仕組みと境界

各頂点の後方隣接がクリークかを、最初の後方隣接との照合で判定します。非chordal時のviolation={v,parent,u}は消去順序への反例で、誘導閉路全体ではありません。color等の最適性はis_chordal=trueのときだけ保証します。

## 計算量

時間O((V+E) log(V+E+1))、領域O(V+E)。 Vは頂点数、Eは論理辺数。テンプレートの比較・加減乗算・Specの各演算はO(1)として数えています。

## 使用する問題・検証

想定問題: Maximum Cardinality Searchの逆順が完全消去順序かを検証します。chordalなら最適彩色と最大クリークも返します。 [実行できる小問題](../../examples/graph_connectivity/chordal_graph_b.cpp)。

Chordal Graph判定を必要とする3.8の最適化・構造判定・更新クエリで利用します。上の使用例は実際にコンパイル・実行し、文書中のコードとの一致も検査します。

`python3 verify/run_b_7_11.py` / `--sanitize`。[検証コード](../../verify/b_7_11/)には定義からの全列挙、独立した素朴解、乱択比較と復元の整合性検査を置いています。[比較方法](../b_7_11/README.md#ローカル検証)を参照してください。オンライン提出・ACは未確認です。図解は小入力の教材モデルで、C++との差異は[図解の説明](../../visualizations/b_7_11/README.md)へ明記しています。
