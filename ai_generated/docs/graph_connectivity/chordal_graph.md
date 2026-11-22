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

`chordal_graph(g)` → `ChordalGraphResult`。単純無向（有向グラフも`invalid_argument`）、重み無視。`is_chordal,perfect_elimination_order,color,maximum_clique,chromatic_number,violation`。ループ・多重辺はinvalid_argument。

頂点・辺IDは0始まり。グラフを受ける関数は構築時点の入力に対する計算です。重みや値の型は特記した制約に従います。

## 仕組みと境界

各頂点の後方隣接がクリークかを、最初の後方隣接との照合で判定します。非chordal時のviolation={v,parent,u}は消去順序への反例で、誘導閉路全体ではありません。is_chordal=falseのときは最初の反例で打ち切るため、color・maximum_clique は空、chromatic_number=0 で、perfect_elimination_order は MCS の逆順（完全消去順序ではない）です。

## 計算量

時間O((V+E) log(V+E+1))、領域O(V+E)。 Vは頂点数、Eは論理辺数。

## 使用する問題・検証

想定問題: 弦グラフの判定と、弦グラフ上の最大クリーク・最小彩色（[Library Checker: Chordal Graph Recognition](https://judge.yosupo.jp/problem/chordal_graph_recognition) は非弦グラフのとき長さ4以上の誘導閉路の出力が必要で、この API の `violation` は3頂点の反例だけなので、そのままでは解けません）。 [実行できる小問題](../../examples/graph_connectivity/chordal_graph_b.cpp)。

検証では、n≤7 の単純無向グラフ450個で、長さ4以上の誘導閉路の全列挙と `is_chordal` を比較し、弦グラフなら全探索した最大クリークと `chromatic_number` の一致、`color` が正しい彩色であることを確かめます。上の使用例は実際にコンパイル・実行し、文書中のコードとの一致も検査します。

`python3 verify/run_b_7_11.py` / `--sanitize`。[検証コード](../../verify/b_7_11/graph_test.cpp)。[比較方法](../b_7_11/README.md#ローカル検証)を参照してください。オンライン提出・ACは未確認です。図解は小入力の教材モデルで、C++との差異は[図解の説明](../../visualizations/b_7_11/README.md)へ明記しています。
