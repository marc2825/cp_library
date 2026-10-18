---
title: Static Top Tree
documentation_of: //tree/static_top_tree.hpp
---

# Static Top Tree

[カテゴリ索引](README.md) / [B項目一覧](../b_7_11/README.md) / [操作できる図解](../../visualizations/b_7_11/index.html#static_top_tree) / [実装](../../tree/static_top_tree.hpp) / [一次資料と設計](../b_7_11/references.md)

固定された森をheavy pathのcompressとlight枝のrakeに分け、それぞれを平衡二分木として保持します。頂点・辺の更新で式の祖先だけを計算し直します。

## usage

```cpp
#include <iostream>
#include "tree/static_top_tree.hpp"
struct SumSpec {
    using Vertex=long long; using Path=long long; using Point=long long;
    Point point_identity() const { return 0; }
    Point rake(Point a,Point b) const { return a+b; }
    Path add_vertex(Point a,Vertex v,int) const { return a+v; }
    Path compress(Path a,Path b,cp::Edge<long long>) const { return a+b; }
    Point to_point(Path a,cp::Edge<long long>) const { return a; }
};
int main() {
    cp::Graph<long long> g(3); g.add_edge(0,1,2); g.add_edge(1,2,3);
    cp::StaticTopTreeDP<SumSpec> tree(g,{1,2,3}); tree.set_vertex(1,5);
    std::cout << tree.component_prod(0) << '\n';
}
```

出力:

```text
9
```

## API・使用条件

`StaticTopTree(g,root=0)`は公開topology。`StaticTopTreeDP<Spec,Weight>(g,values,spec,root)`、`set_vertex,get_vertex,set_edge_cost,component_prod,topology`。SpecにVertex/Path/Pointとpoint_identity/rake/add_vertex/compress/to_pointが必要。

頂点・辺IDは0始まり。グラフを受ける関数は構築時点の入力に対する計算です。重みや値の型は特記した制約に従います。

## 仕組みと境界

この版は各heavy chainとrake列を頂点個数の中点で平衡化し、高さO(log²V)を保証します。重み付き平衡化によるO(logV)版ではありません。rakeは結合的・可換、Point単位元を持つこと。compressはtop→bottom順で、接続辺の向きは親→子へ揃えます。VertexのPathはlight枝だけを含み、heavy子はcompressで繋がります。森以外は拒否。辺重みはtopology構築に影響しません。

## Specの契約

| 型・演算 | 意味 |
|---|---|
| `Vertex` | 入力valuesとset_vertexで渡す点の値。 |
| `Point point_identity()` | light枝がない頂点のPoint。 |
| `Point rake(Point a,Point b)` | 同じ境界頂点に付く、頂点集合が交わらない枝を併合。可換・結合的。 |
| `Path add_vertex(Point light,Vertex value,int v)` | light枝と頂点vをまとめたtop=bottom=vのPath。heavy子は含めない。 |
| `Path compress(Path upper,Path lower,Edge<Weight> e)` | upper.bottomからlower.topへの辺eで2つのPathを接続。境界の間にある頂点・辺を重複させない。 |
| `Point to_point(Path child,Edge<Weight> e)` | 親からchild.topへのlight辺eを追加し、親を境界とする枝へ変換。親のVertex値はここで加えない。 |

各演算は同じ元の木DPを表すよう定義します。Path/Pointに必要な情報を閉じ込めることが条件です。`component_prod(v)`は構築時の根からheavy chainの下端までを境界に持つPathなので、値の取り出し方もSpecに従います。例ではPathを単なる成分和にしています。一般の非線形DPは[比較検証](../../verify/b_7_11/tree_test.cpp)のNonlinearSpecも参照してください。

公開topologyのnodesはKind::{vertex,rake,edge,compress}、left/right/parent、top/bottom、edge_idを持ちます。元の頂点tokenは0..V−1、edge_node[id]は辺の更新先、component_root[v]は成分の式の根。元頂点IDと評価順は一致しないため、初期評価にはevaluation_orderを使用します。topologyを外部から書き換えないでください。

## 計算量

構築O(V)、領域O(V)、更新O(log²(V+1))、成分集約O(1)。 Vは頂点数、Eは論理辺数。テンプレートの比較・加減乗算・Specの各演算はO(1)として数えています。

## 使用する問題・検証

想定問題: 固定された森をheavy pathのcompressとlight枝のrakeに分け、それぞれを平衡二分木として保持します。頂点・辺の更新で式の祖先だけを計算し直します。 [実行できる小問題](../../examples/tree/static_top_tree_b.cpp)。

Static Top Treeを必要とする3.11の最適化・構造判定・更新クエリで利用します。上の使用例は実際にコンパイル・実行し、文書中のコードとの一致も検査します。

`python3 verify/run_b_7_11.py` / `--sanitize`。[検証コード](../../verify/b_7_11/)には定義からの全列挙、独立した素朴解、乱択比較と復元の整合性検査を置いています。[比較方法](../b_7_11/README.md#ローカル検証)を参照してください。オンライン提出・ACは未確認です。図解は小入力の教材モデルで、C++との差異は[図解の説明](../../visualizations/b_7_11/README.md)へ明記しています。
