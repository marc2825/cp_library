---
title: Virtual Tree
documentation_of: //tree/virtual_tree.hpp
---

# Virtual Tree

[カテゴリ索引](README.md) / [図解](../../visualizations/advanced/index.html#virtual) / [実装](../../tree/virtual_tree.hpp) / [一次資料・設計判断](../advanced/references.md)

## usage

```cpp
#include <iostream>
#include "tree/virtual_tree.hpp"
int main() {
    cp::Graph<int> g(5); g.add_edge(0, 1); g.add_edge(1, 2); g.add_edge(1, 3); g.add_edge(3, 4);
    cp::VirtualTree<int> tree(g); auto v = tree.build({2, 4, 4});
    for (int i = 0; i < int(v.vertices.size()); ++i) std::cout << v.vertices[i] << ':' << v.parent[i] << ' ';
    std::cout << '\n';
}
```

[実行できる使用例](../../examples/tree/virtual_tree.cpp)。C++17、namespace cp。ライブラリルートをincludeパスに追加してください。

## API・使用条件

`VirtualTree<T>(Graph<T>,root=0)`、公開 `lca: DoublingLCA<T>`、`build(vector<int> important) const -> VirtualTreeResult`。元グラフは無向の森で閉路は拒否、RootedTreeと同じ距離算術条件。重要頂点の空・重複・別成分に対応。

結果 `vertices` はlocal ID→元頂点、`parent` と `roots` はlocal ID、根のparentは−1。`marked`は元の重要頂点か。`edge_length`は縮約辺に含まれる元の辺数、根では0。重み距離が必要なら公開lcaの `distance(parent_vertex,vertex)` を呼びます。

## 計算量・原理

前処理O(n log n)時間・領域。k個の重要頂点についてO(k log k+k log n)時間、結果O(k)、局所追加領域O(k)。構築後の元グラフ変更は反映しません。

DFS順で隣り合う重要頂点のLCAを加えるだけで、必要な分岐点を全て含められます。再びDFS順に並べ、祖先スタックで最も近い残した祖先へつなぎます。重要頂点のない成分は出力しません。

## 使用する問題・検証

[関連問題](https://atcoder.jp/contests/typical90/tasks/typical90_ai)。関連問題は用途の例であり、このAPIだけでそのまま全て解けるという意味ではありません。特に動的連結性の関連問題は追加の成分和が必要です。

[追加分の検証入口](../../verify/run_advanced.py)で、素朴解との比較、空・重複・境界、非可換集約、履歴分岐、繰返し実行、20万要素の主要構造を検査します。実際の範囲は[検証一覧](../advanced/README.md)を参照。オンライン提出のACは未確認です。

図解は学習用の小さい入力で内部状態・候補・論理的な操作を追えます。[簡略化の範囲](../../visualizations/advanced/README.md)を参照してください。
