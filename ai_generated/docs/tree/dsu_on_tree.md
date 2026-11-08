---
title: DSU on Tree
documentation_of: //tree/dsu_on_tree.hpp
---

# DSU on Tree

[カテゴリ索引](README.md) / [図解](../../visualizations/advanced/index.html#sack) / [実装](../../tree/dsu_on_tree.hpp) / [一次資料・設計判断](../advanced/references.md)

## usage

```cpp
#include <iostream>
#include <set>
#include "tree/dsu_on_tree.hpp"
int main() {
    cp::Graph<int> g(4); g.add_edge(0, 1); g.add_edge(0, 2); g.add_edge(1, 3);
    cp::RootedTree<int> tree(g); std::set<int> bag; int result[4] = {};
    cp::dsu_on_tree(tree, [&](int v) { bag.insert(v); }, [&](int v) { bag.erase(v); },
        [&](int v) { result[v] = int(bag.size()); });
    for (int x : result) std::cout << x << ' ';
    std::cout << '\n';
}
```

[実行できる使用例](../../examples/tree/dsu_on_tree.cpp)。C++17、namespace cp。ライブラリルートをincludeパスに追加してください。

## API・使用条件

`dsu_on_tree(const RootedTree<T>&,add,remove,answer)`。add/removeは元頂点vの寄与を1回追加／削除、answer(v)時の集計にはvの部分木だけが入っています。開始前に集計を空にし、終了時も空へ戻ります。各成分の根でクリアするので森も可。removeはaddを取り消せる必要があり、部分木ごとの答えはanswerで保存します。集約は順序に依存しないものが基本です。

## 計算量・原理

前処理O(n)、各頂点O(log(n+1))回追加／削除、answerは各頂点1回。コールバックO(1)ならO(n log n)時間・O(n)補助領域。深い木でも再帰を使いません。

軽い子の結果を破棄して処理し、最も大きい子の集計だけを保持します。その後に軽い子のEuler区間を再追加します。小さい部分木を再走査するたびサイズが倍増するため、各頂点の再追加は対数回に収まります。

## 使用する問題・検証

[関連問題](https://codeforces.com/contest/600/problem/E)。関連問題は用途の例であり、このAPIだけでそのまま全て解けるという意味ではありません。特に動的連結性の関連問題は追加の成分和が必要です。

[追加分の検証入口](../../verify/run_advanced.py)で、素朴解との比較、空・重複・境界、非可換集約、履歴分岐、繰返し実行、20万要素の主要構造を検査します。実際の範囲は[検証一覧](../advanced/README.md)を参照。オンライン提出のACは未確認です。

図解は学習用の小さい入力で内部状態・候補・論理的な操作を追えます。[簡略化の範囲](../../visualizations/advanced/README.md)を参照してください。
