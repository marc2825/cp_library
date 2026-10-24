---
title: 根変更時の部分木
documentation_of: //tree/rerooted_subtree.hpp
---

# 根変更時の部分木（T-03）

固定根で作ったHLDを使い続けたまま、根を変更したときの部分木を、`hld.pos` 上の高々2区間へ変換します。木の構造やHLDは変更しません。

## API・条件

`rerooted_subtree_intervals(hld, root, v) -> vector<pair<int,int>>` は、根 `root` での頂点 `v` の部分木に属する頂点を表す昇順の半開区間です。空区間は返しません。

1. `root==v`：その連結成分全体。
2. 固定根で `v` が `root` の祖先でない：従来の `v` の部分木1区間。
3. それ以外：`v` から `root` へ向かう最初の子の部分木を、成分全体から除外した1〜2区間。

森にも対応し、別成分の `root,v` は `invalid_argument`。頂点番号は `[0,n)` が前提（既存HLDと同様、assert検査）。使用するのはHLD順であり、`RootedTree.tin` ではありません。結果の列順は固定HLD順で、根変更後のDFS順やパス順を表すものではありません。頂点値を対象とし、辺値の区間規約にはそのまま流用しないでください。

## 計算量・用途

既存HLDの構築 O(n)、追加の事前計算なし、問い合わせ O(log n)、結果の記憶領域 O(1)。子の探索はchainを登るのでLCA表を別途構築しません。FenwickやSegtreeで各区間を合算・作用すれば、根変更を伴う部分木クエリを処理できます。

## 検証

n=1..55の森で全 `root,v` を独立BFSの親配列と照合し、区間の順序・非重複・他成分の除外も確認。20万頂点の鎖、固定根と仮の根が一致する場合、隣接する根、単独成分を含みます。

## 使用例

```cpp
#include <iostream>
#include "../../tree/rerooted_subtree.hpp"
#include "../../range_query/fenwick_tree.hpp"
int main() {
    cp::Graph<int> g(5);
    g.add_edge(0, 1); g.add_edge(0, 2); g.add_edge(1, 3); g.add_edge(1, 4);
    cp::HeavyLightDecomposition<int> hld(g);
    cp::FenwickTree<long long> bit(5);
    for (int v = 0; v < 5; ++v) bit.add(hld.pos[v], v + 1);
    long long sum = 0;
    for (auto [l, r] : cp::rerooted_subtree_intervals(hld, 3, 1)) sum += bit.sum(l, r);
    std::cout << sum << '\n';
}
```

出力:

```text
11
```

[操作できる図解](../../visualizations/tree_extensions/index.html#rerooted_subtree) / [検証範囲・コマンド](../tree_extensions/README.md) / [一次資料と設計判断](../tree_extensions/references.md) / [木カテゴリ](README.md)
