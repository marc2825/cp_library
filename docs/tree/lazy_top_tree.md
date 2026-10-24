---
title: Lazy Top Tree
documentation_of: //tree/lazy_top_tree.hpp
---

# Lazy Top Tree（T-21）

link/cutを伴うTop Treeに、パス・指定根の部分木・成分全体への遅延作用を追加します。spine上の頂点と、それ以外の枝を分けてtagを保持し、exposeで枝が移動する前に保留作用を渡します。

## API・条件

`LazyTopTree<Spec>` は[Dynamic Top Tree](dynamic_top_tree.md)と同じ構築・link/cut・cluster取得APIを持ちます。さらに次を公開します。

- `path_apply(u,v,f)`：両端を含むパス上の頂点だけに作用。
- `subtree_apply(root,v,f)`：根をrootとしたv以下全体に作用。
- `component_apply(v,f)`：vの連結成分全体に作用。

前2つは非連結ならfalse、成功ならtrue。component版はvoid。根の変更、失敗操作、値コピー、例外の契約は共通です。作用素は位置・方向によらず各対象頂点へ一様に適用するものとします。

## Lazy Spec の条件

通常Specに次を加えます。`propagate` は値を書き換えるメンバ関数です。

| 型・関数 | 作用範囲 |
|---|---|
| `Lazy::id()` | 単位作用 |
| `old.propagate(newer)` | oldを先に、newerを後に適用する合成 |
| `Info::propagate(f)` | その頂点 |
| `Point::propagate(f)` | 枝全体 |
| `Path::propagate(f)` | spineだけ |
| `Path::propagate_light(f)` | off-spineだけ |

作用はrake/compress/add_edge/add_vertexによる分解と整合し、O(1)で集約値に反映できる必要があります。spineとoff-spineは互いに素な頂点集合で、それぞれへの作用は独立し、反転とも可換です。単位作用でも呼ばれます。作用合成自体の可換性・逆元は不要で、代入やaffine作用にも対応します。任意のDPに任意のlazy作用を載せられるわけではありません。

## 既存ActedMonoidの共有

`TopTreeMonoidAction<Action>` が `S,F,op,e,mapping,composition,id` を変換します。`op` は**可換**、mappingは単位元を保ち、opに分配できること。長さが必要ならSに含めます。既存の `AddSumAction`、`AssignSumAction`、`AffineSumAction` をそのまま利用可能です。

- Info / Pointの値は `.value`。
- Pathは `.spine` と `.off`、全体積は `.all()`。
- Lazyの値は `.value`。`composition(f,g)` はgの後にfという既存規約です。

## 計算量・問題例・検証

O(1)の集約・作用を前提に各操作ならしO(log n)、構築・空間O(n)。[Dynamic Tree Subtree Add Subtree Sum](https://judge.yosupo.jp/problem/dynamic_tree_subtree_add_subtree_sum)が部分木作用の問題例です。15万の混合操作で、代入・符号反転・加算の順序を、独立した頂点配列とBFSにより比較します。link/cutを跨いで作用が残る場合、root==v、非連結、長い鎖、高次数の星も含みます。

## 使用例

```cpp
#include <iostream>
#include "../../tree/lazy_top_tree.hpp"
#include "../../range_query/range_actions.hpp"
int main() {
    using Spec = cp::TopTreeMonoidAction<cp::AffineSumAction<long long>>;
    cp::LazyTopTree<Spec> tree({{{1, 1}}, {{2, 1}}, {{3, 1}}, {{4, 1}}});
    tree.link(0, 1); tree.link(1, 2); tree.link(1, 3);
    tree.path_apply(0, 2, {{2, 1}});
    tree.subtree_apply(0, 1, {{0, 5}});
    std::cout << tree.path_cluster(0, 2)->spine.sum << '\n';
    tree.cut(1, 3); tree.link(0, 3);
    std::cout << tree.component_cluster(0).all().sum << '\n';
}
```

出力:

```text
13
18
```

[操作できる図解](../../visualizations/tree_extensions/index.html#lazy_top_tree) / [検証範囲・コマンド](../tree_extensions/README.md) / [一次資料と設計判断](../tree_extensions/references.md) / [木カテゴリ](README.md)
