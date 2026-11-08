---
title: 動的部分木集約
documentation_of: //tree/dynamic_subtree_aggregate.hpp
---

# 動的部分木集約（T-22）

[Lazy Top Tree](lazy_top_tree.md)のActedMonoidアダプタを包み、通常のS/Fで部分木集約・作用を使えるようにした公開APIです。内部実装を重複させず、パス・成分全体のAPIも同じ値と作用を共有します。

## API・条件

`DynamicSubtreeAggregate<Action>(vector<S>)` で独立頂点を作ります。`Action` は `S,F,op,e,mapping,composition,id`。opは可換、mappingは単位元を保つモノイド作用であること。逆元は不要です。

| 操作 | 意味 |
|---|---|
| `link(u,v)`, `cut(u,v)`, `connected(u,v)`, `make_root(v)` | 共通の動的森操作 |
| `get(v)`, `set(v,S)` | 頂点の値 |
| `subtree_prod(root,v) -> optional<S>` | 指定根でのv以下の積 |
| `subtree_apply(root,v,F) -> bool` | 同じ集合への作用 |
| `path_prod(u,v) -> optional<S>` / `path_apply(u,v,F)` | 両端を含むパス |
| `component_prod(v) -> S` / `component_apply(v,F)` | 成分全体 |
| `size()`, `allocated_nodes()` | サイズと確保数 |

非連結な問い合わせはnullopt、作用はfalse。存在しない辺のcut・閉路を作るlinkもfalse。添字範囲外は `out_of_range`。空の構築可、コピー不可、move可。Sの中に長さがある場合、頂点一つ分の長さを初期値・setに正しく含めます。加算・乗算の中間値は利用する数値型の範囲内としてください。

rootは任意の同一成分の頂点で、vの隣接頂点に限りません。Library Checker形式の `v,p` （pがvの親）を受け取るときは `subtree_prod(p,v)` とします。ETTのcomponent積やLCTのpath積だけでは表せない、根を指定した集合を扱います。

## 計算量・問題例・検証

構築O(n)、各操作ならしO(log n)、空間O(n)。Actionのコピー・演算費用を掛けます。[部分木加算・部分木和](https://judge.yosupo.jp/problem/dynamic_tree_subtree_add_subtree_sum)と[点加算・パス和](https://judge.yosupo.jp/problem/dynamic_tree_vertex_add_path_sum)のラッパーを用意しました。各12本の独立Python入力で辺の差し替えも含めて照合します。オンラインACは未確認です。

C++側ではaffineの非可換合成・代入を含む混合操作、全頂点値、非連結・失敗操作、破棄時のメモリ解放、10万頂点を検査します。

## 使用例

```cpp
#include <iostream>
#include "../../tree/dynamic_subtree_aggregate.hpp"
#include "../../range_query/range_actions.hpp"
int main() {
    using A = cp::AddSumAction<long long>;
    cp::DynamicSubtreeAggregate<A> tree(A::values({1, 2, 3, 4}));
    tree.link(0, 1); tree.link(1, 2); tree.link(1, 3);
    tree.subtree_apply(0, 1, 10);
    std::cout << tree.subtree_prod(0, 1)->sum << '\n';
    tree.cut(1, 3); tree.link(0, 3);
    std::cout << tree.subtree_prod(3, 0)->sum << '\n';
    std::cout << tree.path_prod(2, 3)->sum << '\n';
}
```

出力:

```text
39
26
40
```

[操作できる図解](../../visualizations/tree_extensions/index.html#dynamic_subtree_aggregate) / [検証範囲・コマンド](../tree_extensions/README.md) / [一次資料と設計判断](../tree_extensions/references.md) / [木カテゴリ](README.md)
