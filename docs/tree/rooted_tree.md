---
title: 根付き木・部分木サイズ・深さ
documentation_of: //tree/rooted_tree.hpp
---

# 根付き木・部分木サイズ・深さ

無向の木・森から、親・深さ・部分木区間を一度に作ります。再帰を使わず、20万頂点のパスも処理できます。重み付き距離と辺数としての深さは別々に保持します。

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/tree/index.html#rooted) / [実装](../../tree/rooted_tree.hpp) / [参考資料](references.md)

## usage

```cpp
#include <iostream>
#include "tree/rooted_tree.hpp"
int main() {
    cp::Graph<> g(4);
    g.add_edge(0, 1); g.add_edge(1, 2); g.add_edge(1, 3);
    cp::RootedTree<> tree(g);
    auto [l, r] = tree.subtree_interval(1);
    std::cout << tree.depth[2] << ' ' << tree.subtree_size[1] << ' ' << l << ' ' << r << '\n';
    for (int v : tree.postorder) std::cout << v << ' ';
    std::cout << '\n';
}
```

[動く使用例](../../examples/tree/rooted_tree.cpp)。リポジトリのルートを include パスに指定します。

## API・使用条件

| API / フィールド | 意味 |
|---|---|
| `RootedTree<T>(g,root=0)` | 指定した根の成分から DFS。残る成分は最小番号の頂点が根 |
| `roots` / `component[v]` | 処理順の根一覧 / 0始まりの成分ID |
| `parent[v]` / `parent_edge[v]` | 親頂点 / 親への元辺ID。各成分の根は両方 `-1` |
| `depth[v]` / `distance_from_root[v]` | 根からの辺数 / 重み和 |
| `preorder` / `postorder` | 行きがけ / 帰りがけの頂点列。各頂点1回 |
| `tin[v]` / `tout[v]` | preorder 上の部分木の半開区間 |
| `subtree_size[v]` | 自身を含む部分木サイズ。`tout[v]-tin[v]` と一致 |
| `size()` | 頂点数 |
| `same_component(u,v)` | 同一成分か |
| `is_ancestor(u,v)` | u が v の祖先か。自分自身を含む。別成分は false |
| `subtree_interval(v)` | `{tin[v],tout[v]}` |

空の森では全配列が空で、root は参照しません。非空なら有効な root を渡します。有向・閉路・自己ループ・多重辺は `invalid_argument`。頂点番号の範囲は assert で確認します。構築後の配列はスナップショットで、元の Graph を破棄しても使えます。公開配列を直接変更すると他の配列との整合性を保てません。

## 原理・計算量・注意点

構築 O(N+E)、空間 O(N)、問い合わせ O(1)。森では E<N なので O(N) です。

DFS で子へ入った位置から、その子から戻る直前までの新しい頂点がちょうど部分木です。帰りがけに子のサイズを親へ加えます。postorder は子が必ず親より前なので、木DPのループにも使えます。

根からの重み和の加算は既存の距離算術を使います。型 T は標準の整数／浮動小数点型。中間値を含め型の範囲内かつ最大値未満が条件で、範囲外は `overflow_error`。負辺・0重みは許容します。

## 使用する問題・検証

[Vertex Add Subtree Sum](https://judge.yosupo.jp/problem/vertex_add_subtree_sum)。[Euler Tour](euler_tour.md) と Fenwick の併用例も参照。

再帰 DFS の全配列、339個の非空な小さな森を全ての根で構築、300ランダム入力、空・閉路・距離超過・20万頂点。

共通の [検証入口](../../verify/run_tree.py) は C++17・警告エラー・debug/release・単独 include・複数翻訳単位・使用例・独立オラクル・ジャッジ入出力を確認します。`--sanitize` で ASan/UBSan。**オンライン AC は未確認**です。
